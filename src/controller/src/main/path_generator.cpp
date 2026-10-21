// =====================================================================
// path_generator.cpp : interactive goal-to-path generator.
//
// Asks the user for a goal (x, y, theta) RELATIVE TO THE ROBOT CENTER,
// builds a dense straight-line path from the robot's current pose to that
// goal, and publishes it on /plan for path_tracker (or path_tracker_sim).
//
// The origin is ALWAYS the robot center (base_footprint) at the moment you
// press Enter, wherever the robot is on the map. Every new goal is again
// measured from where the robot is now.
//
// User frame (what you type):
//     x      metres, + forward
//     y      metres, + to the RIGHT
//     theta  degrees, + CLOCKWISE, - counter clockwise, range [-180, 180]
//
// ROS frame (what is published, REP 103): x forward, y LEFT, yaw CCW.
// So internally: y_ros = -y, yaw_ros = -theta.
//
// Path: poses every point_spacing metres (0.05 m, same as path_planner_v2.py) on the straight line from
// the robot to the goal. The heading turns smoothly from the current
// heading to the goal heading along the way, in the direction you asked
// (+ CW, - CCW). For a pure rotation (x = y = 0) the robot turns in place.
//
//   subscribes : TF  map_frame -> base_frame   (robot pose)
//   publishes  : /plan  nav_msgs/msg/Path      (frame map_frame)
//
// Run (with bringup.launch.py mode:=auto or sim.launch.py running):
//     ros2 run controller path_generator
//
// Parameters: map_frame ("map"), base_frame ("base_footprint"),
//             point_spacing (0.05 m), republish_rate (1.0 Hz)
// =====================================================================

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace
{
double wrap(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

double deg2rad(double d) { return d * M_PI / 180.0; }
double rad2deg(double r) { return r * 180.0 / M_PI; }

// Read one number from stdin. Returns false on EOF or when the user
// types q / quit.
bool ask_number(const std::string& prompt, double& out)
{
    while (true) {
        std::cout << prompt << std::flush;
        std::string line;
        if (!std::getline(std::cin, line)) return false;      // Ctrl+D
        // trim
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        if (line == "q" || line == "quit" || line == "exit") return false;
        if (line.empty()) { out = 0.0; return true; }         // Enter = 0
        std::istringstream ss(line);
        double v;
        if (ss >> v && ss.eof() && std::isfinite(v)) { out = v; return true; }
        std::cout << "  not a number, try again (or q to quit)\n";
    }
}
}  // namespace

class PathGenerator : public rclcpp::Node
{
public:
    PathGenerator() : Node("path_generator")
    {
        map_frame_      = declare_parameter<std::string>("map_frame", "map");
        base_frame_     = declare_parameter<std::string>("base_frame", "base_footprint");
        spacing_        = declare_parameter<double>("point_spacing", 0.05);
        republish_rate_ = declare_parameter<double>("republish_rate", 1.0);
        if (spacing_ < 0.005) spacing_ = 0.005;

        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        pub_ = create_publisher<nav_msgs::msg::Path>("/plan", 10);

        // Republish the current path so a tracker started late still gets
        // it. The tracker ignores an identical path, so progress is kept.
        if (republish_rate_ > 0.0) {
            timer_ = create_wall_timer(
                std::chrono::duration<double>(1.0 / republish_rate_),
                [this]() {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (has_path_) pub_->publish(path_);
                });
        }
    }

    // Current robot pose in the map frame (ROS convention). False if the
    // TF is not available yet.
    bool robot_pose(double& x, double& y, double& yaw)
    {
        try {
            const auto t = tf_buffer_->lookupTransform(
                map_frame_, base_frame_, tf2::TimePointZero,
                tf2::durationFromSec(1.0));
            x = t.transform.translation.x;
            y = t.transform.translation.y;
            const auto& q = t.transform.rotation;
            yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            return true;
        } catch (const tf2::TransformException& ex) {
            std::cout << "  cannot read " << map_frame_ << " -> " << base_frame_
                      << " (" << ex.what() << ")\n"
                      << "  is bringup (mode:=auto) or the sim running?\n";
            return false;
        }
    }

    // Build and publish the path from the robot's current pose to a goal
    // given in the USER frame (x fwd, y right, theta deg CW+).
    bool send_goal(double gx_user, double gy_user, double gth_user_deg)
    {
        double rx, ry, ryaw;
        if (!robot_pose(rx, ry, ryaw)) return false;

        // user frame -> ROS body frame (y left, yaw CCW+)
        const double bx   = gx_user;
        const double by   = -gy_user;
        const double dyaw = -deg2rad(gth_user_deg);     // signed turn, ROS sense

        // body frame -> map frame, about the robot center
        const double c = std::cos(ryaw), s = std::sin(ryaw);
        const double gx = rx + c * bx - s * by;
        const double gy = ry + s * bx + c * by;

        const double dist = std::hypot(gx - rx, gy - ry);
        // number of segments; a pure rotation still gets 2 poses (the
        // tracker needs >= 2) at the same spot, so it turns in place
        const int n = (dist < 1e-3) ? 1
                    : std::max(1, int(std::ceil(dist / spacing_)));

        nav_msgs::msg::Path msg;
        msg.header.stamp    = now();
        msg.header.frame_id = map_frame_;
        msg.poses.reserve(n + 1);
        for (int k = 0; k <= n; ++k) {
            const double t = double(k) / n;
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = rx + t * (gx - rx);
            ps.pose.position.y = ry + t * (gy - ry);
            // heading turns by the SIGNED amount the user asked for
            const double yaw = wrap(ryaw + t * dyaw);
            ps.pose.orientation.z = std::sin(yaw / 2.0);
            ps.pose.orientation.w = std::cos(yaw / 2.0);
            msg.poses.push_back(ps);
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            path_ = msg;
            has_path_ = true;
            pub_->publish(path_);
        }

        // printed in the ROS map frame (y left, yaw CCW+), the same numbers
        // you see in RViz
        std::printf("  from map (%.2f, %.2f, yaw %.1f deg)  to map (%.2f, %.2f, yaw %.1f deg)\n",
                    rx, ry, rad2deg(ryaw), gx, gy, rad2deg(wrap(ryaw + dyaw)));
        std::printf("  %.2f m, %d poses  (map frame is ROS: y left, yaw CCW+)\n",
                    dist, int(msg.poses.size()));
        return true;
    }

private:
    std::string map_frame_, base_frame_;
    double spacing_ {0.05};
    double republish_rate_ {1.0};

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex          mutex_;
    nav_msgs::msg::Path path_;
    bool                has_path_ {false};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathGenerator>();

    // spin ROS (TF + republish timer) in the background, read the
    // keyboard in the foreground
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    std::thread spinner([&exec]() { exec.spin(); });

    std::cout <<
        "\n================ path_generator ================\n"
        " Goal is measured from the ROBOT CENTER, right now:\n"
        "   x     [m]   + forward\n"
        "   y     [m]   + right\n"
        "   theta [deg] + clockwise, - counter clockwise (-180..180)\n"
        " Empty input = 0.  A new goal replaces the current one.\n"
        " Type q to quit.\n"
        "================================================\n";

    while (rclcpp::ok()) {
        double x, y, th;
        std::cout << "\nNew goal\n";
        if (!ask_number("  x     [m]  : ", x))  break;
        if (!ask_number("  y     [m]  : ", y))  break;
        if (!ask_number("  theta [deg]: ", th)) break;

        if (th < -180.0 || th > 180.0) {
            std::cout << "  theta must be in [-180, 180]. For a bigger turn, "
                         "send it as two goals.\n";
            continue;
        }
        if (node->send_goal(x, y, th))
            std::cout << "  path published on /plan\n";
    }

    std::cout << "bye\n";
    exec.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}
