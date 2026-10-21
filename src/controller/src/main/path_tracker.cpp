// =====================================================================
// path_tracker.cpp -- PATH FOLLOWER (publishes /cmd_vel only).
//
// This node does NOT drive motors. It reads the robot pose from the
// map -> base_footprint TF, follows the /plan path with path tracker,
// and publishes the desired body velocity as geometry_msgs/Twist on
// /cmd_vel. A separate node (robot_controller) subscribes to /cmd_vel,
// runs the swerve inverse kinematics, and drives the CAN motors.
//
// Split of concerns:
//     path + pose  ->  path_tracker  ->  /cmd_vel (Twist)
//     /cmd_vel     ->  robot_controller  ->  CAN motors + /joint_states
//
//   subscribes : /plan   nav_msgs/msg/Path        (must be in map_frame)
//                TF      map_frame -> base_frame   (the pose)
//   publishes  : /cmd_vel          geometry_msgs/msg/Twist
//                /lookahead_point  geometry_msgs/msg/PoseStamped (debug)
//
// Twist convention: linear.x = vx_body, linear.y = vy_body,
// angular.z = omega, all in the robot (base) frame.
//
// path tracker parameters load from path_tracker_config.yaml (/path_tracker).
// =====================================================================

#include "Path_Tracker.hpp"
#include "Path_Generation.hpp"   // Path struct type only

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <algorithm>
#include <mutex>
#include <string>
#include <vector>
#include <cstddef>

static constexpr double CONTROL_FREQ = 100.0;

static inline double yaw_from_quat(double x, double y, double z, double w)
{
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
static inline double yaw_of(const geometry_msgs::msg::Quaternion& q)
{
    return yaw_from_quat(q.x, q.y, q.z, q.w);
}
// normalize_angle is declared in Path_Generation.hpp (included above) and
// defined in the controller library, so we use that one rather than
// redefining it here (a static redefinition clashes with the extern decl).

class PathTrackerNode : public rclcpp::Node
{
public:
    PathTrackerNode() : Node("path_tracker")
    {
        // ---- frames / TF ----
        map_frame_    = declare_parameter<std::string>("map_frame", "map");
        base_frame_   = declare_parameter<std::string>("base_frame", "base_footprint");
        tf_poll_freq_ = declare_parameter<double>("tf_poll_freq", 20.0);
        strict_frame_ = declare_parameter<bool>("strict_frame", true);
        pose_timeout_ = declare_parameter<double>("pose_timeout", 0.25);

        // ---- path tracker ----
        pt_ = {};
        pt_.CONTROLLER_FREQUENCY = CONTROL_FREQ;
        declare_pt_params();
        load_pt_params();
        param_init(&init_);

        param_cb_handle_ = add_on_set_parameters_callback(
            std::bind(&PathTrackerNode::on_param_change, this,
                      std::placeholders::_1));

        // ---- TF ----
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // ---- pubs / subs ----
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        la_pub_  = create_publisher<geometry_msgs::msg::PoseStamped>("/lookahead_point", 10);

        plan_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/plan", rclcpp::QoS(10),
            std::bind(&PathTrackerNode::plan_cb, this, std::placeholders::_1));

        // poll pose at tf_poll_freq (double FAST-LIO's rate to avoid aliasing)
        pose_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / tf_poll_freq_),
            std::bind(&PathTrackerNode::poll_pose, this));

        // control at 100 Hz
        control_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / CONTROL_FREQ),
            std::bind(&PathTrackerNode::control_step, this));

        RCLCPP_INFO(get_logger(),
                    "path_tracker (path follower) started. Reads %s -> %s, "
                    "follows /plan, publishes /cmd_vel.",
                    map_frame_.c_str(), base_frame_.c_str());
    }

private:
    // ---- parameter plumbing ----
    void declare_pt_params()
    {
        declare_parameter<double>("min_lookahead_distance", 0.1);
        declare_parameter<double>("max_lookahead_distance", 1.0);
        declare_parameter<double>("lookahead_time", 0.26);
        declare_parameter<double>("desired_linear_velocity", 1.0);
        declare_parameter<double>("kp_heading", 2.0);
        declare_parameter<double>("max_linear_acceleration", 0.25);
        declare_parameter<double>("approach_velocity_scaling_distance", 0.7);
        declare_parameter<double>("minimum_approach_linear_velocity", 0.05);
        declare_parameter<double>("minimum_linear_velocity", 0.0);
        declare_parameter<double>("maximum_angular_velocity", 0.5);
        declare_parameter<double>("maximum_angular_acceleration", 0.4);
        declare_parameter<double>("distance_goal_tolerance", 0.15);
        declare_parameter<double>("yaw_goal_tolerance", 0.1);
        declare_parameter<bool>("corner_detection", false);
        declare_parameter<double>("corner_velocity", 0.4);
        declare_parameter<double>("min_corner_slowdown_distance", 0.5);
        declare_parameter<double>("corner_decel", 2.0);
        declare_parameter<double>("corner_angle_threshold", 0.35);
    }

    void load_pt_params()
    {
        pt_.MIN_LOOKAHEAD_DISTANCE             = get_parameter("min_lookahead_distance").as_double();
        pt_.MAX_LOOKAHEAD_DISTANCE             = get_parameter("max_lookahead_distance").as_double();
        pt_.LOOKAHEAD_TIME                     = get_parameter("lookahead_time").as_double();
        pt_.V                                  = get_parameter("desired_linear_velocity").as_double();
        pt_.KP_HEADING                         = get_parameter("kp_heading").as_double();
        pt_.MAXIMUM_LINEAR_ACCELERATION        = get_parameter("max_linear_acceleration").as_double();
        pt_.APPROACH_VELOCITY_SCALING_DISTANCE = get_parameter("approach_velocity_scaling_distance").as_double();
        pt_.MINIMUM_APPROACH_LINEAR_VELOCITY   = get_parameter("minimum_approach_linear_velocity").as_double();
        pt_.MINIMUM_LINEAR_VELOCITY            = get_parameter("minimum_linear_velocity").as_double();
        pt_.MAXIMUM_ANGULAR_VELOCITY           = get_parameter("maximum_angular_velocity").as_double();
        pt_.MAXIMUM_ANGULAR_ACCELERATION       = get_parameter("maximum_angular_acceleration").as_double();
        pt_.DISTANCE_GOAL_TOLERANCE            = get_parameter("distance_goal_tolerance").as_double();
        pt_.YAW_GOAL_TOLERANCE                 = get_parameter("yaw_goal_tolerance").as_double();
        pt_.CORNER_DETECTION                   = get_parameter("corner_detection").as_bool();
        pt_.CORNER_VELOCITY                    = get_parameter("corner_velocity").as_double();
        pt_.MIN_CORNER_SLOWDOWN_DISTANCE           = get_parameter("min_corner_slowdown_distance").as_double();
        pt_.CORNER_DECEL                       = get_parameter("corner_decel").as_double();
        pt_.CORNER_ANGLE_THRESHOLD             = get_parameter("corner_angle_threshold").as_double();
    }

    rcl_interfaces::msg::SetParametersResult
    on_param_change(const std::vector<rclcpp::Parameter>& /*params*/)
    {
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            load_pt_params();
        }
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = true;
        RCLCPP_INFO(get_logger(), "parameters updated live");
        return r;
    }

    // ---- path intake ----
    void plan_cb(nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.size() < 2) {
            RCLCPP_WARN(get_logger(), "ignoring /plan with < 2 poses");
            return;
        }
        std::lock_guard<std::mutex> lock(path_mutex_);

        const std::size_t n = msg->poses.size();
        if (have_path_ && path_.x.size() == n) {
            bool same = true;
            for (std::size_t i = 0; i < n; ++i) {
                const double th = yaw_of(msg->poses[i].pose.orientation);
                if (std::fabs(path_.x[i] - msg->poses[i].pose.position.x) > 1e-6 ||
                    std::fabs(path_.y[i] - msg->poses[i].pose.position.y) > 1e-6 ||
                    std::fabs(normalize_angle(path_.theta[i] - th)) > 1e-6) {
                    same = false; break;
                }
            }
            if (same) return;   // periodic republish, keep progress
        }

        path_.x.clear(); path_.y.clear(); path_.theta.clear();
        path_.x.reserve(n); path_.y.reserve(n); path_.theta.reserve(n);
        for (const auto& ps : msg->poses) {
            path_.x.push_back(ps.pose.position.x);
            path_.y.push_back(ps.pose.position.y);
            path_.theta.push_back(yaw_of(ps.pose.orientation));
        }
        path_frame_  = msg->header.frame_id;
        have_path_   = true;
        search_from_ = 0;
        all_done_    = false;
        param_init(&init_);

        RCLCPP_INFO(get_logger(),
                    "new /plan accepted: %zu poses in frame '%s'",
                    n, path_frame_.c_str());
    }

    // ---- pose polling from TF ----
    void poll_pose()
    {
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform(map_frame_, base_frame_,
                                            tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "waiting for %s -> %s (%s)",
                                 map_frame_.c_str(), base_frame_.c_str(), ex.what());
            return;
        }

        const double x  = t.transform.translation.x;
        const double y  = t.transform.translation.y;
        const double th = yaw_of(t.transform.rotation);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(th)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "non-finite pose, ignoring");
            return;
        }

        std::lock_guard<std::mutex> lock(fb_mutex_);
        pose_x_ = x; pose_y_ = y; pose_theta_ = th;
        last_pose_time_ = t.header.stamp;
        if (!have_pose_) {
            have_pose_ = true;
            RCLCPP_INFO(get_logger(), "first pose from %s -> %s",
                        map_frame_.c_str(), base_frame_.c_str());
        }
    }

    // ---- publish a zero twist (stop) ----
    void publish_stop()
    {
        geometry_msgs::msg::Twist tw;   // all zeros
        cmd_pub_->publish(tw);
    }

    // ---- 100 Hz control: path -> /cmd_vel ----
    void control_step()
    {
        double px, py, pth;
        bool   pose_ok;
        {
            std::lock_guard<std::mutex> lock(fb_mutex_);
            pose_ok = have_pose_ &&
                      (now() - last_pose_time_).seconds() < pose_timeout_;
            px = pose_x_; py = pose_y_; pth = pose_theta_;
        }

        if (!pose_ok) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "no fresh %s -> %s within %.2f s, publishing stop",
                                 map_frame_.c_str(), base_frame_.c_str(), pose_timeout_);
            publish_stop();
            return;
        }

        float vx = 0.0f, vy = 0.0f, omega = 0.0f;

        {
            std::lock_guard<std::mutex> lock(path_mutex_);

            if (!have_path_ || all_done_) {
                publish_stop();
                return;
            }

            if (strict_frame_ && path_frame_ != map_frame_) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                    "path is in '%s' but the pose is in '%s'. HOLDING STILL. "
                    "Set frame_id: \"map\" in path_config_v2.yaml.",
                    path_frame_.c_str(), map_frame_.c_str());
                publish_stop();
                return;
            }

            const int n_points = int(path_.x.size());
            const double x_now = px, y_now = py, th = pth;

            const double L = get_lookahead_distance(&pt_, &init_);
            double x_L, y_L, theta_L;
            int    closest_idx;
            get_lookahead_point(path_.x.data(), path_.y.data(), path_.theta.data(),
                                n_points, search_from_, x_now, y_now, L,
                                &x_L, &y_L, &theta_L, &closest_idx);
            if (closest_idx > search_from_) search_from_ = closest_idx;
            publish_lookahead(x_L, y_L, theta_L);

            const double d_g   = get_goal_distance(path_.x.data(), path_.y.data(),
                                                   n_points, x_now, y_now);
            const double l_rem = get_remaining_path_length(path_.x.data(), path_.y.data(),
                                                           n_points, closest_idx);
            const double d     = get_lookahead_distance_to_point(x_L, y_L, x_now, y_now);

            const bool path_consumed = search_from_ >= n_points - 2;
            // For a CLOSED loop the last path point coincides with the
            // start, so d_g (distance to the last point) is ~0 at launch and
            // the goal/approach logic would misfire, making the robot creep
            // forward then reverse. Gate "near the goal" on actual progress:
            // only treat the goal as reachable once the robot has traversed
            // most of the path. Before that, force the goal far away so the
            // approach-scaling and position-done cannot trigger at the start.
            const bool near_end = search_from_ >= (n_points * 3) / 4;
            const double d_g_eff = near_end ? d_g : 1e3;
            const double position_done = d_g_eff < pt_.DISTANCE_GOAL_TOLERANCE;
            const bool heading_done  =
                std::fabs(normalize_angle(path_.theta.back() - th)) < pt_.YAW_GOAL_TOLERANCE;

            const double head_cmd_err = normalize_angle(theta_L - th);
            if (std::fabs(head_cmd_err) > 2.0) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "lookahead commands a %.0f deg turn at (%.2f, %.2f), idx %d/%d. "
                    "Check the path does not pass close to itself here.",
                    head_cmd_err * 180.0 / M_PI, x_now, y_now, closest_idx, n_points);
            }

            if (path_consumed && position_done && heading_done) {
                all_done_ = true;
                RCLCPP_INFO(get_logger(), "path complete (goal reached)");
                publish_stop();
                return;
            }
            else if (path_consumed && position_done && !heading_done) {
                // rotate-in-place at the final waypoint: no path motion, so
                // no feedforward -- just the proportional term rotates to
                // the goal heading.
                omega = float(get_angular_velocity(&pt_, &init_,
                                                   path_.theta.back(), th,
                                                   pt_.MIN_LOOKAHEAD_DISTANCE,
                                                   pt_.V, 0.0));
                vx = 0.0f; vy = 0.0f;
            }
            else {
                const double v_cap = get_corner_speed_limit(&pt_, &init_,
                                        path_.x.data(), path_.y.data(),
                                        n_points, closest_idx);
                const double v  = get_linear_velocity(&pt_, &init_, d_g_eff, l_rem, v_cap);
                // Heading reference is the path heading at the robot's
                // CURRENT position on the path (closest point), not the
                // look-ahead point. This makes the robot track the heading
                // intended for each pose it is actually at, so per-pose
                // heading is followed precisely. The look-ahead point still
                // drives TRANSLATION (look-ahead tracking); only the heading reference
                // uses the closest point.
                const int hidx = (closest_idx < n_points) ? closest_idx : n_points - 1;
                const double theta_ref = path_.theta[hidx];
                // feedforward = path heading rate at the current point,
                // scaled by the current speed
                const double om_ff = get_heading_feedforward(
                                        path_.theta.data(), hidx,
                                        n_points, path_.x.data(),
                                        path_.y.data(), v);
                const double om = get_angular_velocity(&pt_, &init_, theta_ref, th,
                                                       d, init_.V_PREV, om_ff);
                get_world_velocity(&init_, v, d, x_L, y_L, x_now, y_now);
                double vx_body, vy_body;
                world_to_robot_frame_velocity(init_.VX_WORLD, init_.VY_WORLD, th,
                                              &vx_body, &vy_body);
                vx = float(vx_body); vy = float(vy_body); omega = float(om);
            }
        }

        // publish the body-frame twist; robot_controller does the IK + CAN
        geometry_msgs::msg::Twist tw;
        tw.linear.x  = vx;
        tw.linear.y  = vy;
        tw.angular.z = omega;
        cmd_pub_->publish(tw);
    }

    void publish_lookahead(double xL, double yL, double thL)
    {
        geometry_msgs::msg::PoseStamped p;
        p.header.stamp = now();
        p.header.frame_id = map_frame_;
        p.pose.position.x = xL;
        p.pose.position.y = yL;
        p.pose.orientation.z = std::sin(thL / 2.0);
        p.pose.orientation.w = std::cos(thL / 2.0);
        la_pub_->publish(p);
    }

    // ---- path tracker state ----
    Path_Tracker pt_;
    Init           init_;

    // ---- pose feedback ----
    std::mutex   fb_mutex_;
    double       pose_x_ {0.0}, pose_y_ {0.0}, pose_theta_ {0.0};
    bool         have_pose_ {false};
    rclcpp::Time last_pose_time_ {0, 0, RCL_ROS_TIME};

    // ---- path ----
    std::mutex  path_mutex_;
    Path        path_;
    std::string path_frame_;
    bool        have_path_ {false};
    int         search_from_ {0};
    bool        all_done_ {false};

    // ---- params ----
    std::string map_frame_, base_frame_;
    double      tf_poll_freq_ {100.0};
    bool        strict_frame_ {true};
    double      pose_timeout_ {0.25};

    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  la_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr           plan_sub_;
    rclcpp::TimerBase::SharedPtr pose_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathTrackerNode>());
    rclcpp::shutdown();
    return 0;
}