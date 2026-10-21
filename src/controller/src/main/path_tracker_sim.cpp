// =====================================================================
// path_tracker_sim.cpp -- KINEMATIC SIMULATION of the 3-wheel swerve robot
// following a path, using the SAME path tracker logic as
// path_tracker but with PERFECT odometry (no lidar, no CAN, no
// motors).
//
// HOW IT WORKS (self-contained closed loop):
//   1. subscribe /plan  (the SAME path path_planner_v2.py publishes)
//   2. run path tracker -> body velocity (vx, vy, omega), published on
//      /cmd_vel exactly like the real path_tracker
//   2b. motor driver stage, SAME as robot_controller: clamp the twist,
//      slew-limit it, then (after IK) desaturate the wheel speeds. The
//      limits come from robot_controller_config.yaml, the same file the
//      real robot_controller reads.
//   3. robot_control3  : body velocity -> wheel commands  (INVERSE kin)
//   4. get_robot_odom_pose3 : wheel commands -> integrate the simulated
//                             pose forward one step        (FORWARD kin)
//      This IS the "perfect odometry": the pose is exactly what the
//      commanded wheels produce, no sensor, no drift.
//   5. publish map -> base_footprint TF (the perfect pose) and
//      /joint_states (the 6 wheel joints) so RViz animates the robot
//      body moving along the path with the wheels steering + spinning.
//
// So you SEE, in RViz, exactly how the robot will drive the path and how
// every link updates, without any hardware.
//
//   subscribes : /plan          nav_msgs/msg/Path   (frame "map")
//   publishes  : TF map -> base_footprint
//                /joint_states  sensor_msgs/msg/JointState
//                /lookahead_point geometry_msgs/msg/PoseStamped (debug)
//
// Run:
//   ros2 run controller path_tracker_sim
//   ros2 launch robot_bringup description.launch.py   (model + RViz)
//   python3 path_planner_v2.py   (in src/simulation)
//
// The follow logic below is kept IN SYNC with path_tracker.cpp. The tuning
// is shared automatically: both nodes read the same /** block of
// path_tracker_config.yaml, so tuning the real robot tunes the sim too.
//
// What the sim still does NOT model: FAST-LIO pose noise, its ~10 Hz
// update rate and latency, wheel slip, and motor response lag.
// =====================================================================

#include "Path_Tracker.hpp"
#include "Swervedrive_Kinematic.hpp"
#include "Path_Generation.hpp"   // Path struct type only

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

static constexpr double CONTROL_FREQ = 100.0;
static constexpr double Ts           = 1.0 / CONTROL_FREQ;

// Same URDF joint names as path_tracker, so the sim animates the
// same links.
static const char* const STEER_JOINT[NUM_MODULES_3] = {
    "base_steering_wheel1_joint",
    "base_steering_wheel2_joint",
    "base_steering_wheel3_joint",
};
static const char* const SPEED_JOINT[NUM_MODULES_3] = {
    "steer_speed_wheel1_joint",
    "steer_speed_wheel2_joint",
    "steer_speed_wheel3_joint",
};

static inline double yaw_from_quat(double x, double y, double z, double w)
{
    return std::atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z));
}
static inline double yaw_of(const geometry_msgs::msg::Quaternion& q)
{
    return yaw_from_quat(q.x, q.y, q.z, q.w);
}

class PathTrackerSimNode : public rclcpp::Node
{
public:
    PathTrackerSimNode() : Node("path_tracker_sim")
    {
        map_frame_  = declare_parameter<std::string>("map_frame", "map");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

        // manual_mode: if true, the sim robot is driven by /cmd_vel_manual
        // (joystick) instead of following /plan with path tracker. Same
        // integrate-the-pose loop, different velocity source.
        manual_mode_ = declare_parameter<bool>("manual_mode", false);

        // ---- path tracker parameters (declared + loaded from YAML,
        //      the SAME file path_tracker reads) ----
        pt_ = {};
        pt_.CONTROLLER_FREQUENCY = CONTROL_FREQ;
        declare_pt_params();
        load_pt_params();
        param_init(&init_);

        swerve_init3(&sr_, float(wheel_base_radius_), float(wheel_radius_));

        param_cb_handle_ = add_on_set_parameters_callback(
            std::bind(&PathTrackerSimNode::on_param_change, this,
                      std::placeholders::_1));

        for (int i = 0; i < NUM_MODULES_3; i++) {
            wheel_speed_[i] = 0.0f;
            phi_[i]         = 0.0f;
            roll_angle_[i]  = 0.0;
        }

        // simulated pose starts at the origin (map == start)
        x_ = 0.0f; y_ = 0.0f; theta_ = 0.0f;

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        la_pub_    = create_publisher<geometry_msgs::msg::PoseStamped>("/lookahead_point", 10);
        traveled_pub_ = create_publisher<nav_msgs::msg::Path>("/traveled_path", 10);
        speed_pub_    = create_publisher<std_msgs::msg::Float32>("/robot_speed", 10);
        cmd_pub_      = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        traveled_.header.frame_id = map_frame_;

        plan_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/plan", rclcpp::QoS(10),
            std::bind(&PathTrackerSimNode::plan_cb, this, std::placeholders::_1));

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_manual", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                manual_vx_    = float(msg->linear.x);
                manual_vy_    = float(msg->linear.y);
                manual_omega_ = float(msg->angular.z);
            });

        // publish the initial pose so RViz shows the robot at the start
        // even before a path arrives.
        publish_pose_and_joints();

        timer_ = create_wall_timer(
            std::chrono::duration<double>(Ts),
            std::bind(&PathTrackerSimNode::step, this));

        RCLCPP_INFO(get_logger(),
                    "path_tracker_sim started (perfect odometry). Waiting for /plan; "
                    "publishing %s -> %s and /joint_states.",
                    map_frame_.c_str(), base_frame_.c_str());
    }

private:

    // ---- parameter plumbing (mirrors path_tracker) -------------
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

        // ---- motor driver stage (same names + defaults as robot_controller,
        //      loaded from robot_controller_config.yaml by the sim launch) ----
        // The real robot sends /cmd_vel through robot_controller, which
        // clamps, slew-limits and desaturates before the wheels move. The
        // sim applies the SAME stage so it drives like the hardware.
        declare_parameter<double>("robot_l", 0.24647);
        declare_parameter<double>("wheel_r", 0.045);
        declare_parameter<double>("wheel_speed_max", 35.0);
        declare_parameter<double>("max_linear_velocity", 1.5);
        declare_parameter<double>("max_angular_velocity", 2.0);
        declare_parameter<double>("max_linear_accel", 2.0);
        declare_parameter<double>("max_angular_accel", 3.0);

        // false = like the real robot: start where the robot is.
        // true  = jump the sim robot to the first path pose on a new /plan.
        declare_parameter<bool>("teleport_to_start", false);
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
        wheel_base_radius_ = get_parameter("robot_l").as_double();
        wheel_radius_      = get_parameter("wheel_r").as_double();
        wheel_speed_max_   = get_parameter("wheel_speed_max").as_double();
        max_linear_        = get_parameter("max_linear_velocity").as_double();
        max_angular_       = get_parameter("max_angular_velocity").as_double();
        max_linear_accel_  = get_parameter("max_linear_accel").as_double();
        max_angular_accel_ = get_parameter("max_angular_accel").as_double();
        teleport_to_start_ = get_parameter("teleport_to_start").as_bool();
    }

    rcl_interfaces::msg::SetParametersResult
    on_param_change(const std::vector<rclcpp::Parameter>& /*params*/)
    {
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            load_pt_params();
            swerve_init3(&sr_, float(wheel_base_radius_), float(wheel_radius_));
        }
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = true;
        RCLCPP_INFO(get_logger(), "sim parameters updated live");
        return r;
    }

    // Accept the SAME path the planner sends. Same duplicate + heading
    // check idea as path_tracker, simplified.
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
        have_path_   = true;
        search_from_ = 0;
        all_done_    = false;
        param_init(&init_);

        // The real robot starts wherever it is when the path arrives, so by
        // default the sim does too. teleport_to_start: true places the sim
        // robot on the first path pose instead (handy for repeated runs).
        if (teleport_to_start_) {
            x_ = float(path_.x.front());
            y_ = float(path_.y.front());
            theta_ = float(path_.theta.front());
            cur_vx_ = cur_vy_ = cur_omega_ = 0.0;
        }

        RCLCPP_INFO(get_logger(),
                    "new /plan: %zu poses, first (%.2f, %.2f) -> last (%.2f, %.2f)",
                    n, path_.x.front(), path_.y.front(),
                    path_.x.back(), path_.y.back());
    }

    // 100 Hz: (manual twist OR path tracker) -> IK -> integrate perfect
    // pose -> publish.
    void step()
    {
        float vx = 0.0f, vy = 0.0f, omega = 0.0f;

        if (manual_mode_) {
            // Joystick drive: velocity comes straight from /cmd_vel_manual.
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            vx = manual_vx_; vy = manual_vy_; omega = manual_omega_;
        }
        else
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            if (have_path_ && !all_done_) {
                const int n = int(path_.x.size());

                const double L = get_lookahead_distance(&pt_, &init_);
                double x_L, y_L, theta_L;
                int closest_idx;
                get_lookahead_point(path_.x.data(), path_.y.data(), path_.theta.data(),
                                    n, search_from_, x_, y_, L,
                                    &x_L, &y_L, &theta_L, &closest_idx);
                if (closest_idx > search_from_) search_from_ = closest_idx;
                publish_lookahead(x_L, y_L, theta_L);

                const double d_g   = get_goal_distance(path_.x.data(), path_.y.data(),
                                                       n, x_, y_);
                const double l_rem = get_remaining_path_length(path_.x.data(), path_.y.data(),
                                                               n, closest_idx);
                const double d     = get_lookahead_distance_to_point(x_L, y_L, x_, y_);

                const bool path_consumed = search_from_ >= n - 2;
                // closed-loop guard: goal == start, so gate on progress
                const bool near_end = search_from_ >= (n * 3) / 4;
                const double d_g_eff = near_end ? d_g : 1e3;
                const bool position_done = d_g_eff < pt_.DISTANCE_GOAL_TOLERANCE;
                const bool heading_done  =
                    std::fabs(normalize_angle(path_.theta.back() - theta_)) < pt_.YAW_GOAL_TOLERANCE;

                if (path_consumed && position_done && heading_done) {
                    all_done_ = true;
                    RCLCPP_INFO(get_logger(), "sim: goal reached");
                }
                else if (path_consumed && position_done && !heading_done) {
                    omega = float(get_angular_velocity(&pt_, &init_,
                                                       path_.theta.back(), theta_,
                                                       pt_.MIN_LOOKAHEAD_DISTANCE, pt_.V, 0.0));
                }
                else {
                    const double v_cap = get_corner_speed_limit(&pt_, &init_,
                                            path_.x.data(), path_.y.data(),
                                            n, closest_idx);
                    const double v  = get_linear_velocity(&pt_, &init_, d_g_eff, l_rem, v_cap);
                    // heading reference = closest-point heading (track each
                    // pose's heading precisely); look-ahead drives translation
                    const int hidx = (closest_idx < n) ? closest_idx : n - 1;
                    const double theta_ref = path_.theta[hidx];
                    const double om_ff = get_heading_feedforward(
                                            path_.theta.data(), hidx,
                                            n, path_.x.data(), path_.y.data(), v);
                    const double om = get_angular_velocity(&pt_, &init_, theta_ref, theta_, d, init_.V_PREV, om_ff);
                    get_world_velocity(&init_, v, d, x_L, y_L, x_, y_);
                    double vx_body, vy_body;
                    world_to_robot_frame_velocity(init_.VX_WORLD, init_.VY_WORLD, theta_,
                                                  &vx_body, &vy_body);
                    vx = float(vx_body); vy = float(vy_body); omega = float(om);
                }
            }
        }

        // Publish the tracker's twist on /cmd_vel, exactly what the real
        // path_tracker publishes (BEFORE the motor driver's limits), so the
        // data_recorder sees the same signal in sim and on hardware.
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = vx;
        cmd.linear.y  = vy;
        cmd.angular.z = omega;
        cmd_pub_->publish(cmd);

        // ---- motor driver stage: same steps as robot_controller ----
        // 1. clamp the incoming twist
        const double tgt_vx    = clamp(vx,    -max_linear_,  max_linear_);
        const double tgt_vy    = clamp(vy,    -max_linear_,  max_linear_);
        const double tgt_omega = clamp(omega, -max_angular_, max_angular_);
        // 2. slew (acceleration) limit
        cur_vx_    = slew(cur_vx_,    tgt_vx,    max_linear_accel_  * Ts);
        cur_vy_    = slew(cur_vy_,    tgt_vy,    max_linear_accel_  * Ts);
        cur_omega_ = slew(cur_omega_, tgt_omega, max_angular_accel_ * Ts);

        // 3. IK: body velocity -> wheel commands
        robot_control3(&sr_, wheel_speed_, phi_,
                       float(cur_vx_), float(cur_vy_), float(cur_omega_));

        // 4. wheel desaturation: scale all three equally if any is over
        {
            float max_ws = 0.0f;
            for (int i = 0; i < NUM_MODULES_3; i++)
                max_ws = std::max(max_ws, std::fabs(wheel_speed_[i]));
            if (max_ws > float(wheel_speed_max_)) {
                const float scale = float(wheel_speed_max_) / max_ws;
                for (int i = 0; i < NUM_MODULES_3; i++)
                    wheel_speed_[i] *= scale;
            }
        }

        // FORWARD kin: integrate the perfect pose from the (limited) wheel
        // commands, so the pose reflects what the real wheels would do.
        get_robot_odom_pose3(&sr_, wheel_speed_, phi_, float(Ts),
                             &x_, &y_, &theta_);

        publish_pose_and_joints();
        // /robot_speed = the speed the robot actually drives (after limits)
        publish_speed_and_trail(float(cur_vx_), float(cur_vy_));
    }

    static double clamp(double v, double lo, double hi)
    { return std::max(lo, std::min(hi, v)); }

    static double slew(double current, double target, double max_delta)
    {
        const double d = target - current;
        if (d >  max_delta) return current + max_delta;
        if (d < -max_delta) return current - max_delta;
        return target;
    }

    // Publish the commanded linear speed (magnitude of the body velocity)
    // on /robot_speed, and accumulate the driven pose into /traveled_path
    // so the actual trajectory can be shown in RViz (add a Path display).
    void publish_speed_and_trail(float vx, float vy)
    {
        std_msgs::msg::Float32 s;
        s.data = std::sqrt(vx * vx + vy * vy);
        speed_pub_->publish(s);

        // append to the trail only if the robot moved enough, to keep the
        // path from filling with near-duplicate poses
        const double step = (have_trail_)
            ? std::hypot(x_ - trail_last_x_, y_ - trail_last_y_) : 1e9;
        if (!have_trail_ || step >= 0.02) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.stamp = now();
            ps.header.frame_id = map_frame_;
            ps.pose.position.x = x_;
            ps.pose.position.y = y_;
            ps.pose.orientation.z = std::sin(theta_ / 2.0);
            ps.pose.orientation.w = std::cos(theta_ / 2.0);
            traveled_.poses.push_back(ps);
            trail_last_x_ = x_;
            trail_last_y_ = y_;
            have_trail_ = true;
        }
        traveled_.header.stamp = now();
        traveled_pub_->publish(traveled_);
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

    // Publish the perfect pose as map -> base_footprint TF, and the six
    // wheel joints so the model animates.
    void publish_pose_and_joints()
    {
        const rclcpp::Time t = now();

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = t;
        tf.header.frame_id = map_frame_;
        tf.child_frame_id  = base_frame_;
        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.z = std::sin(theta_ / 2.0);
        tf.transform.rotation.w = std::cos(theta_ / 2.0);
        tf_broadcaster_->sendTransform(tf);

        // integrate wheel roll for the spin animation
        for (int i = 0; i < NUM_MODULES_3; i++)
            roll_angle_[i] += wheel_speed_[i] * Ts;

        sensor_msgs::msg::JointState js;
        js.header.stamp = t;
        for (int i = 0; i < NUM_MODULES_3; i++) {
            js.name.push_back(STEER_JOINT[i]);
            js.position.push_back(phi_[i]);
        }
        for (int i = 0; i < NUM_MODULES_3; i++) {
            js.name.push_back(SPEED_JOINT[i]);
            js.position.push_back(roll_angle_[i]);
        }
        joint_pub_->publish(js);
    }

    // ---- path tracker + kinematics ----
    Path_Tracker pt_;
    Init           init_;
    Swerve_Robot3  sr_;

    // ---- simulated (perfect) pose ----
    // float, because get_robot_odom_pose3 takes float* for the pose.
    float  x_ {0.0f}, y_ {0.0f}, theta_ {0.0f};
    float  wheel_speed_[NUM_MODULES_3];
    float  phi_[NUM_MODULES_3];
    double roll_angle_[NUM_MODULES_3];

    // ---- path ----
    std::mutex  path_mutex_;
    Path        path_;
    bool        have_path_ {false};
    int         search_from_ {0};
    bool        all_done_ {false};

    std::string map_frame_, base_frame_;
    double wheel_base_radius_ {0.24647};
    double wheel_radius_ {0.045};
    bool   teleport_to_start_ {false};

    // ---- motor driver stage (mirrors robot_controller) ----
    double wheel_speed_max_ {35.0};
    double max_linear_ {1.5}, max_angular_ {2.0};
    double max_linear_accel_ {2.0}, max_angular_accel_ {3.0};
    double cur_vx_ {0.0}, cur_vy_ {0.0}, cur_omega_ {0.0};
    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    // ---- manual drive ----
    bool         manual_mode_ {false};
    std::mutex   cmd_mutex_;
    float        manual_vx_ {0.0f}, manual_vy_ {0.0f}, manual_omega_ {0.0f};

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr   joint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr la_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr           traveled_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr        speed_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr     cmd_pub_;
    nav_msgs::msg::Path traveled_;
    double trail_last_x_ {0.0};
    double trail_last_y_ {0.0};
    bool   have_trail_ {false};
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr         plan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr   cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathTrackerSimNode>());
    rclcpp::shutdown();
    return 0;
}