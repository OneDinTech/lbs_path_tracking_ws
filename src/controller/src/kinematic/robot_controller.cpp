// =====================================================================
// robot_controller.cpp -- swerve MOTOR DRIVER (the base controller).
//
//   subscribes : /cmd_vel        geometry_msgs/msg/Twist
//                                 linear.x = vx, linear.y = vy,
//                                 angular.z = omega  (robot/base frame)
//   publishes  : /joint_states   sensor_msgs/msg/JointState
//                                 3 steering + 3 speed joints (RViz)
//
// This is the ONLY node that drives the autonomous motors. It takes a
// body-frame twist, runs the swerve inverse kinematics, desaturates,
// applies the hardware drive-sign inversion, and pushes speed + steer
// commands onto CAN at a fixed rate.
//
//   path + pose -> path_tracker -> /cmd_vel -> robot_controller -> CAN
//
// Do NOT run at the same time as manual_drive: both write the same motor
// CAN ids. manual_drive listens on /cmd_vel_manual; this listens on
// /cmd_vel. Run one or the other depending on the launch mode.
//
// SAFETY: a watchdog zeroes the command if no /cmd_vel arrives within
// cmd_timeout seconds, so a crashed path-follower cannot leave the last
// command executing forever.
//
// Parameters load from manual_config.yaml (/robot_controller section),
// same physical values as /manual_drive. Keep invert_drive equal to
// manual_drive's, it is the same hardware fact.
// =====================================================================

#include "Swervedrive_Kinematic.hpp"
#include "can.hpp"
#include "driver.hpp"
#include "packer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <mutex>
#include <cmath>
#include <algorithm>

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

class RobotControllerNode : public rclcpp::Node
{
public:
    RobotControllerNode()
    : Node("robot_controller")
    {
        // ---- parameters ----
        can_iface_   = declare_parameter<std::string>("can_interface", "can0");
        can_bitrate_ = declare_parameter<int>("can_bitrate", 1000000);

        speed_ids_ = declare_parameter<std::vector<int64_t>>(
                         "speed_ids", std::vector<int64_t>{1, 3, 5});
        steer_ids_ = declare_parameter<std::vector<int64_t>>(
                         "steer_ids", std::vector<int64_t>{2, 4, 6});

        robot_l_ = declare_parameter<double>("robot_l", 0.24647);
        wheel_r_ = declare_parameter<double>("wheel_r", 0.045);

        wheel_speed_max_ = declare_parameter<double>("wheel_speed_max", 35.0);

        max_linear_  = declare_parameter<double>("max_linear_velocity", 1.5);
        max_angular_ = declare_parameter<double>("max_angular_velocity", 2.0);

        max_linear_accel_  = declare_parameter<double>("max_linear_accel", 2.0);
        max_angular_accel_ = declare_parameter<double>("max_angular_accel", 3.0);

        cmd_timeout_  = declare_parameter<double>("cmd_timeout", 0.3);
        control_rate_ = declare_parameter<double>("control_rate", 100.0);

        // Same hardware fact as manual_drive: positive command spins the
        // wheel backward on this robot, so the wire value is flipped.
        // Keep equal to manual_drive's invert_drive.
        invert_drive_ = declare_parameter<bool>("invert_drive", true);

        if (speed_ids_.size() != size_t(NUM_MODULES_3) ||
            steer_ids_.size() != size_t(NUM_MODULES_3)) {
            RCLCPP_FATAL(get_logger(),
                         "speed_ids and steer_ids must each have %d entries",
                         NUM_MODULES_3);
            throw std::runtime_error("bad CAN id parameter length");
        }

        // ---- CAN ----
        can_ = std::make_unique<CAN>(can_iface_, uint32_t(can_bitrate_), 1000);
        if (!can_->enable())
            RCLCPP_ERROR(get_logger(),
                         "Failed to open CAN '%s'. Wheels will not move.",
                         can_iface_.c_str());
        else
            RCLCPP_INFO(get_logger(), "CAN ready on %s", can_iface_.c_str());

        driver_ = std::make_unique<Driver>(*can_);
        swerve_init3(&sr_, float(robot_l_), float(wheel_r_));

        for (int i = 0; i < NUM_MODULES_3; i++) {
            wheel_speed_cmd_[i] = 0.0f;
            phi_cmd_[i]         = 0.0f;
            roll_angle_[i]      = 0.0;
        }
        last_cmd_time_ = now();

        // Subscribe to BOTH command topics. Auto mode drives /cmd_vel
        // (from path_tracker); manual mode drives /cmd_vel_manual (from
        // ps4). Only one ever publishes at a time (auto and manual are
        // exclusive launch modes), so a single handler serves both and
        // there is no arbitration to do.
        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&RobotControllerNode::on_cmd, this, std::placeholders::_1));
        cmd_manual_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_manual", 10,
            std::bind(&RobotControllerNode::on_cmd, this, std::placeholders::_1));

        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
                         "/joint_states", 10);

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / control_rate_),
            std::bind(&RobotControllerNode::control_step, this));

        RCLCPP_INFO(get_logger(),
                    "robot_controller ready: listening on /cmd_vel (auto) and "
                    "/cmd_vel_manual (manual). timeout %.2fs, max %.2f m/s, %.2f rad/s",
                    cmd_timeout_, max_linear_, max_angular_);
    }

    ~RobotControllerNode() override
    {
        for (int i = 0; i < NUM_MODULES_3; i++) {
            driver_->send_stop(uint16_t(speed_ids_[i]));
            driver_->send_stop(uint16_t(steer_ids_[i]));
        }
        RCLCPP_INFO(get_logger(), "robot_controller stopped, motors sent stop");
    }

private:
    // Shared command handler for both /cmd_vel and /cmd_vel_manual.
    void on_cmd(geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        req_vx_    = clamp(msg->linear.x,  -max_linear_,  max_linear_);
        req_vy_    = clamp(msg->linear.y,  -max_linear_,  max_linear_);
        req_omega_ = clamp(msg->angular.z, -max_angular_, max_angular_);
        last_cmd_time_ = now();
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

    void control_step()
    {
        const double dt = 1.0 / control_rate_;

        double tgt_vx, tgt_vy, tgt_omega;
        bool   stale;
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            stale = (now() - last_cmd_time_).seconds() > cmd_timeout_;
            tgt_vx    = stale ? 0.0 : req_vx_;
            tgt_vy    = stale ? 0.0 : req_vy_;
            tgt_omega = stale ? 0.0 : req_omega_;
        }

        if (stale && (std::fabs(cur_vx_) > 1e-6 ||
                      std::fabs(cur_vy_) > 1e-6 ||
                      std::fabs(cur_omega_) > 1e-6)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "no /cmd_vel for %.2fs, ramping to stop",
                                 cmd_timeout_);
        }

        cur_vx_    = slew(cur_vx_,    tgt_vx,    max_linear_accel_  * dt);
        cur_vy_    = slew(cur_vy_,    tgt_vy,    max_linear_accel_  * dt);
        cur_omega_ = slew(cur_omega_, tgt_omega, max_angular_accel_ * dt);

        // ---- inverse kinematics ----
        robot_control3(&sr_, wheel_speed_cmd_, phi_cmd_,
                       float(cur_vx_), float(cur_vy_), float(cur_omega_));

        // ---- joint desaturation (scale all three equally) ----
        {
            float max_ws = 0.0f;
            for (int i = 0; i < NUM_MODULES_3; i++)
                max_ws = std::max(max_ws, std::fabs(wheel_speed_cmd_[i]));
            if (max_ws > float(wheel_speed_max_)) {
                const float scale = float(wheel_speed_max_) / max_ws;
                for (int i = 0; i < NUM_MODULES_3; i++)
                    wheel_speed_cmd_[i] *= scale;
            }
        }

        // ---- TX to CAN ----
        for (int i = 0; i < NUM_MODULES_3; i++) {
            const float out = invert_drive_ ? -wheel_speed_cmd_[i]
                                            :  wheel_speed_cmd_[i];

            if (driver_->send_speed(uint16_t(speed_ids_[i]), out) == CANError::BUFFER_FULL)
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "CAN TX buffer full (speed) wheel %d", i);

            if (driver_->send_position(uint16_t(steer_ids_[i]), phi_cmd_[i]) == CANError::BUFFER_FULL)
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "CAN TX buffer full (steer) wheel %d", i);
        }

        // ---- /joint_states for RViz (steer angle + integrated spin) ----
        for (int i = 0; i < NUM_MODULES_3; i++)
            roll_angle_[i] += wheel_speed_cmd_[i] * dt;

        sensor_msgs::msg::JointState js;
        js.header.stamp = now();
        for (int i = 0; i < NUM_MODULES_3; i++) {
            js.name.push_back(STEER_JOINT[i]);
            js.position.push_back(phi_cmd_[i]);
        }
        for (int i = 0; i < NUM_MODULES_3; i++) {
            js.name.push_back(SPEED_JOINT[i]);
            js.position.push_back(roll_angle_[i]);
        }
        joint_pub_->publish(js);
    }

    // ---- config ----
    std::string          can_iface_;
    int                  can_bitrate_ {};
    std::vector<int64_t> speed_ids_, steer_ids_;
    double robot_l_ {}, wheel_r_ {}, wheel_speed_max_ {};
    double max_linear_ {}, max_angular_ {};
    double max_linear_accel_ {}, max_angular_accel_ {};
    double cmd_timeout_ {}, control_rate_ {};
    bool   invert_drive_ {};

    // ---- runtime ----
    std::unique_ptr<CAN>    can_;
    std::unique_ptr<Driver> driver_;
    Swerve_Robot3           sr_;

    std::mutex   cmd_mutex_;
    double       req_vx_ {0.0}, req_vy_ {0.0}, req_omega_ {0.0};
    rclcpp::Time last_cmd_time_;

    double cur_vx_ {0.0}, cur_vy_ {0.0}, cur_omega_ {0.0};

    float  wheel_speed_cmd_[NUM_MODULES_3];
    float  phi_cmd_[NUM_MODULES_3];
    double roll_angle_[NUM_MODULES_3];

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr   cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr   cmd_manual_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr   joint_pub_;
    rclcpp::TimerBase::SharedPtr                                 timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotControllerNode>());
    rclcpp::shutdown();
    return 0;
}