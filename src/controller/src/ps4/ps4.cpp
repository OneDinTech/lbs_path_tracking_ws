// =====================================================================
// ps4.cpp -- PS4 joystick reader. Converts /joy into /cmd_vel_manual.
//
// All tuning (speeds, step, axis indices, button indices) comes from
// parameters, loaded from config/manual_config.yaml. Change them there
// and relaunch, or live with:
//     ros2 param set /dynamic_teleop angular_speed 1.5
//
// This node does NOT drive motors. It only publishes a twist. On the
// real robot manual_drive consumes that twist and drives CAN; in the
// sim path_tracker_sim (manual_mode) consumes it.
// =====================================================================

#include <memory>
#include <algorithm>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

class DynamicTeleop : public rclcpp::Node
{
public:
    DynamicTeleop() : Node("dynamic_teleop")
    {
        // ---- parameters (from manual_config.yaml) ----
        linear_speed_  = declare_parameter<double>("linear_speed", 0.8);
        angular_speed_ = declare_parameter<double>("angular_speed", 2.0);
        speed_step_    = declare_parameter<double>("speed_step", 0.1);

        axis_vx_    = declare_parameter<int>("axis_vx", 1);
        axis_vy_    = declare_parameter<int>("axis_vy", 0);
        axis_omega_ = declare_parameter<int>("axis_omega", 3);

        btn_lin_up_   = declare_parameter<int>("btn_linear_up", 2);
        btn_lin_down_ = declare_parameter<int>("btn_linear_down", 0);
        btn_ang_up_   = declare_parameter<int>("btn_angular_up", 1);
        btn_ang_down_ = declare_parameter<int>("btn_angular_down", 3);

        param_cb_handle_ = add_on_set_parameters_callback(
            std::bind(&DynamicTeleop::on_param_change, this,
                      std::placeholders::_1));

        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&DynamicTeleop::joyCallback, this, std::placeholders::_1));
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_manual", 10);

        RCLCPP_INFO(get_logger(),
                    "Dynamic teleop started. Linear: %.2f  Angular: %.2f",
                    linear_speed_, angular_speed_);
    }

private:
    // Live re-load of the tuning params.
    rcl_interfaces::msg::SetParametersResult
    on_param_change(const std::vector<rclcpp::Parameter>& params)
    {
        for (const auto& p : params) {
            const std::string& n = p.get_name();
            if      (n == "linear_speed")     linear_speed_  = p.as_double();
            else if (n == "angular_speed")    angular_speed_ = p.as_double();
            else if (n == "speed_step")       speed_step_    = p.as_double();
            else if (n == "axis_vx")          axis_vx_    = int(p.as_int());
            else if (n == "axis_vy")          axis_vy_    = int(p.as_int());
            else if (n == "axis_omega")       axis_omega_ = int(p.as_int());
            else if (n == "btn_linear_up")    btn_lin_up_   = int(p.as_int());
            else if (n == "btn_linear_down")  btn_lin_down_ = int(p.as_int());
            else if (n == "btn_angular_up")   btn_ang_up_   = int(p.as_int());
            else if (n == "btn_angular_down") btn_ang_down_ = int(p.as_int());
        }
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = true;
        return r;
    }

    // Safe index read: returns 0.0 / 0 if the index is out of range for
    // this joystick, so a bad axis/button index cannot crash the node.
    static double axis_at(const sensor_msgs::msg::Joy::SharedPtr& m, int i)
    { return (i >= 0 && i < int(m->axes.size())) ? m->axes[i] : 0.0; }
    static int btn_at(const sensor_msgs::msg::Joy::SharedPtr& m, int i)
    { return (i >= 0 && i < int(m->buttons.size())) ? m->buttons[i] : 0; }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // ---- linear speed: up / down buttons (edge-triggered) ----
        const int b_lin_up   = btn_at(msg, btn_lin_up_);
        const int b_lin_down = btn_at(msg, btn_lin_down_);
        const int b_ang_up   = btn_at(msg, btn_ang_up_);
        const int b_ang_down = btn_at(msg, btn_ang_down_);

        if (b_lin_up == 1 && last_lin_up_ == 0) {
            linear_speed_ += speed_step_;
            RCLCPP_INFO(get_logger(), "Linear speed: %.2f", linear_speed_);
        }
        if (b_lin_down == 1 && last_lin_down_ == 0) {
            linear_speed_ = std::max(0.0, linear_speed_ - speed_step_);
            RCLCPP_INFO(get_logger(), "Linear speed: %.2f", linear_speed_);
        }
        if (b_ang_up == 1 && last_ang_up_ == 0) {
            angular_speed_ += speed_step_;
            RCLCPP_INFO(get_logger(), "Angular speed: %.2f", angular_speed_);
        }
        if (b_ang_down == 1 && last_ang_down_ == 0) {
            angular_speed_ = std::max(0.0, angular_speed_ - speed_step_);
            RCLCPP_INFO(get_logger(), "Angular speed: %.2f", angular_speed_);
        }
        last_lin_up_   = b_lin_up;
        last_lin_down_ = b_lin_down;
        last_ang_up_   = b_ang_up;
        last_ang_down_ = b_ang_down;

        geometry_msgs::msg::Twist twist;
        twist.linear.x  = axis_at(msg, axis_vx_)    * linear_speed_;
        twist.linear.y  = axis_at(msg, axis_vy_)    * linear_speed_;
        twist.angular.z = axis_at(msg, axis_omega_) * angular_speed_;
        cmd_pub_->publish(twist);
    }

    // ---- params ----
    double linear_speed_, angular_speed_, speed_step_;
    int    axis_vx_, axis_vy_, axis_omega_;
    int    btn_lin_up_, btn_lin_down_, btn_ang_up_, btn_ang_down_;

    // ---- edge-detect state ----
    int last_lin_up_ {0}, last_lin_down_ {0}, last_ang_up_ {0}, last_ang_down_ {0};

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicTeleop>());
    rclcpp::shutdown();
    return 0;
}