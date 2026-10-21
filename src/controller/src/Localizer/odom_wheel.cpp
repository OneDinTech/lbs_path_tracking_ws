// =====================================================================
// odom_wheel.cpp -- dead wheel + gyro odometry. Owns odom -> base_link.
//
//   subscribes : /imu/data          sensor_msgs/msg/Imu  (yaw rate only)
//   publishes  : /odometry/filtered nav_msgs/msg/Odometry (pose + twist)
//                /dead_wheel/odom   nav_msgs/msg/Odometry (twist only)
//                TF: odom -> base_link
//
// ---------------------------------------------------------------------
// WHY THIS REPLACED ekf_local
//
// The local layer had no redundancy: dead wheels measure vx and vy,
// the gyro measures yaw rate, and nothing measures the same state
// twice. With nothing to weigh against anything, a Kalman filter
// collapses to plain dead reckoning:
//
//     x += (vx*cos0 - vy*sin0) * dt
//     y += (vx*sin0 + vy*cos0) * dt
//     0 += w * dt
//
// which is what this node does directly. Removing robot_localization
// from the local path also removes a class of failures that cost real
// time: unloaded YAML producing a silently unconfigured filter, and
// NaN from a zero quaternion during gravity removal.
//
// ekf_global STAYS. That layer does have redundancy, dead reckoning
// versus the lidar's absolute pose, two independent measurements of
// the same state. That is the case a Kalman filter actually exists
// for. /dead_wheel/odom is still published for it to consume.
// ---------------------------------------------------------------------
//
// SENSOR LAYOUT
//   EncoderFeedback frame: position @ byte 0, speed @ byte 4,
//   float32 little endian. Dead wheels use FIXED absolute CAN ids,
//   with no +100 offset, because they are passive and have no
//   corresponding TX command to offset from.
//
//     deadwheel_x_id -> body frame X translation (forward +)
//     deadwheel_y_id -> body frame Y translation (left +)
//
//   v = speed * radius * calibration
//
// CONVENTION: REP-103. x forward positive, y left positive.
//   If a wheel reads backwards on the bench, flip it with invert_x /
//   invert_y rather than negating the calibration, so calibration
//   stays a pure scale.
//
// YAW: integrated from angular_velocity.z. The IMU's orientation
// quaternion is deliberately NOT used. That field is zero-filled
// before the serial thread receives its first packet, and a zero
// quaternion cannot be normalized.
//
// RX runs on its own thread, decoupled from the publish timer: a
// blocking CAN read on a ROS timer stalls everything behind it.
// =====================================================================

#include "can.hpp"
#include "driver.hpp"
#include "packer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>
#include <cmath>
#include <memory>

class OdomWheelNode : public rclcpp::Node
{
public:
    OdomWheelNode()
    : Node("odom_wheel"),
      rx_running_(true)
    {
        // ---- parameters -------------------------------------------------
        // ROS parameters rather than compile time constants so the
        // calibration can be retuned without a rebuild. The radius and
        // calibration defaults came from another robot's
        // odometry_node.py and are almost certainly wrong for this one.
        // Measure before trusting the output.
        can_iface_   = declare_parameter<std::string>("can_interface", "can0");
        can_bitrate_ = declare_parameter<int>("can_bitrate", 1000000);

        deadwheel_x_id_ = declare_parameter<int>("deadwheel_x_id", 162);
        deadwheel_y_id_ = declare_parameter<int>("deadwheel_y_id", 161);

        wheel_radius_ = declare_parameter<double>("wheel_radius", 0.028);
        cal_x_        = declare_parameter<double>("calibration_x", 1.0696132);
        cal_y_        = declare_parameter<double>("calibration_y", 1.0661124);

        invert_x_   = declare_parameter<bool>("invert_x", false);
        invert_y_   = declare_parameter<bool>("invert_y", false);
        invert_yaw_ = declare_parameter<bool>("invert_yaw", false);

        publish_rate_ = declare_parameter<double>("publish_rate", 100.0);

        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        imu_topic_  = declare_parameter<std::string>("imu_topic", "/imu/data");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/filtered");
        twist_topic_ = declare_parameter<std::string>("twist_topic", "/dead_wheel/odom");

        // This node owns odom -> base_link. Nothing else may publish
        // that edge. TF does not error on two publishers, it
        // interleaves them, and the pose oscillates between two
        // estimates. It looks like a controller bug and you hunt it in
        // the wrong place.
        publish_tf_ = declare_parameter<bool>("publish_tf", true);

        // If no CAN frame arrives from a wheel within this window its
        // velocity is treated as unknown rather than held. Integrating
        // a stale velocity is how odometry silently walks away from
        // reality when a sensor drops off the bus.
        stale_timeout_ = declare_parameter<double>("stale_timeout", 0.1);

        // Covariance published on the pose. Dead reckoning has no
        // absolute reference, so this grows without bound in reality.
        // A fixed modest value is enough for ekf_global to weigh
        // against the lidar. Raise it to let lidar corrections
        // dominate more.
        pose_variance_ = declare_parameter<double>("pose_variance", 0.05);
        yaw_variance_  = declare_parameter<double>("yaw_variance", 0.05);
        vel_variance_  = declare_parameter<double>("velocity_variance", 0.001);

        // ---- CAN --------------------------------------------------------
        can_ = std::make_unique<CAN>(can_iface_, uint32_t(can_bitrate_), 1000);
        if (!can_->enable()) {
            RCLCPP_ERROR(get_logger(),
                         "Failed to open CAN interface '%s'. No odometry will be published.",
                         can_iface_.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "CAN ready on %s", can_iface_.c_str());
        }

        odom_pub_  = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
        twist_pub_ = create_publisher<nav_msgs::msg::Odometry>(twist_topic_, 10);

        if (publish_tf_)
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // ---- IMU: yaw rate only -----------------------------------------
        // Own callback group so it is not queued behind the publish
        // timer. Bursty yaw updates make the heading estimate lag,
        // which downstream shows up as the controller hunting.
        imu_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions imu_opts;
        imu_opts.callback_group = imu_cb_group_;

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::QoS(50),
            [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                const rclcpp::Time stamp(msg->header.stamp);
                double wz = msg->angular_velocity.z;
                if (invert_yaw_) wz = -wz;
                if (!std::isfinite(wz)) return;

                std::lock_guard<std::mutex> lock(state_mutex_);

                if (!yaw_init_) {
                    // First message only establishes the timing
                    // baseline, so yaw starts at exactly 0.
                    last_imu_time_ = stamp;
                    yaw_init_      = true;
                    theta_         = 0.0;
                    RCLCPP_INFO(get_logger(), "yaw integration started");
                    return;
                }

                const double dt = (stamp - last_imu_time_).seconds();
                last_imu_time_ = stamp;
                omega_ = wz;

                // Guard a bad dt (clock jump, dropped messages) from
                // corrupting the integral.
                if (dt > 0.0 && dt < 1.0)
                    theta_ += wz * dt;
            },
            imu_opts);

        // Zero the pose without restarting the node. Useful when
        // re-running a test from a marked spot on the floor.
        reset_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/reset",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    x_ = 0.0; y_ = 0.0; theta_ = 0.0;
                    yaw_init_ = false;
                }
                res->success = true;
                res->message = "pose zeroed";
                RCLCPP_WARN(get_logger(), "pose reset to origin");
            });

        rx_thread_ = std::thread(&OdomWheelNode::rx_loop, this);

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate_),
            std::bind(&OdomWheelNode::publish_step, this));

        RCLCPP_INFO(get_logger(),
                    "odom_wheel started: x_id=%d y_id=%d r=%.4f cal=(%.6f, %.6f), "
                    "owns %s -> %s",
                    deadwheel_x_id_, deadwheel_y_id_, wheel_radius_,
                    cal_x_, cal_y_, odom_frame_.c_str(), base_frame_.c_str());
    }

    ~OdomWheelNode() override
    {
        rx_running_.store(false);
        if (rx_thread_.joinable())
            rx_thread_.join();
        RCLCPP_INFO(get_logger(), "odom_wheel stopped");
    }

private:

    // =================================================================
    // RX THREAD
    // One receive, then dispatch by id. Reading once per sensor and
    // discarding non matching frames throws away roughly half the data
    // when two sensors share a bus.
    // =================================================================
    void rx_loop()
    {
        while (rx_running_.load()) {
            uint16_t id {}; uint8_t data[8] {}; uint8_t len {};

            if (can_->receive_frame(id, data, len, 50) != CANError::OK)
                continue;
            if (len < 8)
                continue;

            const float spd = Packer::unpack_f32(data + 4);
            if (!std::isfinite(spd))
                continue;

            const rclcpp::Time stamp = now();

            if (int(id) == deadwheel_x_id_) {
                double v = double(spd) * wheel_radius_ * cal_x_;
                if (invert_x_) v = -v;
                std::lock_guard<std::mutex> lock(state_mutex_);
                vx_ = v; last_x_time_ = stamp; got_x_ = true;
            }
            else if (int(id) == deadwheel_y_id_) {
                double v = double(spd) * wheel_radius_ * cal_y_;
                if (invert_y_) v = -v;
                std::lock_guard<std::mutex> lock(state_mutex_);
                vy_ = v; last_y_time_ = stamp; got_y_ = true;
            }
            // any other id on the bus belongs to someone else
        }
    }

    // =================================================================
    // INTEGRATE AND PUBLISH
    // =================================================================
    void publish_step()
    {
        const rclcpp::Time stamp = now();
        const double dt = 1.0 / publish_rate_;

        double vx = 0.0, vy = 0.0, th = 0.0, om = 0.0, px = 0.0, py = 0.0;
        bool   x_fresh = false, y_fresh = false, ready = false;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            x_fresh = got_x_ && (stamp - last_x_time_).seconds() < stale_timeout_;
            y_fresh = got_y_ && (stamp - last_y_time_).seconds() < stale_timeout_;

            // A stale axis contributes zero rather than its last value.
            vx = x_fresh ? vx_ : 0.0;
            vy = y_fresh ? vy_ : 0.0;
            th = theta_;
            om = omega_;

            // Integrate body frame velocity into the odom frame. This
            // is the whole of what ekf_local was doing.
            const double c = std::cos(th), s = std::sin(th);
            x_ += dt * (c * vx - s * vy);
            y_ += dt * (s * vx + c * vy);

            px = x_; py = y_;

            // Do not publish before the gyro has a timing baseline:
            // heading would be a fixed zero and the integration would
            // lay down a straight line in the wrong direction.
            ready = yaw_init_ && (got_x_ || got_y_);
        }

        if (!ready) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "waiting for first IMU message on %s and CAN data "
                                 "(x_id=%d, y_id=%d)",
                                 imu_topic_.c_str(), deadwheel_x_id_, deadwheel_y_id_);
            return;
        }

        const double qz = std::sin(th * 0.5);
        const double qw = std::cos(th * 0.5);

        // ---- full odometry, for the controller and ekf_global --------
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = stamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id  = base_frame_;

        odom.pose.pose.position.x    = px;
        odom.pose.pose.position.y    = py;
        odom.pose.pose.orientation.z = qz;
        odom.pose.pose.orientation.w = qw;

        for (int i = 0; i < 36; i++) odom.pose.covariance[i] = 0.0;
        odom.pose.covariance[0]  = pose_variance_;
        odom.pose.covariance[7]  = pose_variance_;
        odom.pose.covariance[14] = 1e6;
        odom.pose.covariance[21] = 1e6;
        odom.pose.covariance[28] = 1e6;
        odom.pose.covariance[35] = yaw_variance_;

        odom.twist.twist.linear.x  = vx;    // body frame
        odom.twist.twist.linear.y  = vy;
        odom.twist.twist.angular.z = om;

        for (int i = 0; i < 36; i++) odom.twist.covariance[i] = 0.0;
        odom.twist.covariance[0]  = x_fresh ? vel_variance_ : 1e6;
        odom.twist.covariance[7]  = y_fresh ? vel_variance_ : 1e6;
        odom.twist.covariance[14] = 1e6;
        odom.twist.covariance[21] = 1e6;
        odom.twist.covariance[28] = 1e6;
        odom.twist.covariance[35] = vel_variance_;

        odom_pub_->publish(odom);

        // ---- velocity only, for ekf_global --------------------------
        // ekf_global fuses these velocities alongside the lidar pose.
        // The pose fields carry a huge covariance so that filter cannot
        // accidentally fuse this node's dead reckoned position as if it
        // were an independent measurement: it is the same information
        // the velocities already carry.
        nav_msgs::msg::Odometry tw = odom;
        tw.pose.pose.position.x    = 0.0;
        tw.pose.pose.position.y    = 0.0;
        tw.pose.pose.orientation.z = 0.0;
        tw.pose.pose.orientation.w = 1.0;
        for (int i = 0; i < 36; i++) tw.pose.covariance[i] = 0.0;
        tw.pose.covariance[0]  = 1e6;
        tw.pose.covariance[7]  = 1e6;
        tw.pose.covariance[14] = 1e6;
        tw.pose.covariance[21] = 1e6;
        tw.pose.covariance[28] = 1e6;
        tw.pose.covariance[35] = 1e6;
        twist_pub_->publish(tw);

        // ---- TF ------------------------------------------------------
        if (publish_tf_ && tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp            = stamp;
            t.header.frame_id         = odom_frame_;
            t.child_frame_id          = base_frame_;
            t.transform.translation.x = px;
            t.transform.translation.y = py;
            t.transform.translation.z = 0.0;
            t.transform.rotation.z    = qz;
            t.transform.rotation.w    = qw;
            tf_broadcaster_->sendTransform(t);
        }
    }

    // ---- config ----
    std::string can_iface_, odom_frame_, base_frame_;
    std::string imu_topic_, odom_topic_, twist_topic_;
    int         can_bitrate_ {};
    int         deadwheel_x_id_ {}, deadwheel_y_id_ {};
    double      wheel_radius_ {}, cal_x_ {}, cal_y_ {};
    bool        invert_x_ {}, invert_y_ {}, invert_yaw_ {};
    bool        publish_tf_ {};
    double      publish_rate_ {}, stale_timeout_ {};
    double      pose_variance_ {}, yaw_variance_ {}, vel_variance_ {};

    // ---- runtime ----
    std::unique_ptr<CAN> can_;
    std::thread          rx_thread_;
    std::atomic<bool>    rx_running_;

    std::mutex   state_mutex_;
    double       vx_ {0.0}, vy_ {0.0};
    double       x_ {0.0}, y_ {0.0}, theta_ {0.0}, omega_ {0.0};
    bool         got_x_ {false}, got_y_ {false}, yaw_init_ {false};
    rclcpp::Time last_x_time_ {0, 0, RCL_ROS_TIME};
    rclcpp::Time last_y_time_ {0, 0, RCL_ROS_TIME};
    rclcpp::Time last_imu_time_ {0, 0, RCL_ROS_TIME};

    rclcpp::CallbackGroup::SharedPtr                      imu_cb_group_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr  odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr  twist_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr     reset_srv_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>         tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr                           timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomWheelNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}