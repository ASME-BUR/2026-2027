// bench_sim_node: a kinematic stand-in for the vehicle, the EKF and the camera,
// for the DRY BENCH ONLY. Never launch it on the vehicle.
//
//   /gait_command (from the real bur_controller) --> unicycle model
//       --> /odometry/filtered (with twist.angular.z: the controller's yaw-rate
//           loop reads it), TF odom -> base_link
//       --> /vision (bur_msgs/CVDetections) for one configured target,
//           in the pixel-x layout detection.hpp assumes (D-A4)
//
// Every number here is invented. The sim proves the plumbing and the tree
// logic, not performance. Speed and turn rate come from M-3, not from here.

#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "bur_msgs/msg/cv_detections.hpp"
#include "bur_msgs/msg/gait_command.hpp"

#include "bur_autonomy/detection.hpp"
#include "bur_autonomy/sim/unicycle.hpp"

namespace bur_autonomy
{

class BenchSimNode : public rclcpp::Node
{
public:
    BenchSimNode() : rclcpp::Node("bench_sim")
    {
        // Inverse of the controller's open-loop surge_to_freq (controller.yaml),
        // so commanded speed and simulated speed agree.
        surge_to_freq_ = declare_parameter("surge_to_freq", 2.0);
        // yaw rate = turn_gain * turn_bias. With the controller's P-only
        // yaw_rate.kp 0.4 the loop gain is 0.4 * 5 = 2, so the sim settles at
        // 2/3 of the commanded yaw rate (~0.27 rad/s at max_yaw_rate 0.4; a
        // pi turn in ~12 s, inside TurnToHeading's 20 s timeout). The 0.3 rad
        // turn_bias_max cap allows 1.5 rad/s, so it never binds.
        turn_gain_ = declare_parameter("turn_gain", 5.0);
        cmd_timeout_s_ = declare_parameter("cmd_timeout_s", 0.5);
        target_x_ = declare_parameter("target_x", 6.0);
        target_y_ = declare_parameter("target_y", 2.0);
        target_label_ = declare_parameter("target_label", 0);
        max_range_m_ = declare_parameter("max_range_m", 15.0);
        cam_.image_width_px = declare_parameter("camera_image_width_px", 1280.0);
        cam_.hfov_rad = declare_parameter("camera_hfov_rad", 1.92);
        const double vision_hz = declare_parameter("vision_rate_hz", 2.0);   // ~yolo.py

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);
        vision_pub_ = create_publisher<bur_msgs::msg::CVDetections>("/vision", 10);
        tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        gait_sub_ = create_subscription<bur_msgs::msg::GaitCommand>(
            "/gait_command", 10, [this](bur_msgs::msg::GaitCommand::ConstSharedPtr m) {
                gait_ = *m;
                last_gait_ = now();
                have_gait_ = true;
            });

        last_step_ = now();
        step_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { step(); });
        vision_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / vision_hz)), [this]() { vision(); });
    }

private:
    void step()
    {
        const rclcpp::Time t = now();
        const double dt = (t - last_step_).seconds();
        last_step_ = t;

        double surge = 0.0, yaw_rate = 0.0;
        const bool fresh = have_gait_ && (t - last_gait_).seconds() < cmd_timeout_s_;
        const bool driving = fresh && (gait_.mode == bur_msgs::msg::GaitCommand::MODE_CRUISE ||
                                       gait_.mode == bur_msgs::msg::GaitCommand::MODE_TURN ||
                                       gait_.mode == bur_msgs::msg::GaitCommand::MODE_HOLD);
        if (driving) {
            surge = (surge_to_freq_ > 0.0) ? gait_.frequency_hz / surge_to_freq_ : 0.0;
            // HOLD is frequency 0: a static bend, no thrust, so no turning either.
            yaw_rate = (gait_.frequency_hz > 0.0f) ? turn_gain_ * gait_.turn_bias : 0.0;
        }
        body_.step(surge, yaw_rate, dt);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, body_.yaw);

        nav_msgs::msg::Odometry o;
        o.header.stamp = t;
        o.header.frame_id = "odom";
        o.child_frame_id = "base_link";
        o.pose.pose.position.x = body_.x;
        o.pose.pose.position.y = body_.y;
        o.pose.pose.orientation.x = q.x();
        o.pose.pose.orientation.y = q.y();
        o.pose.pose.orientation.z = q.z();
        o.pose.pose.orientation.w = q.w();
        o.twist.twist.linear.x = body_.v;
        o.twist.twist.angular.z = body_.r;
        odom_pub_->publish(o);

        geometry_msgs::msg::TransformStamped tf;
        tf.header = o.header;
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = body_.x;
        tf.transform.translation.y = body_.y;
        tf.transform.rotation = o.pose.pose.orientation;
        tf_->sendTransform(tf);
    }

    void vision()
    {
        bur_msgs::msg::CVDetections out;
        out.header.stamp = now();
        const double b = body_.bearingTo(target_x_, target_y_);
        if (inFieldOfView(b, cam_) && body_.rangeTo(target_x_, target_y_) < max_range_m_) {
            bur_msgs::msg::CVDetection d;
            d.label = static_cast<int32_t>(target_label_);
            d.conf = 0.9f;
            d.bbox.pose.position.x = pixelForBearing(b, cam_);
            out.detected.push_back(d);
        }
        vision_pub_->publish(out);
    }

    double surge_to_freq_, turn_gain_, cmd_timeout_s_;
    double target_x_, target_y_, max_range_m_;
    int64_t target_label_;
    CameraModel cam_;
    sim::Unicycle body_;

    bur_msgs::msg::GaitCommand gait_;
    bool have_gait_ = false;
    rclcpp::Time last_gait_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_step_{0, 0, RCL_ROS_TIME};

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<bur_msgs::msg::CVDetections>::SharedPtr vision_pub_;
    rclcpp::Subscription<bur_msgs::msg::GaitCommand>::SharedPtr gait_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
    rclcpp::TimerBase::SharedPtr step_timer_, vision_timer_;
};

}  // namespace bur_autonomy

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bur_autonomy::BenchSimNode>());
    rclcpp::shutdown();
    return 0;
}
