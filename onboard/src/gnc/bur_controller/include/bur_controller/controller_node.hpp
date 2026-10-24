#ifndef BUR_CONTROLLER_NODE_HPP
#define BUR_CONTROLLER_NODE_HPP

// Outer control loop for the servo-chain vehicle.
//
// This is the package that changed shape most in the port. bur-onboard ran six
// independent cascaded PID axes at 30 Hz and emitted a WrenchStamped, on the
// assumption that the thruster array could produce arbitrary force and torque
// instantly. None of that holds here:
//
//   - there is no sway axis at all
//   - thrust is produced over a tail-beat period (~0.5-3 Hz), not instantly
//   - heading rate is coupled to forward speed
//
// So the loop runs slow (10 Hz by default) and its output is a set of gait
// PARAMETERS, not a force. The fast work happens on the MCU.
//
//   /odometry/filtered  +  /swim_command   ->   /gait_command
//
// TODO(fish): the surge_mps -> frequency_hz map is currently a single gain.
// It must become a measured table; see the plant characterisation step in
// config/controller.yaml.

#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include "bur_msgs/msg/gait_command.hpp"
#include "bur_msgs/msg/swim_command.hpp"

#include "pid.hpp"

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void swimCallback(const bur_msgs::msg::SwimCommand::SharedPtr msg);
    void update();

    void setConstants();
    rcl_interfaces::msg::SetParametersResult
    parametersCallback(const std::vector<rclcpp::Parameter> & parameters);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<bur_msgs::msg::SwimCommand>::SharedPtr swim_sub_;
    rclcpp::Publisher<bur_msgs::msg::GaitCommand>::SharedPtr gait_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    // One loop per controllable axis. Note there are four, not six.
    PID depth_;      // depth error    -> pitch command
    PID pitch_;      // pitch error    -> pitch command trim
    PID yaw_rate_;   // yaw rate error -> turn bias

    bur_msgs::msg::SwimCommand cmd_;       // latest intent
    nav_msgs::msg::Odometry state_;        // latest estimate
    bool have_state_ = false;
    bool have_cmd_ = false;

    rclcpp::Time last_update_;
    rclcpp::Time last_cmd_stamp_;

    // cached parameters
    double surge_to_freq_ = 1.0;
    double freq_min_ = 0.0;
    double freq_max_ = 2.0;
    double amplitude_cruise_ = 0.5;
    double turn_bias_max_ = 0.3;
    double fin_angle_max_ = 0.4;   // max dive-plane deflection, rad
    double cmd_timeout_s_ = 0.5;
};

#endif
