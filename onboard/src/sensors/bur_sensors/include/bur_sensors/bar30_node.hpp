#ifndef BUR_BAR30_NODE_HPP
#define BUR_BAR30_NODE_HPP

// Ported from bur-onboard/src/sensors/bur_sensors/bar30_node.cpp.
//
// Changes:
//   * no pigpiod. The node opens /dev/i2c-<bus> locally, so it must RUN ON THE
//     PI. Last year it could run anywhere and reach the Pi over TCP, which was
//     a liability nobody wanted.
//   * `pi_address` / `pi_port` parameters replaced by `i2c_bus`.
//   * covariance is now a sane diagonal. bur-onboard filled all 36 entries of
//     the 6x6 with 0.005, including the off-diagonals, which is a singular
//     matrix. It never bit because the EKF only consumed the z term.
//   * dropped the duplicate publisher. bur-onboard published the same reading
//     on both `depth_sensor` (PoseWithCovarianceStamped) and `depth_sensor2`
//     (Odometry) because the EKF wanted one and joy_command wanted the other.
//     Only the Odometry one is kept; see TODO below.

#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include "MS5837.h"

class Bar30Node : public rclcpp::Node
{
public:
    Bar30Node();
    ~Bar30Node();

private:
    void timerCallback();

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr depth_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pressure_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temp_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<MS5837> sensor_;
    int pi_ = -1;
    bool ok_ = false;
    double depth_variance_ = 0.005;
};

#endif
