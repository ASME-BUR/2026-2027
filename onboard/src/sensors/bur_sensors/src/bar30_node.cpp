#include "bar30_node.hpp"

#include "i2c_bus.h"

Bar30Node::Bar30Node() : rclcpp::Node("bar30")
{
    declare_parameter("i2c_bus", 1);
    declare_parameter("rate", 30);
    declare_parameter("fluid_density", 1000.0);
    declare_parameter("depth_variance", 0.005);
    declare_parameter("frame_id", "base_link");

    // TODO(hw): bur-onboard used 1005.0 in the launch file and 1029.0 as the
    // node default, with no note about which pool that was for. Fresh water is
    // ~997-1000, sea water ~1025. Pick deliberately; a 3% density error is a
    // 3% depth error.
    depth_variance_ = get_parameter("depth_variance").as_double();

    pi_ = pigpio_start(nullptr, nullptr);   // no-op; see README.md, "Hardware notes"

    sensor_ = std::make_unique<MS5837>(
        pi_, static_cast<uint8_t>(get_parameter("i2c_bus").as_int()));

    if (!sensor_->init()) {
        RCLCPP_ERROR(get_logger(),
                     "Bar30 not detected on /dev/i2c-%ld. Is the sensor wired, "
                     "and is the i2c-dev module loaded?",
                     get_parameter("i2c_bus").as_int());
        return;
    }

    sensor_->setFluidDensity(
        static_cast<float>(get_parameter("fluid_density").as_double()));
    sensor_->setOSR(5);

    // TODO(verify): bur-onboard called setModel(1) == MS5837_02BA right after
    // init(), which OVERRIDES the model that init() just auto-detected from the
    // PROM version word. A Bar30 is a 30BA part, so this looks like it was
    // forcing the wrong conversion polynomial. Check which model the detection
    // reports and then delete this comment or restore an explicit setModel.

    depth_pub_ = create_publisher<nav_msgs::msg::Odometry>("depth_sensor", 10);
    pressure_pub_ = create_publisher<std_msgs::msg::Float32>("pressure", 10);
    temp_pub_ = create_publisher<std_msgs::msg::Float32>("temp", 10);

    const int period = 1000 / static_cast<int>(get_parameter("rate").as_int());
    timer_ = create_wall_timer(std::chrono::milliseconds(period),
                               std::bind(&Bar30Node::timerCallback, this));
    ok_ = true;
    RCLCPP_INFO(get_logger(), "Bar30 publishing depth_sensor at %d Hz",
                1000 / period);
}

Bar30Node::~Bar30Node()
{
    pigpio_stop(pi_);
}

void Bar30Node::timerCallback()
{
    if (!ok_) { return; }

    sensor_->read();

    nav_msgs::msg::Odometry depth;
    depth.header.stamp = now();
    depth.header.frame_id = get_parameter("frame_id").as_string();
    depth.child_frame_id = "";

    // NWU: depth below the surface is negative z. Sign carried over from
    // bur-onboard so the EKF config does not have to change.
    depth.pose.pose.position.z = -sensor_->depth();

    // Diagonal only. Everything this message does not measure gets a large
    // variance so the filter ignores it rather than trusting a zero.
    const double big = 1e6;
    depth.pose.covariance[0]  = big;              // x
    depth.pose.covariance[7]  = big;              // y
    depth.pose.covariance[14] = depth_variance_;  // z  <- the real measurement
    depth.pose.covariance[21] = big;              // roll
    depth.pose.covariance[28] = big;              // pitch
    depth.pose.covariance[35] = big;              // yaw
    depth_pub_->publish(depth);

    std_msgs::msg::Float32 p;
    p.data = sensor_->pressure();
    pressure_pub_->publish(p);

    std_msgs::msg::Float32 t;
    t.data = sensor_->temperature();
    temp_pub_->publish(t);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Bar30Node>());
    rclcpp::shutdown();
    return 0;
}
