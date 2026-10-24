// Pi-side gait generator.
//
// In the intended architecture the MCU owns the phase accumulator and
// subscribes to /gait_command directly over micro-ROS, so this node is NOT in
// the control path on the vehicle. It exists for two things that matter anyway:
//
//   1. bench bring-up, where having the angles visible as a ROS topic is worth
//      far more than deterministic timing
//   2. publishing JointState so robot_state_publisher can animate the URDF in
//      RViz, which is the cheapest way to see a gait before the hull is wet
//
// TODO(decide): if the MCU-owns-phase split is abandoned, this becomes the
// primary actuation path and needs a real-time story: a lock-free timer, SCHED_FIFO,
// and a hard look at whether the Pi can hold 50 Hz under load.

#include <chrono>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "bur_gait/gait.hpp"
#include "bur_msgs/msg/gait_command.hpp"
#include "bur_msgs/msg/servo_chain_command.hpp"

using std::placeholders::_1;

class GaitNode : public rclcpp::Node
{
public:
    GaitNode() : rclcpp::Node("bur_gait"), start_(now())
    {
        declare_parameter("gait_topic", "/gait_command");
        declare_parameter("servo_topic", "/servo_chain_command");
        declare_parameter("joint_state_topic", "/joint_states");
        declare_parameter("update_rate", 50);

        declare_parameter("n_joints", 6);
        declare_parameter("envelope_exponent", 2.0);
        declare_parameter("joint_limit_rad", 0.6);
        declare_parameter("max_rate_rad_s", 6.0);
        declare_parameter("joint_prefix", "joint_");

        cfg_.n_joints = static_cast<std::size_t>(get_parameter("n_joints").as_int());
        cfg_.envelope_exponent = get_parameter("envelope_exponent").as_double();
        cfg_.joint_limit_rad = get_parameter("joint_limit_rad").as_double();
        cfg_.max_rate_rad_s = get_parameter("max_rate_rad_s").as_double();
        prefix_ = get_parameter("joint_prefix").as_string();

        last_.assign(cfg_.n_joints, 0.0);

        gait_sub_ = create_subscription<bur_msgs::msg::GaitCommand>(
            get_parameter("gait_topic").as_string(), 10,
            std::bind(&GaitNode::gaitCallback, this, _1));
        servo_pub_ = create_publisher<bur_msgs::msg::ServoChainCommand>(
            get_parameter("servo_topic").as_string(), 10);
        js_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            get_parameter("joint_state_topic").as_string(), 10);

        const int rate = get_parameter("update_rate").as_int();
        timer_ = create_wall_timer(std::chrono::milliseconds(1000 / rate),
                                   std::bind(&GaitNode::update, this));
        last_tick_ = now();
    }

private:
    void gaitCallback(const bur_msgs::msg::GaitCommand::SharedPtr msg)
    {
        mode_ = msg->mode;
        p_.frequency_hz = msg->frequency_hz;
        p_.amplitude = msg->amplitude;
        p_.wavelength = msg->wavelength;
        p_.turn_bias = msg->turn_bias;
        // msg->fin_angle_rad is intentionally not consumed here. Fins are a
        // separate actuator and this node is only the chain preview.
        // TODO(port): route fin_angle_rad to the fin servo once D-3 settles how
        // many fins there are and where they sit. It needs a home in
        // ServoChainCommand (or its own message) and a link in the URDF before
        // RViz can show it.
        last_cmd_ = now();
    }

    void update()
    {
        const rclcpp::Time t = now();
        const double dt = (t - last_tick_).seconds();
        last_tick_ = t;

        // Watchdog. A dropped command must coast, never hold a stale angle.
        const bool stale = (last_cmd_.nanoseconds() == 0) ||
                           (t - last_cmd_).seconds() > 0.3;

        std::vector<double> target(cfg_.n_joints, 0.0);

        if (!stale && mode_ != bur_msgs::msg::GaitCommand::MODE_ESTOP &&
            mode_ != bur_msgs::msg::GaitCommand::MODE_COAST) {
            const double elapsed = (t - start_).seconds();
            target = bur_gait::jointAngles(elapsed, p_, cfg_);
        }
        // ESTOP / COAST / stale all fall through with target = all zeros,
        // i.e. the chain straightens rather than freezing mid-beat.

        bur_gait::rateLimit(target, last_, dt, cfg_);
        last_ = target;

        bur_msgs::msg::ServoChainCommand out;
        out.header.stamp = t;
        out.header.frame_id = "base_link";
        out.position_rad.reserve(target.size());
        for (double v : target) {
            out.position_rad.push_back(static_cast<float>(v));
        }
        out.sequence = seq_++;
        servo_pub_->publish(out);

        sensor_msgs::msg::JointState js;
        js.header.stamp = t;
        js.name.reserve(target.size());
        js.position.reserve(target.size());
        for (std::size_t i = 0; i < target.size(); ++i) {
            js.name.push_back(prefix_ + std::to_string(i));
            js.position.push_back(target[i]);
        }
        js_pub_->publish(js);
    }

    rclcpp::Subscription<bur_msgs::msg::GaitCommand>::SharedPtr gait_sub_;
    rclcpp::Publisher<bur_msgs::msg::ServoChainCommand>::SharedPtr servo_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bur_gait::ChainConfig cfg_;
    bur_gait::GaitParams p_;
    std::vector<double> last_;
    std::string prefix_;

    rclcpp::Time start_;
    rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_cmd_{0, 0, RCL_ROS_TIME};
    uint8_t mode_ = 1;   // COAST
    uint8_t seq_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GaitNode>());
    rclcpp::shutdown();
    return 0;
}
