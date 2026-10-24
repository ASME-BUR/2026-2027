#include "controller_node.hpp"

using std::placeholders::_1;

ControllerNode::ControllerNode()
: rclcpp::Node("bur_controller"),
  last_update_(0, 0, RCL_ROS_TIME),
  last_cmd_stamp_(0, 0, RCL_ROS_TIME)
{
    declare_parameter("odom_topic", "/odometry/filtered");
    declare_parameter("swim_topic", "/swim_command");
    declare_parameter("gait_topic", "/gait_command");
    declare_parameter("update_rate", 10);

    // TODO(hw): every gain below is a placeholder. None has been near water.
    declare_parameter("depth.kp", 0.5);
    declare_parameter("depth.ki", 0.0);
    declare_parameter("depth.kd", 0.0);
    declare_parameter("depth.i_limit", 0.5);

    declare_parameter("pitch.kp", 0.5);
    declare_parameter("pitch.ki", 0.0);
    declare_parameter("pitch.kd", 0.0);
    declare_parameter("pitch.i_limit", 0.3);

    declare_parameter("yaw_rate.kp", 0.4);
    declare_parameter("yaw_rate.ki", 0.0);
    declare_parameter("yaw_rate.kd", 0.0);
    declare_parameter("yaw_rate.i_limit", 0.3);

    declare_parameter("surge_to_freq", 2.0);
    declare_parameter("freq_min", 0.0);
    declare_parameter("freq_max", 2.5);
    declare_parameter("amplitude_cruise", 0.5);
    declare_parameter("turn_bias_max", 0.3);
    declare_parameter("fin_angle_max", 0.4);
    declare_parameter("cmd_timeout", 0.5);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        get_parameter("odom_topic").as_string(), 10,
        std::bind(&ControllerNode::odomCallback, this, _1));
    swim_sub_ = create_subscription<bur_msgs::msg::SwimCommand>(
        get_parameter("swim_topic").as_string(), 10,
        std::bind(&ControllerNode::swimCallback, this, _1));
    gait_pub_ = create_publisher<bur_msgs::msg::GaitCommand>(
        get_parameter("gait_topic").as_string(), 10);

    const int rate = get_parameter("update_rate").as_int();
    timer_ = create_wall_timer(std::chrono::milliseconds(1000 / rate),
                               std::bind(&ControllerNode::update, this));

    param_cb_ = add_on_set_parameters_callback(
        std::bind(&ControllerNode::parametersCallback, this, _1));

    setConstants();
}

void ControllerNode::setConstants()
{
    depth_.setGains(get_parameter("depth.kp").as_double(),
                    get_parameter("depth.ki").as_double(),
                    get_parameter("depth.kd").as_double());
    depth_.setIntegralLimit(get_parameter("depth.i_limit").as_double());

    pitch_.setGains(get_parameter("pitch.kp").as_double(),
                    get_parameter("pitch.ki").as_double(),
                    get_parameter("pitch.kd").as_double());
    pitch_.setIntegralLimit(get_parameter("pitch.i_limit").as_double());

    yaw_rate_.setGains(get_parameter("yaw_rate.kp").as_double(),
                       get_parameter("yaw_rate.ki").as_double(),
                       get_parameter("yaw_rate.kd").as_double());
    yaw_rate_.setIntegralLimit(get_parameter("yaw_rate.i_limit").as_double());

    surge_to_freq_     = get_parameter("surge_to_freq").as_double();
    freq_min_          = get_parameter("freq_min").as_double();
    freq_max_          = get_parameter("freq_max").as_double();
    amplitude_cruise_  = get_parameter("amplitude_cruise").as_double();
    turn_bias_max_     = get_parameter("turn_bias_max").as_double();
    fin_angle_max_     = get_parameter("fin_angle_max").as_double();
    cmd_timeout_s_     = get_parameter("cmd_timeout").as_double();
}

rcl_interfaces::msg::SetParametersResult
ControllerNode::parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
    for (const auto & p : parameters) {
        RCLCPP_INFO(get_logger(), "%s := %s", p.get_name().c_str(),
                    p.value_to_string().c_str());
    }
    // Carried over from bur-onboard: gains are retunable at runtime via rqt,
    // which is how the pool tests were actually done.
    setConstants();

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
}

void ControllerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    state_ = *msg;
    have_state_ = true;
}

void ControllerNode::swimCallback(const bur_msgs::msg::SwimCommand::SharedPtr msg)
{
    cmd_ = *msg;
    have_cmd_ = true;
    last_cmd_stamp_ = now();
}

void ControllerNode::update()
{
    bur_msgs::msg::GaitCommand out;
    out.header.stamp = now();
    out.header.frame_id = "base_link";

    // Deadman. bur-onboard gated on joystick button 9, which meant the
    // controller published zeros forever if no gamepad was connected. Here the
    // gate is a stale-command timeout instead, so autonomy and teleop behave
    // the same way.
    const bool stale = !have_cmd_ ||
        (now() - last_cmd_stamp_).seconds() > cmd_timeout_s_;

    if (stale || !have_state_ || !cmd_.enable) {
        out.mode = bur_msgs::msg::GaitCommand::MODE_COAST;
        depth_.reset();
        pitch_.reset();
        yaw_rate_.reset();
        last_update_ = now();
        gait_pub_->publish(out);
        return;
    }

    const rclcpp::Time t = now();
    if (last_update_.nanoseconds() == 0) {
        last_update_ = t;
        return;
    }
    const double dt = (t - last_update_).seconds();
    last_update_ = t;

    // ---- current attitude ----
    tf2::Quaternion q(state_.pose.pose.orientation.x, state_.pose.pose.orientation.y,
                      state_.pose.pose.orientation.z, state_.pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    (void)roll;   // the body has no roll authority
    (void)yaw;    // heading is commanded as a rate, not a pose

    // ---- surge -> tail-beat frequency ----
    // TODO(hw): replace this single gain with a measured lookup table of
    // (frequency, amplitude) -> steady-state surge speed. Until the vehicle has
    // been in a pool this is a guess, and it is the main reason commanded speed
    // will not match actual speed.
    double freq = std::abs(cmd_.surge_mps) * surge_to_freq_;
    freq = clamp_d(freq, freq_min_, freq_max_);

    // ---- yaw rate -> turn bias ----
    const double yaw_rate_err = cmd_.yaw_rate_rps - state_.twist.twist.angular.z;
    double turn_bias = yaw_rate_.computeCommand(yaw_rate_err, dt);
    turn_bias = clamp_d(turn_bias, -turn_bias_max_, turn_bias_max_);

    // ---- depth + pitch -> fin deflection ----
    // Cascade: depth error sets a pitch target, the pitch loop tracks it, and
    // the output is a fin deflection (D-3: the Z-only chain cannot pitch).
    //
    // TODO(fish): this cascade assumes the fins sit at an extremity, producing
    // a pitching MOMENT that the body then converts to depth change via
    // surge x sin(pitch). If the fins end up near the centre of mass they
    // produce near-direct lift instead, and the pitch inner loop should be
    // dropped so depth maps straight to fin angle. See D-3's placement table.
    //
    // TODO(fish): fin lift scales as v^2 and this loop does not know the
    // vehicle's forward speed, so at low speed it winds up against an error it
    // cannot affect. That is task C-8 and it matters more with fins than it did
    // without them.
    const double depth_err = cmd_.depth_m - state_.pose.pose.position.z;
    const double pitch_from_depth = depth_.computeCommand(depth_err, dt);
    const double pitch_err = (cmd_.pitch_rad + pitch_from_depth) - pitch;
    double fin_angle = pitch_.computeCommand(pitch_err, dt);
    fin_angle = clamp_d(fin_angle, -fin_angle_max_, fin_angle_max_);

    // ---- assemble ----
    out.mode = (std::abs(cmd_.yaw_rate_rps) > 0.05)
                   ? bur_msgs::msg::GaitCommand::MODE_TURN
                   : bur_msgs::msg::GaitCommand::MODE_CRUISE;
    if (freq <= freq_min_) {
        out.mode = bur_msgs::msg::GaitCommand::MODE_HOLD;
    }
    out.frequency_hz = static_cast<float>(freq);
    out.amplitude    = static_cast<float>(amplitude_cruise_);
    out.wavelength   = 1.0f;   // TODO(hw): tune, then move to a parameter
    out.turn_bias    = static_cast<float>(turn_bias);
    out.fin_angle_rad = static_cast<float>(fin_angle);

    gait_pub_->publish(out);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
