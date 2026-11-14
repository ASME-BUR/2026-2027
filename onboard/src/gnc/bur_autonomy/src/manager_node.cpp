// bur_autonomy manager (A-1): ticks a BehaviorTree.CPP v4 mission tree at
// 10 Hz and publishes bur_msgs/SwimCommand. See PLAN.md.
//
// Safety notes, each of which bit the previous vehicle or the plan review:
//   - The tree is built in the constructor BEFORE the timer exists. The old
//     manager started ticking before its tree was built.
//   - It never calls rclcpp::shutdown(). When a mission ends it coasts for
//     idle_coast_s and then stops publishing, so a hand-started teleop is not
//     drowned out, and the node stays up for logs and ~/status.
//   - bur_controller's have_state_ never goes false once odometry has
//     arrived, so THIS node's odom staleness check is the only guard against
//     stale odometry: no tick while it is stale, abort after odom_loss_abort_s.
//   - Every parameter comes from one table, and an unknown override (a typo
//     in the YAML) is a startup error, not a silently ignored value.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "bur_msgs/msg/cv_detections.hpp"
#include "bur_msgs/msg/swim_command.hpp"

#include "bur_autonomy/detection.hpp"
#include "bur_autonomy/register.hpp"

namespace bur_autonomy
{

namespace
{

// ---- the one parameter table ----------------------------------------------
struct ParamSpec
{
    const char * name;
    rclcpp::ParameterValue default_value;
    const char * description;
};

const std::vector<ParamSpec> & paramTable()
{
    static const std::vector<ParamSpec> table = {
        {"behavior_tree", rclcpp::ParameterValue(std::string("")), "absolute path to the mission XML"},
        {"odom_topic", rclcpp::ParameterValue(std::string("/odometry/filtered")), ""},
        {"vision_topic", rclcpp::ParameterValue(std::string("/vision")),
         "bur_msgs/CVDetections. NOTE yolo.py publishes yolo_msgs on yolo_detections (D-A4)"},
        {"swim_topic", rclcpp::ParameterValue(std::string("/swim_command")), ""},
        {"tick_rate_hz", rclcpp::ParameterValue(10), ""},
        {"autostart", rclcpp::ParameterValue(false), "start the mission without ~/start"},
        {"odom_timeout_s", rclcpp::ParameterValue(0.5), "older odometry pauses the tree"},
        {"odom_loss_abort_s", rclcpp::ParameterValue(2.0), "pause this long -> mission FAILURE"},
        {"idle_coast_s", rclcpp::ParameterValue(1.0), "publish enable=false this long when going idle"},
        {"heading_kp", rclcpp::ParameterValue(1.0), ""},
        {"max_yaw_rate_rps", rclcpp::ParameterValue(0.4), "TODO(hw): M-3"},
        {"turn_surge_mps", rclcpp::ParameterValue(0.15), "see controller.yaml freq_min"},
        {"settle_ticks", rclcpp::ParameterValue(3), ""},
        {"search_leg_tolerance_rad", rclcpp::ParameterValue(0.1), ""},
        {"detection_max_age_s", rclcpp::ParameterValue(1.5), "must be < detection_lost_s"},
        {"detection_lost_s", rclcpp::ParameterValue(2.0), "must be > detection_max_age_s"},
        {"min_conf", rclcpp::ParameterValue(0.5), ""},
        {"swim_forward_mode", rclcpp::ParameterValue(std::string("timed")), "timed | odometry"},
        {"camera_image_width_px", rclcpp::ParameterValue(1280.0), "TODO(hw)"},
        {"camera_hfov_rad", rclcpp::ParameterValue(1.92), "TODO(hw)"},
    };
    return table;
}

bool whitelisted(const std::string & name)
{
    return name == "use_sim_time" || name.rfind("qos_overrides.", 0) == 0;
}

std::string statusName(BT::NodeStatus s)
{
    switch (s) {
    case BT::NodeStatus::IDLE: return "IDLE";
    case BT::NodeStatus::RUNNING: return "RUNNING";
    case BT::NodeStatus::SUCCESS: return "SUCCESS";
    case BT::NodeStatus::FAILURE: return "FAILURE";
    case BT::NodeStatus::SKIPPED: return "SKIPPED";
    }
    return "?";
}

}  // namespace

class ManagerNode : public rclcpp::Node
{
public:
    explicit ManagerNode(const rclcpp::NodeOptions & options)
    : rclcpp::Node("bur_autonomy", options)
    {
        // 1. parameters (declare + read from the one table; reject unknowns)
        for (const auto & p : paramTable()) {
            rcl_interfaces::msg::ParameterDescriptor d;
            d.description = p.description;
            declare_parameter(p.name, p.default_value, d);
        }
        rejectUnknownOverrides();
        readParameters();

        // 2. context, factory, tree. Throws on a bad tree: exit non-zero.
        ctx_ = std::make_shared<MissionContext>();
        ctx_->cfg = cfg_;
        ctx_->log = [this](const std::string & m) { RCLCPP_INFO(get_logger(), "%s", m.c_str()); };
        ctx_->on_leaf_start = [this](const std::string & leaf) {
            RCLCPP_INFO(get_logger(), "leaf start: %s", leaf.c_str());
        };
        const std::string tree_path = get_parameter("behavior_tree").as_string();
        if (tree_path.empty()) {
            throw std::runtime_error("parameter behavior_tree is empty");
        }
        mission_ = buildMissionTree(tree_path, ctx_);
        RCLCPP_INFO(get_logger(), "loaded %s", tree_path.c_str());

        // 3. I/O
        swim_pub_ = create_publisher<bur_msgs::msg::SwimCommand>(
            get_parameter("swim_topic").as_string(), 10);
        status_pub_ = create_publisher<std_msgs::msg::String>("~/status", 10);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            get_parameter("odom_topic").as_string(), 10,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { onOdom(*m); });
        vision_sub_ = create_subscription<bur_msgs::msg::CVDetections>(
            get_parameter("vision_topic").as_string(), 10,
            [this](bur_msgs::msg::CVDetections::ConstSharedPtr m) { onVision(*m); });
        start_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/start", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                res->success = start(res->message);
            });
        stop_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/stop", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                stop("stopped by ~/stop");
                res->success = true;
                res->message = "idle";
            });

        if (get_parameter("autostart").as_bool()) {
            std::string msg;
            start(msg);
        }

        // 4. timer, last
        const int hz = std::max<int64_t>(1, get_parameter("tick_rate_hz").as_int());
        period_s_ = 1.0 / hz;
        timer_ = create_wall_timer(std::chrono::milliseconds(1000 / hz), [this]() { onTick(); });
    }

private:
    enum class State { kIdle, kActive };

    void rejectUnknownOverrides()
    {
        std::set<std::string> known;
        for (const auto & p : paramTable()) { known.insert(p.name); }
        std::vector<std::string> unknown;
        for (const auto & kv : get_node_parameters_interface()->get_parameter_overrides()) {
            if (!known.count(kv.first) && !whitelisted(kv.first)) { unknown.push_back(kv.first); }
        }
        if (!unknown.empty()) {
            std::ostringstream s;
            s << "unknown parameter(s) for bur_autonomy (typo in the YAML?):";
            for (const auto & u : unknown) { s << " " << u; }
            throw std::runtime_error(s.str());
        }
    }

    void readParameters()
    {
        cfg_.heading_kp = get_parameter("heading_kp").as_double();
        cfg_.max_yaw_rate_rps = get_parameter("max_yaw_rate_rps").as_double();
        cfg_.turn_surge_mps = get_parameter("turn_surge_mps").as_double();
        cfg_.settle_ticks = static_cast<int>(get_parameter("settle_ticks").as_int());
        cfg_.search_leg_tolerance_rad = get_parameter("search_leg_tolerance_rad").as_double();
        cfg_.detection_max_age_s = get_parameter("detection_max_age_s").as_double();
        cfg_.detection_lost_s = get_parameter("detection_lost_s").as_double();
        cfg_.min_conf = get_parameter("min_conf").as_double();
        cfg_.swim_forward_mode = get_parameter("swim_forward_mode").as_string();
        cam_.image_width_px = get_parameter("camera_image_width_px").as_double();
        cam_.hfov_rad = get_parameter("camera_hfov_rad").as_double();
        odom_timeout_s_ = get_parameter("odom_timeout_s").as_double();
        odom_loss_abort_s_ = get_parameter("odom_loss_abort_s").as_double();
        idle_coast_s_ = get_parameter("idle_coast_s").as_double();

        if (!(cfg_.detection_lost_s > cfg_.detection_max_age_s)) {
            throw std::runtime_error(
                "detection_lost_s must be > detection_max_age_s, or a lost target "
                "would still count as detected on the retry and the search would never run");
        }
        if (cfg_.swim_forward_mode != "timed" && cfg_.swim_forward_mode != "odometry") {
            throw std::runtime_error("swim_forward_mode must be 'timed' or 'odometry'");
        }
        if (cfg_.turn_surge_mps <= 0.0) {
            throw std::runtime_error(
                "turn_surge_mps must be > 0: at zero surge the controller sends MODE_HOLD "
                "(a static bend) and the vehicle cannot turn");
        }
    }

    double nowS() { return now().seconds(); }

    void onOdom(const nav_msgs::msg::Odometry & m)
    {
        tf2::Quaternion q(m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                          m.pose.pose.orientation.z, m.pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        ctx_->x = m.pose.pose.position.x;
        ctx_->y = m.pose.pose.position.y;
        ctx_->z = m.pose.pose.position.z;
        ctx_->yaw = yaw;
        ctx_->have_odom = true;
        // Staleness is judged by receipt time: robust to clock offsets between
        // the MCU, the EKF and this node.
        last_odom_rx_s_ = nowS();
    }

    void onVision(const bur_msgs::msg::CVDetections & m)
    {
        // yolo.py sets no header, so detections are stamped on receipt, with
        // the vehicle's yaw at receipt (the bearing is relative to it).
        const double t = nowS();
        for (const auto & d : m.detected) {
            Detection det;
            det.label = d.label;
            det.conf = d.conf;
            det.bearing_rad = detectionBearing(d.bbox.pose.position.x, cam_);
            det.stamp_s = t;
            det.yaw_at_receipt = ctx_->yaw;
            ctx_->recordSighting(det);
        }
    }

    bool start(std::string & message)
    {
        if (state_ == State::kActive) {
            message = "refused: a mission is already active";
            return false;
        }
        // Fresh start: tree statuses, retry counters and leaf state reset
        // (leaves re-initialise in onStart), plus this node's own timers.
        mission_.tree.haltTree();
        last_tick_s_ = -1.0;
        odom_bad_since_s_ = -1.0;
        idle_since_s_ = -1.0;
        last_status_ = BT::NodeStatus::IDLE;
        state_ = State::kActive;
        message = "mission started";
        RCLCPP_INFO(get_logger(), "%s", message.c_str());
        return true;
    }

    void stop(const std::string & why)
    {
        mission_.tree.haltTree();
        if (state_ != State::kIdle || idle_since_s_ < 0.0) {
            idle_since_s_ = nowS();
        }
        state_ = State::kIdle;
        RCLCPP_INFO(get_logger(), "idle: %s", why.c_str());
    }

    void publishSwim(bool enable)
    {
        bur_msgs::msg::SwimCommand c;
        c.header.stamp = now();
        c.header.frame_id = "base_link";
        if (enable) {
            // Never negative: the controller takes abs(surge), so a negative
            // value would swim forward anyway.
            c.surge_mps = static_cast<float>(std::max(0.0, ctx_->surge_mps));
            c.yaw_rate_rps = static_cast<float>(ctx_->yaw_rate_rps);
        }
        // Depth is deferred (D-3): 0 for both, the fields stay.
        c.depth_m = 0.0f;
        c.pitch_rad = 0.0f;
        c.enable = enable;
        swim_pub_->publish(c);
    }

    void publishStatus(const std::string & state)
    {
        std_msgs::msg::String s;
        s.data = "state=" + state + " tree=" + statusName(last_status_) +
                 " leaf=" + (ctx_->active_leaf.empty() ? "-" : ctx_->active_leaf);
        status_pub_->publish(s);
    }

    void onTick()
    {
        const double t = nowS();
        ctx_->now_s = t;

        if (state_ == State::kIdle) {
            // Coast for a moment after going idle, then go quiet (review: an
            // idle manager publishing every tick would starve teleop). Before
            // the first start, idle_since_s_ < 0 and nothing is published.
            if (idle_since_s_ >= 0.0 && t - idle_since_s_ < idle_coast_s_) {
                publishSwim(false);
            }
            publishStatus("IDLE");
            return;
        }

        // tick_dt_s: clamped to 2 periods so a pause adds at most one period.
        double dt = (last_tick_s_ < 0.0) ? 0.0 : t - last_tick_s_;
        dt = std::clamp(dt, 0.0, 2.0 * period_s_);

        if (!ctx_->have_odom) {
            // Never had odometry (startup: the EKF or bench_sim is still being
            // discovered). That is not a LOSS, so it does not count toward
            // odom_loss_abort_s: wait, coasting, without ticking. Found on the
            // first headless bench run, where autostart beat DDS discovery.
            publishSwim(false);
            publishStatus("WAITING_FOR_ODOM");
            last_tick_s_ = -1.0;
            return;
        }
        const bool odom_ok = (t - last_odom_rx_s_) <= odom_timeout_s_;
        if (!odom_ok) {
            if (odom_bad_since_s_ < 0.0) { odom_bad_since_s_ = t; }
            publishSwim(false);
            publishStatus("PAUSED_NO_ODOM");
            if (t - odom_bad_since_s_ > odom_loss_abort_s_) {
                RCLCPP_ERROR(get_logger(), "odometry lost for %.1f s: mission FAILURE",
                             t - odom_bad_since_s_);
                last_status_ = BT::NodeStatus::FAILURE;
                stop("odometry lost");
            }
            // Not ticked: no leaf accumulates time while paused.
            last_tick_s_ = -1.0;
            return;
        }
        odom_bad_since_s_ = -1.0;
        ctx_->tick_dt_s = dt;
        last_tick_s_ = t;

        ctx_->clearOutputs();
        BT::NodeStatus st;
        try {
            st = mission_.tree.tickOnce();
        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "tree threw: %s -> mission FAILURE", e.what());
            last_status_ = BT::NodeStatus::FAILURE;
            stop("exception");
            publishSwim(false);
            return;
        }
        last_status_ = st;

        if (st == BT::NodeStatus::SUCCESS || st == BT::NodeStatus::FAILURE) {
            RCLCPP_INFO(get_logger(), "mission %s", statusName(st).c_str());
            stop("mission ended");
            publishSwim(false);
            publishStatus("IDLE");
            return;
        }
        // A RetryUntilSuccessful starting its next attempt can return RUNNING
        // with no leaf active: one coast tick, accepted (PLAN.md section 1).
        publishSwim(ctx_->enable);
        publishStatus("ACTIVE");
    }

    LeafConfig cfg_;
    CameraModel cam_;
    std::shared_ptr<MissionContext> ctx_;
    MissionTree mission_;

    rclcpp::Publisher<bur_msgs::msg::SwimCommand>::SharedPtr swim_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<bur_msgs::msg::CVDetections>::SharedPtr vision_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_, stop_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_ = State::kIdle;
    BT::NodeStatus last_status_ = BT::NodeStatus::IDLE;
    double period_s_ = 0.1;
    double odom_timeout_s_ = 0.5, odom_loss_abort_s_ = 2.0, idle_coast_s_ = 1.0;
    double last_odom_rx_s_ = 0.0;
    double last_tick_s_ = -1.0;
    double odom_bad_since_s_ = -1.0;
    double idle_since_s_ = -1.0;
};

}  // namespace bur_autonomy

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    int rc = 0;
    try {
        rclcpp::NodeOptions opts;
        opts.automatically_declare_parameters_from_overrides(false);
        rclcpp::spin(std::make_shared<bur_autonomy::ManagerNode>(opts));
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("bur_autonomy"), "%s", e.what());
        rc = 1;
    }
    rclcpp::shutdown();
    return rc;
}
