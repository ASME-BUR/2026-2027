#include "bur_autonomy/leaves.hpp"

#include <cmath>

#include "bur_autonomy/geometry.hpp"
#include "bur_autonomy/register.hpp"

namespace bur_autonomy
{

namespace
{
struct Point
{
    double x;
    double y;
};
}  // namespace

void ContextAction::beginLeaf()
{
    active_s_ = 0.0;
    if (ctx_->on_leaf_start) { ctx_->on_leaf_start(registrationName()); }
}

// ---------------------------------------------------------------------------
// TurnToHeading
// ---------------------------------------------------------------------------

BT::PortsList TurnToHeading::providedPorts()
{
    // heading_rad and label have NO default: which one is required depends on
    // `source`, and onStart checks it.
    return {
        BT::InputPort<std::string>("source", "absolute", "absolute | detection"),
        BT::InputPort<double>("heading_rad", "target heading, odom frame (source=absolute)"),
        BT::InputPort<int>("label", "detection label to face (source=detection)"),
        BT::InputPort<double>("tolerance_rad", 0.1, "success band"),
        BT::InputPort<double>("timeout_s", 20.0, "fail after this much active time"),
    };
}

BT::NodeStatus TurnToHeading::onStart()
{
    beginLeaf();
    const std::string source = requireInput<std::string>("source");
    if (source == "absolute") {
        from_detection_ = false;
        heading_ = requireInput<double>("heading_rad");
    } else if (source == "detection") {
        from_detection_ = true;
        label_ = requireInput<int>("label");
    } else {
        throw BT::RuntimeError("TurnToHeading: source must be 'absolute' or 'detection', got '" +
                               source + "'");
    }
    tolerance_ = requireInput<double>("tolerance_rad");
    timeout_ = requireInput<double>("timeout_s");
    settled_ = 0;
    return step();
}

BT::NodeStatus TurnToHeading::onRunning()
{
    accumulate();
    return step();
}

BT::NodeStatus TurnToHeading::step()
{
    markActive();
    if (active_s_ > timeout_) {
        ctx_->info("TurnToHeading: timeout");
        return BT::NodeStatus::FAILURE;
    }

    double target = heading_;
    if (from_detection_) {
        const Detection * d = ctx_->sighting(label_, ctx_->cfg.min_conf);
        // detection_lost_s > detection_max_age_s (LeafConfig invariant): when
        // this fails as lost, IsDetected is false on the retry and the search
        // runs again.
        if (d == nullptr || ctx_->now_s - d->stamp_s > ctx_->cfg.detection_lost_s) {
            ctx_->info("TurnToHeading: lost label " + std::to_string(label_));
            return BT::NodeStatus::FAILURE;
        }
        target = wrapAngle(d->yaw_at_receipt + d->bearing_rad);
    }

    const double err = wrapAngle(target - ctx_->yaw);
    if (std::fabs(err) < tolerance_) {
        if (++settled_ >= ctx_->cfg.settle_ticks) { return BT::NodeStatus::SUCCESS; }
    } else {
        settled_ = 0;
    }

    ctx_->surge_mps = ctx_->cfg.turn_surge_mps;
    ctx_->yaw_rate_rps = headingToYawRate(err, ctx_->cfg.heading_kp, ctx_->cfg.max_yaw_rate_rps);
    ctx_->enable = true;
    return BT::NodeStatus::RUNNING;
}

// ---------------------------------------------------------------------------
// SwimForward
// ---------------------------------------------------------------------------

BT::PortsList SwimForward::providedPorts()
{
    return {
        BT::InputPort<double>("distance_m", "distance to cover"),
        BT::InputPort<double>("speed_mps", "forward speed, > 0"),
        BT::InputPort<double>("timeout_s", 0.0, "0 = twice the expected time"),
        // No default: absent means "use the manager's swim_forward_mode".
        BT::InputPort<std::string>("mode", "timed | odometry (overrides the parameter)"),
    };
}

BT::NodeStatus SwimForward::onStart()
{
    beginLeaf();
    distance_ = requireInput<double>("distance_m");
    speed_ = requireInput<double>("speed_mps");
    if (!(speed_ > 0.0) || !(distance_ >= 0.0)) {
        // No reverse: the controller takes abs(surge).
        throw BT::RuntimeError("SwimForward: speed_mps must be > 0 and distance_m >= 0");
    }
    std::string mode = ctx_->cfg.swim_forward_mode;
    if (auto m = getInput<std::string>("mode")) { mode = m.value(); }
    if (mode != "timed" && mode != "odometry") {
        throw BT::RuntimeError("SwimForward: mode must be 'timed' or 'odometry', got '" + mode + "'");
    }
    timed_ = (mode == "timed");
    timeout_ = requireInput<double>("timeout_s");
    if (timeout_ <= 0.0) { timeout_ = 2.0 * distance_ / speed_; }
    hold_yaw_ = ctx_->yaw;
    start_x_ = ctx_->x;
    start_y_ = ctx_->y;
    return step();
}

BT::NodeStatus SwimForward::onRunning()
{
    accumulate();
    return step();
}

BT::NodeStatus SwimForward::step()
{
    markActive();
    const bool done = timed_
        ? active_s_ >= distance_ / speed_
        : horizontalDistance(Point{start_x_, start_y_}, Point{ctx_->x, ctx_->y}) >= distance_;
    if (done) { return BT::NodeStatus::SUCCESS; }
    if (active_s_ > timeout_) {
        ctx_->info("SwimForward: timeout");
        return BT::NodeStatus::FAILURE;
    }
    ctx_->surge_mps = speed_;
    ctx_->yaw_rate_rps = headingToYawRate(wrapAngle(hold_yaw_ - ctx_->yaw), ctx_->cfg.heading_kp,
                                          ctx_->cfg.max_yaw_rate_rps);
    ctx_->enable = true;
    return BT::NodeStatus::RUNNING;
}

// ---------------------------------------------------------------------------
// SearchPattern
// ---------------------------------------------------------------------------

BT::PortsList SearchPattern::providedPorts()
{
    return {
        BT::InputPort<int>("label", "detection label to look for"),
        BT::InputPort<double>("sweep_rad", "total sweep, centred on the starting heading"),
        BT::InputPort<double>("timeout_s", 30.0, "fail after this much active time"),
    };
}

BT::NodeStatus SearchPattern::onStart()
{
    beginLeaf();
    label_ = requireInput<int>("label");
    half_sweep_ = std::fabs(requireInput<double>("sweep_rad")) / 2.0;
    timeout_ = requireInput<double>("timeout_s");
    start_yaw_ = ctx_->yaw;
    leg_ = 0;
    return step();
}

BT::NodeStatus SearchPattern::onRunning()
{
    accumulate();
    return step();
}

BT::NodeStatus SearchPattern::step()
{
    markActive();
    const Detection * d = ctx_->sighting(label_, ctx_->cfg.min_conf);
    if (d != nullptr && ctx_->now_s - d->stamp_s <= ctx_->cfg.detection_max_age_s) {
        return BT::NodeStatus::SUCCESS;
    }
    if (active_s_ > timeout_) {
        ctx_->info("SearchPattern: timeout");
        return BT::NodeStatus::FAILURE;
    }
    // Legs alternate +half, -half, +half, ... around the starting heading.
    double target = wrapAngle(start_yaw_ + ((leg_ % 2 == 0) ? half_sweep_ : -half_sweep_));
    double err = wrapAngle(target - ctx_->yaw);
    if (std::fabs(err) < ctx_->cfg.search_leg_tolerance_rad) {
        ++leg_;
        target = wrapAngle(start_yaw_ + ((leg_ % 2 == 0) ? half_sweep_ : -half_sweep_));
        err = wrapAngle(target - ctx_->yaw);
    }
    ctx_->surge_mps = ctx_->cfg.turn_surge_mps;
    ctx_->yaw_rate_rps = headingToYawRate(err, ctx_->cfg.heading_kp, ctx_->cfg.max_yaw_rate_rps);
    ctx_->enable = true;
    return BT::NodeStatus::RUNNING;
}

// ---------------------------------------------------------------------------
// IsDetected
// ---------------------------------------------------------------------------

BT::PortsList IsDetected::providedPorts()
{
    return {
        BT::InputPort<int>("label", "detection label"),
        BT::InputPort<double>("min_conf", "overrides the manager's min_conf"),
    };
}

BT::NodeStatus IsDetected::tick()
{
    auto label = getInput<int>("label");
    if (!label) {
        throw BT::RuntimeError("IsDetected: missing required port [label]: " + label.error());
    }
    double min_conf = ctx_->cfg.min_conf;
    if (auto c = getInput<double>("min_conf")) { min_conf = c.value(); }
    const Detection * d = ctx_->sighting(label.value(), min_conf);
    const bool fresh = d != nullptr && ctx_->now_s - d->stamp_s <= ctx_->cfg.detection_max_age_s;
    return fresh ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// Deferred depth placeholders (D-A2)
// ---------------------------------------------------------------------------

BT::PortsList HoldDepth::providedPorts()
{
    return {
        BT::InputPort<double>("depth_m", "unused until D-3"),
        BT::InputPort<double>("tolerance_m", "unused until D-3"),
        BT::InputPort<double>("timeout_s", "unused until D-3"),
    };
}

BT::NodeStatus HoldDepth::tick()
{
    ctx_->active_leaf = "HoldDepth";
    ctx_->info("HoldDepth: deferred, blocked by D-3 / C-8 (no depth control yet); failing");
    return BT::NodeStatus::FAILURE;
}

BT::PortsList Surface::providedPorts()
{
    return {BT::InputPort<double>("timeout_s", "unused until F-4 / D-3")};
}

BT::NodeStatus Surface::tick()
{
    ctx_->active_leaf = "Surface";
    ctx_->info("Surface: deferred, needs F-4 and D-3 / C-8 (no depth control yet); failing");
    return BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// Registration
// ---------------------------------------------------------------------------

void registerLeaves(BT::BehaviorTreeFactory & factory, const ContextPtr & ctx)
{
    factory.registerNodeType<TurnToHeading>("TurnToHeading", ctx);
    factory.registerNodeType<SwimForward>("SwimForward", ctx);
    factory.registerNodeType<SearchPattern>("SearchPattern", ctx);
    factory.registerNodeType<IsDetected>("IsDetected", ctx);
    factory.registerNodeType<HoldDepth>("HoldDepth", ctx);
    factory.registerNodeType<Surface>("Surface", ctx);
}

namespace
{
MissionTree finish(std::unique_ptr<BT::BehaviorTreeFactory> f)
{
    MissionTree m;
    m.tree = f->createTree("MainTree");
    m.factory = std::move(f);
    return m;
}
}  // namespace

MissionTree buildMissionTree(const std::string & xml_path, const ContextPtr & ctx)
{
    auto f = std::make_unique<BT::BehaviorTreeFactory>();
    registerLeaves(*f, ctx);
    f->registerBehaviorTreeFromFile(xml_path);
    return finish(std::move(f));
}

MissionTree buildMissionTreeFromText(const std::string & xml_text, const ContextPtr & ctx)
{
    auto f = std::make_unique<BT::BehaviorTreeFactory>();
    registerLeaves(*f, ctx);
    f->registerBehaviorTreeFromText(xml_text);
    return finish(std::move(f));
}

}  // namespace bur_autonomy
