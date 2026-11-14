#ifndef BUR_AUTONOMY_LEAVES_HPP
#define BUR_AUTONOMY_LEAVES_HPP

// Behaviour-tree leaves for a non-holonomic swimming body (A-2).
//
// There is no GoToPose: reaching a point means TurnToHeading, then SwimForward.
// Every leaf writes only surge (>= 0) and yaw rate into the shared context;
// the manager zeroes those before each tick, so a leaf that stops running stops
// commanding motion. Trees must not put action leaves under a Parallel node.
//
// Timing: leaves accumulate ctx.tick_dt_s on ticks where they run, never wall
// time, so a pause (manager not ticking) is not counted as swimming.
//
// Ports: required inputs that are missing throw BT::RuntimeError, which the
// manager turns into a mission FAILURE. Optional inputs are read with
// `if (auto r = getInput<T>(..))` and never declared with a "" default.

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"

#include "bur_autonomy/mission_context.hpp"

namespace bur_autonomy
{

using ContextPtr = std::shared_ptr<MissionContext>;

// Common base: owns the context pointer and the active-time accumulator.
class ContextAction : public BT::StatefulActionNode
{
public:
    ContextAction(const std::string & name, const BT::NodeConfig & config, ContextPtr ctx)
    : BT::StatefulActionNode(name, config), ctx_(std::move(ctx)) {}

protected:
    // Call at the top of onStart(): resets the clock and reports the leaf.
    void beginLeaf();
    // Call at the top of onRunning(): accumulates this tick's dt.
    void accumulate() { active_s_ += ctx_->tick_dt_s; }
    void markActive() { ctx_->active_leaf = registrationName(); }

    template <typename T>
    T requireInput(const std::string & key)
    {
        auto r = getInput<T>(key);
        if (!r) {
            throw BT::RuntimeError(registrationName() + " '" + name() +
                                   "': missing required port [" + key + "]: " + r.error());
        }
        return r.value();
    }

    ContextPtr ctx_;
    double active_s_ = 0.0;
};

// Turn to an absolute heading (odom frame; 0 = heading at power-on) or toward
// the latest sighting of a label.
class TurnToHeading : public ContextAction
{
public:
    using ContextAction::ContextAction;
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {}

private:
    BT::NodeStatus step();
    bool from_detection_ = false;
    double heading_ = 0.0;
    int label_ = -1;
    double tolerance_ = 0.1;
    double timeout_ = 20.0;
    int settled_ = 0;
};

// Swim forward a distance, holding the heading captured at start.
class SwimForward : public ContextAction
{
public:
    using ContextAction::ContextAction;
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {}

private:
    BT::NodeStatus step();
    bool timed_ = true;
    double distance_ = 0.0;
    double speed_ = 0.0;
    double timeout_ = 0.0;
    double hold_yaw_ = 0.0;
    double start_x_ = 0.0, start_y_ = 0.0;
};

// Sweep left and right around the starting heading until a label is seen.
class SearchPattern : public ContextAction
{
public:
    using ContextAction::ContextAction;
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {}

private:
    BT::NodeStatus step();
    int label_ = -1;
    double half_sweep_ = 0.0;
    double timeout_ = 30.0;
    double start_yaw_ = 0.0;
    int leg_ = 0;
};

// Condition: a sighting of `label` younger than detection_max_age_s.
class IsDetected : public BT::ConditionNode
{
public:
    IsDetected(const std::string & name, const BT::NodeConfig & config, ContextPtr ctx)
    : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    ContextPtr ctx_;
};

// Placeholders for the deferred depth leaves (D-A2). Registered with the ports
// fish_tree.xml uses, so that tree loads and A-3 covers it, but they contain
// no depth logic and always FAIL.
//
// TODO(fish): HoldDepth is blocked by D-3 (fin count and placement), and a
// depth change at zero surge is C-8's confirmed defect. Implement it there.
class HoldDepth : public BT::SyncActionNode
{
public:
    HoldDepth(const std::string & name, const BT::NodeConfig & config, ContextPtr ctx)
    : BT::SyncActionNode(name, config), ctx_(std::move(ctx)) {}
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    ContextPtr ctx_;
};

// TODO(fish): Surface needs real depth (F-4) and a depth loop (D-3 / C-8).
class Surface : public BT::SyncActionNode
{
public:
    Surface(const std::string & name, const BT::NodeConfig & config, ContextPtr ctx)
    : BT::SyncActionNode(name, config), ctx_(std::move(ctx)) {}
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    ContextPtr ctx_;
};

}  // namespace bur_autonomy

#endif
