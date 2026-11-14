// Mission-level tests: the real fish_tree_horizontal.xml, built exactly as the
// manager builds it (buildMissionTree), closed-loop against a kinematic
// unicycle and a fake detector that behaves like yolo.py (2 Hz, latency,
// pixel-x through the inverse of detectionBearing, gated by the FOV).
//
// The sim's numbers are invented: these tests prove the tree logic, not
// vehicle performance.

#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "bur_autonomy/detection.hpp"
#include "bur_autonomy/register.hpp"
#include "bur_autonomy/sim/unicycle.hpp"

using bur_autonomy::CameraModel;
using bur_autonomy::Detection;
using bur_autonomy::MissionContext;

namespace
{

constexpr double kTick = 0.1;
const std::string kTree = std::string(BUR_TREE_DIR) + "/fish_tree_horizontal.xml";

// The test's own fixed camera, NOT the placeholder parameter values (those await hardware).
CameraModel testCamera()
{
    CameraModel c;
    c.image_width_px = 1280.0;
    c.hfov_rad = 1.2;   // ~69 deg
    return c;
}

struct Mission
{
    std::shared_ptr<MissionContext> ctx = std::make_shared<MissionContext>();
    bur_autonomy::MissionTree m;
    bur_autonomy::sim::Unicycle body;
    CameraModel cam = testCamera();

    double tx = 0.0, ty = 0.0;     // target position, odom frame
    int label = 0;
    double frame_period = 0.5;     // ~yolo.py
    double latency = 0.3;
    double next_frame = 0.0;
    bool camera_on = true;
    struct Pending { double deliver_at; bool seen; double pixel_x; };
    std::deque<Pending> in_flight;

    double t = 0.0;
    std::vector<std::string> leaf_starts;
    std::vector<double> leaf_start_times;
    int max_violations = 0;

    explicit Mission(const std::string & mode = "odometry")
    {
        ctx->cfg.swim_forward_mode = mode;
        ctx->on_leaf_start = [this](const std::string & n) {
            leaf_starts.push_back(n);
            leaf_start_times.push_back(t);
        };
        m = bur_autonomy::buildMissionTree(kTree, ctx);
    }

    void placeTarget(double range, double bearing)
    {
        tx = body.x + range * std::cos(body.yaw + bearing);
        ty = body.y + range * std::sin(body.yaw + bearing);
    }

    // Camera: capture at frame times, deliver after `latency`. Delivery is
    // what the manager sees, and it records yaw AT RECEIPT, as the node does.
    void camera(double now)
    {
        if (camera_on && now >= next_frame) {
            next_frame += frame_period;
            const double b = body.bearingTo(tx, ty);
            in_flight.push_back({now + latency, bur_autonomy::inFieldOfView(b, cam),
                                 bur_autonomy::pixelForBearing(b, cam)});
        }
        while (!in_flight.empty() && in_flight.front().deliver_at <= now) {
            const Pending p = in_flight.front();
            in_flight.pop_front();
            if (!p.seen) { continue; }
            Detection d;
            d.label = label;
            d.conf = 0.9;
            d.bearing_rad = bur_autonomy::detectionBearing(p.pixel_x, cam);
            d.stamp_s = now;
            d.yaw_at_receipt = body.yaw;
            ctx->recordSighting(d);
        }
    }

    // Let the camera run before the mission starts, so the target (if
    // visible) is already known on the first tick.
    void warmUp(double seconds = 1.0)
    {
        next_frame = -seconds;
        for (double w = -seconds; w < 0.0; w += kTick) {
            t = w;
            camera(w);
        }
        t = 0.0;
    }

    void syncPose()
    {
        ctx->x = body.x;
        ctx->y = body.y;
        ctx->yaw = body.yaw;
        ctx->have_odom = true;
    }

    // One manager tick plus one sim step. `paused` = odometry stale: the
    // manager does not tick and commands coast.
    BT::NodeStatus step(bool paused = false, double resume_dt = kTick)
    {
        camera(t);
        syncPose();
        BT::NodeStatus st = BT::NodeStatus::RUNNING;
        if (paused) {
            body.step(0.0, 0.0, kTick);
        } else {
            ctx->now_s = t;
            ctx->tick_dt_s = resume_dt;
            ctx->clearOutputs();
            st = m.tree.tickOnce();
            // Invariants of the outputs, on every tick.
            if (!(ctx->surge_mps >= 0.0) || !std::isfinite(ctx->yaw_rate_rps) ||
                std::fabs(ctx->yaw_rate_rps) > ctx->cfg.max_yaw_rate_rps + 1e-9) {
                ++max_violations;
            }
            if (ctx->enable) {
                body.step(ctx->surge_mps, ctx->yaw_rate_rps, kTick);
            } else {
                body.step(0.0, 0.0, kTick);
            }
        }
        t += kTick;
        return st;
    }

    BT::NodeStatus runUntilDone(double limit_s)
    {
        BT::NodeStatus st = BT::NodeStatus::RUNNING;
        while (t < limit_s) {
            st = step();
            if (st == BT::NodeStatus::SUCCESS || st == BT::NodeStatus::FAILURE) { break; }
        }
        return st;
    }

    int count(const std::string & leaf) const
    {
        int n = 0;
        for (const auto & s : leaf_starts) { n += (s == leaf); }
        return n;
    }
};

}  // namespace

// 1. Target ahead and visible: face it, swim 4 m toward it.
TEST(MissionSim, TargetAheadSucceeds)
{
    Mission s;
    s.placeTarget(10.0, 0.3);
    s.warmUp();
    const double r0 = s.body.rangeTo(s.tx, s.ty);
    EXPECT_EQ(BT::NodeStatus::SUCCESS, s.runUntilDone(120.0));
    EXPECT_EQ(0, s.count("SearchPattern"));   // seen from the start
    EXPECT_LT(s.body.rangeTo(s.tx, s.ty), r0 - 3.0);   // actually closed on it
    EXPECT_EQ(0, s.max_violations);
}

// 2a. Outside the initial FOV but well inside sweep/2 + HFOV/2: search finds it.
TEST(MissionSim, TargetInsideSearchCoverageSucceeds)
{
    Mission s;
    // coverage = 1.57/2 + 1.2/2 = 1.385 rad; initial FOV edge = 0.6 rad.
    s.placeTarget(8.0, 1.0);
    s.warmUp();
    EXPECT_EQ(BT::NodeStatus::SUCCESS, s.runUntilDone(150.0));
    EXPECT_GE(s.count("SearchPattern"), 1);
    EXPECT_EQ(0, s.max_violations);
}

// 2b. Directly astern and far: never seen, 3 attempts, FAILURE.
TEST(MissionSim, TargetOutsideCoverageFailsAfterThreeAttempts)
{
    Mission s;
    s.placeTarget(40.0, bur_autonomy::kPi);
    s.warmUp();
    EXPECT_EQ(BT::NodeStatus::FAILURE, s.runUntilDone(200.0));
    EXPECT_EQ(3, s.count("SearchPattern"));
    EXPECT_EQ(0, s.count("TurnToHeading"));
    EXPECT_EQ(0, s.max_violations);
}

// 4. Seen, then lost during the turn: the retry must SEARCH again, not resume
//    at the failed TurnToHeading (the SequenceWithMemory trap).
TEST(MissionSim, LostDuringTurnSearchesAgain)
{
    Mission s;
    // A slow turn (0.55 rad at <= 0.15 rad/s takes > 3.5 s) so the target is
    // declared lost (detection_lost_s = 2 s) before the turn can settle.
    s.ctx->cfg.max_yaw_rate_rps = 0.15;
    s.placeTarget(8.0, 0.55);   // just inside the FOV (half-angle 0.6)
    s.warmUp();
    double blackout_until = -1.0;
    BT::NodeStatus st = BT::NodeStatus::RUNNING;
    while (s.t < 150.0) {
        if (blackout_until < 0.0 && s.count("TurnToHeading") == 1) {
            blackout_until = s.t + 5.0;   // occluded for 5 s from the first turn
        }
        s.camera_on = !(blackout_until > 0.0 && s.t < blackout_until);
        if (!s.camera_on) { s.in_flight.clear(); }
        st = s.step();
        if (st == BT::NodeStatus::SUCCESS || st == BT::NodeStatus::FAILURE) { break; }
    }
    EXPECT_EQ(BT::NodeStatus::SUCCESS, st);
    // Order: TurnToHeading (fails: lost) -> SearchPattern -> TurnToHeading -> SwimForward.
    ASSERT_GE(s.leaf_starts.size(), 4u);
    EXPECT_EQ("TurnToHeading", s.leaf_starts[0]);
    EXPECT_EQ("SearchPattern", s.leaf_starts[1]);
    EXPECT_EQ("TurnToHeading", s.leaf_starts[2]);
    EXPECT_EQ("SwimForward", s.leaf_starts.back());
    EXPECT_EQ(0, s.max_violations);
}

// 5. Odometry pause mid-SwimForward (timed mode): the pause is not credited.
TEST(MissionSim, PauseIsNotCountedAsSwimming)
{
    Mission s("timed");
    s.placeTarget(10.0, 0.0);
    s.warmUp();
    while (s.t < 60.0 && s.count("SwimForward") == 0) { s.step(); }
    ASSERT_EQ(1, s.count("SwimForward"));
    const double swim_start = s.leaf_start_times.back();
    for (int i = 0; i < 20; ++i) { s.step(); }                 // 2 s swimming
    for (int i = 0; i < 50; ++i) { s.step(/*paused=*/true); }  // 5 s paused
    // First tick after the pause: the manager clamps dt to 2 periods.
    BT::NodeStatus st = s.step(false, 2 * kTick);
    while (st == BT::NodeStatus::RUNNING && s.t < 120.0) { st = s.step(); }
    EXPECT_EQ(BT::NodeStatus::SUCCESS, st);
    // 4 m at 0.3 m/s = 13.3 s of swimming, plus the 5 s pause on the clock.
    EXPECT_GE(s.t - swim_start, 13.3 + 5.0 - 0.5);
}
