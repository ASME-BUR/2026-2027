// Leaf-level tests (A-2): each leaf against a scripted MissionContext and a
// fake clock. No ROS.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "bur_autonomy/register.hpp"

using bur_autonomy::Detection;
using bur_autonomy::MissionContext;
using bur_autonomy::MissionTree;

namespace
{

constexpr double kDt = 0.1;

std::string wrap(const std::string & leaf_xml)
{
    return "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\">" + leaf_xml +
           "</BehaviorTree></root>";
}

struct Harness
{
    std::shared_ptr<MissionContext> ctx = std::make_shared<MissionContext>();
    MissionTree m;

    explicit Harness(const std::string & leaf_xml, bool build = true)
    {
        if (build) { m = bur_autonomy::buildMissionTreeFromText(wrap(leaf_xml), ctx); }
    }

    // One manager-style tick: advance the clock, clear outputs, tick.
    BT::NodeStatus tick(double dt = kDt)
    {
        ctx->now_s += dt;
        ctx->tick_dt_s = dt;
        ctx->clearOutputs();
        return m.tree.tickOnce();
    }

    void see(int label, double bearing, double conf = 0.9)
    {
        Detection d;
        d.label = label;
        d.conf = conf;
        d.bearing_rad = bearing;
        d.stamp_s = ctx->now_s;
        d.yaw_at_receipt = ctx->yaw;
        ctx->recordSighting(d);
    }
};

}  // namespace

// ---------------------------------------------------------------- TurnToHeading

TEST(TurnToHeading, TurnsTowardAbsoluteHeadingAndSettles)
{
    Harness h(R"(<TurnToHeading source="absolute" heading_rad="1.0" tolerance_rad="0.1"/>)");
    EXPECT_EQ(BT::NodeStatus::RUNNING, h.tick());
    EXPECT_GT(h.ctx->yaw_rate_rps, 0.0);                       // to port
    EXPECT_DOUBLE_EQ(h.ctx->cfg.turn_surge_mps, h.ctx->surge_mps);   // must not stall in HOLD
    EXPECT_TRUE(h.ctx->enable);

    h.ctx->yaw = 1.02;   // inside tolerance
    EXPECT_EQ(BT::NodeStatus::RUNNING, h.tick());   // settle 1
    EXPECT_EQ(BT::NodeStatus::RUNNING, h.tick());   // settle 2
    EXPECT_EQ(BT::NodeStatus::SUCCESS, h.tick());   // settle 3
}

TEST(TurnToHeading, TakesTheShortWayAcrossPi)
{
    Harness h(R"(<TurnToHeading source="absolute" heading_rad="3.0"/>)");
    h.ctx->yaw = -3.0;   // 0.28 rad clockwise, not 6 rad anticlockwise
    h.tick();
    EXPECT_LT(h.ctx->yaw_rate_rps, 0.0);
    EXPECT_LE(std::fabs(h.ctx->yaw_rate_rps), h.ctx->cfg.max_yaw_rate_rps);
}

TEST(TurnToHeading, TimesOutOnActiveTime)
{
    Harness h(R"(<TurnToHeading source="absolute" heading_rad="1.0" timeout_s="2.0"/>)");
    BT::NodeStatus s = BT::NodeStatus::RUNNING;
    int ticks = 0;
    while (s == BT::NodeStatus::RUNNING && ticks < 100) { s = h.tick(); ++ticks; }
    EXPECT_EQ(BT::NodeStatus::FAILURE, s);
    EXPECT_NEAR(21, ticks, 1);
}

TEST(TurnToHeading, DetectionUsesYawAtReceipt)
{
    Harness h(R"(<TurnToHeading source="detection" label="0"/>)");
    h.ctx->yaw = 0.5;
    h.see(0, 0.2);        // target at 0.7 absolute
    h.ctx->yaw = 0.0;     // the vehicle has since turned back
    h.tick();
    // Error is 0.7 (yaw_at_receipt + bearing - yaw), not 0.2.
    EXPECT_NEAR(std::min(0.7 * h.ctx->cfg.heading_kp, h.ctx->cfg.max_yaw_rate_rps),
                h.ctx->yaw_rate_rps, 1e-9);
}

TEST(TurnToHeading, FailsWhenDetectionIsLost)
{
    Harness h(R"(<TurnToHeading source="detection" label="0"/>)");
    h.see(0, 0.5);
    BT::NodeStatus s = BT::NodeStatus::RUNNING;
    double t = 0.0;
    while (s == BT::NodeStatus::RUNNING && t < 10.0) { s = h.tick(); t += kDt; }
    EXPECT_EQ(BT::NodeStatus::FAILURE, s);
    EXPECT_NEAR(h.ctx->cfg.detection_lost_s, t, 0.15);
}

TEST(TurnToHeading, MissingHeadingForAbsoluteThrows)
{
    Harness h(R"(<TurnToHeading source="absolute"/>)");
    EXPECT_THROW(h.tick(), BT::RuntimeError);
}

TEST(TurnToHeading, MissingLabelForDetectionThrows)
{
    Harness h(R"(<TurnToHeading source="detection"/>)");
    EXPECT_THROW(h.tick(), BT::RuntimeError);
}

TEST(TurnToHeading, UnknownSourceThrows)
{
    Harness h(R"(<TurnToHeading source="compass" heading_rad="1.0"/>)");
    EXPECT_THROW(h.tick(), BT::RuntimeError);
}

TEST(TurnToHeading, HaltAndRestartResetsTheClock)
{
    Harness h(R"(<TurnToHeading source="absolute" heading_rad="1.0" timeout_s="1.0"/>)");
    for (int i = 0; i < 8; ++i) { EXPECT_EQ(BT::NodeStatus::RUNNING, h.tick()); }
    h.m.tree.haltTree();
    // A fresh start gets the full 1.0 s again.
    for (int i = 0; i < 8; ++i) { EXPECT_EQ(BT::NodeStatus::RUNNING, h.tick()); }
}

// ---------------------------------------------------------------- SwimForward

TEST(SwimForward, TimedModeCountsActiveTimeOnly)
{
    Harness h(R"(<SwimForward distance_m="1.0" speed_mps="0.5"/>)");   // 2 s
    h.ctx->cfg.swim_forward_mode = "timed";
    EXPECT_EQ(BT::NodeStatus::RUNNING, h.tick());
    EXPECT_DOUBLE_EQ(0.5, h.ctx->surge_mps);
    // A long pause: the manager does not tick, then resumes with a clamped dt.
    h.ctx->now_s += 30.0;
    int ticks = 0;
    BT::NodeStatus s = BT::NodeStatus::RUNNING;
    while (s == BT::NodeStatus::RUNNING && ticks < 100) { s = h.tick(); ++ticks; }
    EXPECT_EQ(BT::NodeStatus::SUCCESS, s);
    EXPECT_NEAR(20, ticks, 1);   // the 30 s pause was not credited
}

TEST(SwimForward, FallsBackToConfiguredModeWhenPortAbsent)
{
    Harness h(R"(<SwimForward distance_m="1.0" speed_mps="0.5" timeout_s="100"/>)");
    h.ctx->cfg.swim_forward_mode = "odometry";
    for (int i = 0; i < 50; ++i) { EXPECT_EQ(BT::NodeStatus::RUNNING, h.tick()); }   // no motion
    h.ctx->x = 0.8;
    h.ctx->y = 0.7;   // 1.06 m
    EXPECT_EQ(BT::NodeStatus::SUCCESS, h.tick());
}

TEST(SwimForward, XmlModeOverridesConfig)
{
    Harness h(R"(<SwimForward distance_m="0.1" speed_mps="1.0" mode="timed"/>)");
    h.ctx->cfg.swim_forward_mode = "odometry";
    h.tick();
    EXPECT_EQ(BT::NodeStatus::SUCCESS, h.tick());   // 0.1 s of active time, no motion needed
}

TEST(SwimForward, HoldsStartHeading)
{
    Harness h(R"(<SwimForward distance_m="5" speed_mps="0.3"/>)");
    h.ctx->yaw = 0.3;
    h.tick();
    h.ctx->yaw = 0.5;   // drifted to port
    h.tick();
    EXPECT_LT(h.ctx->yaw_rate_rps, 0.0);
}

TEST(SwimForward, BadModeThrows)
{
    Harness h(R"(<SwimForward distance_m="1" speed_mps="0.3" mode="odometery"/>)");
    EXPECT_THROW(h.tick(), BT::RuntimeError);
}

TEST(SwimForward, NonPositiveSpeedThrows)
{
    Harness h(R"(<SwimForward distance_m="1" speed_mps="-0.3"/>)");
    EXPECT_THROW(h.tick(), BT::RuntimeError);
}

TEST(SwimForward, MissingDistanceThrows)
{
    Harness h(R"(<SwimForward speed_mps="0.3"/>)");
    EXPECT_THROW(h.tick(), BT::RuntimeError);
}

// ---------------------------------------------------------------- SearchPattern

TEST(SearchPattern, SucceedsOnFreshDetection)
{
    Harness h(R"(<SearchPattern label="0" sweep_rad="1.57"/>)");
    EXPECT_EQ(BT::NodeStatus::RUNNING, h.tick());
    h.see(0, 0.1);
    EXPECT_EQ(BT::NodeStatus::SUCCESS, h.tick());
}

TEST(SearchPattern, AlternatesLegs)
{
    Harness h(R"(<SearchPattern label="0" sweep_rad="1.0"/>)");
    h.tick();
    EXPECT_GT(h.ctx->yaw_rate_rps, 0.0);   // first leg: +0.5
    h.ctx->yaw = 0.5;
    h.tick();
    EXPECT_LT(h.ctx->yaw_rate_rps, 0.0);   // second leg: -0.5
    h.ctx->yaw = -0.5;
    h.tick();
    EXPECT_GT(h.ctx->yaw_rate_rps, 0.0);   // back to +0.5
}

TEST(SearchPattern, IgnoresStaleOrWrongDetections)
{
    Harness h(R"(<SearchPattern label="0" sweep_rad="1.0" timeout_s="1.0"/>)");
    h.see(1, 0.0);          // wrong label
    h.see(0, 0.0, 0.1);     // too low confidence
    BT::NodeStatus s = BT::NodeStatus::RUNNING;
    int ticks = 0;
    while (s == BT::NodeStatus::RUNNING && ticks < 50) { s = h.tick(); ++ticks; }
    EXPECT_EQ(BT::NodeStatus::FAILURE, s);
}

// ---------------------------------------------------------------- IsDetected

TEST(IsDetected, FreshnessWindow)
{
    Harness h(R"(<IsDetected label="0"/>)");
    EXPECT_EQ(BT::NodeStatus::FAILURE, h.tick());
    h.see(0, 0.0);
    EXPECT_EQ(BT::NodeStatus::SUCCESS, h.tick());
    h.ctx->now_s += h.ctx->cfg.detection_max_age_s;
    EXPECT_EQ(BT::NodeStatus::FAILURE, h.tick());
}

TEST(IsDetected, MinConfPortOverridesConfig)
{
    Harness h(R"(<IsDetected label="0" min_conf="0.95"/>)");
    h.see(0, 0.0, 0.9);
    EXPECT_EQ(BT::NodeStatus::FAILURE, h.tick());
}

// ---------------------------------------------------------------- placeholders

TEST(DeferredDepth, PlaceholdersAlwaysFail)
{
    Harness a(R"(<HoldDepth depth_m="-1.0" tolerance_m="0.2" timeout_s="20.0"/>)");
    EXPECT_EQ(BT::NodeStatus::FAILURE, a.tick());
    EXPECT_FALSE(a.ctx->enable);
    Harness b(R"(<Surface/>)");
    EXPECT_EQ(BT::NodeStatus::FAILURE, b.tick());
}

// ---------------------------------------------------------------- invariants

TEST(Invariants, LeafConfigDefaultsKeepDetectionOrdering)
{
    bur_autonomy::LeafConfig c;
    EXPECT_GT(c.detection_lost_s, c.detection_max_age_s);
    EXPECT_GT(c.turn_surge_mps, 0.0);
}
