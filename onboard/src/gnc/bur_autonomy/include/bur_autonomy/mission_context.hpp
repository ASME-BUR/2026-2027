#ifndef BUR_AUTONOMY_MISSION_CONTEXT_HPP
#define BUR_AUTONOMY_MISSION_CONTEXT_HPP

// State shared between the manager node and the behaviour-tree leaves.
//
// ROS-free on purpose: the manager copies ROS messages in, the leaves read and
// write plain fields, and the manager copies the outputs back into a
// SwimCommand. That keeps every leaf, every tree load and a whole simulated
// mission testable with a fake clock (PLAN.md section 1).

#include <functional>
#include <string>
#include <vector>

namespace bur_autonomy
{

// One remembered sighting per label (the latest). Bearing is relative to the
// body at the moment of receipt, so the absolute direction to the target is
// yaw_at_receipt + bearing_rad. Using the yaw at receipt, not the current
// yaw, matters because detections arrive ~2 Hz plus inference latency.
struct Detection
{
    int label = -1;
    double conf = 0.0;
    double bearing_rad = 0.0;   // +ve = target to port (NWU)
    double stamp_s = 0.0;       // receipt time, ctx clock
    double yaw_at_receipt = 0.0;
};

// Tuning shared by the leaves. Filled from the manager's parameter table;
// the XML carries only per-use values (targets, tolerances, timeouts).
struct LeafConfig
{
    double heading_kp = 1.0;           // rad/s per rad of heading error
    double max_yaw_rate_rps = 0.4;     // TODO(hw): from M-3's turn-rate table
    // Forward speed while turning. Must give a tail-beat frequency above the
    // controller's freq_min, or the controller drops to MODE_HOLD (a static
    // bend with no thrust) and the vehicle does not turn. With surge_to_freq
    // 2.0 and freq_min 0.0 (controller.yaml), 0.15 m/s -> 0.3 Hz.
    double turn_surge_mps = 0.15;      // TODO(hw): M-3
    int settle_ticks = 3;              // consecutive ticks inside tolerance
    double search_leg_tolerance_rad = 0.1;
    // Detection timing. yolo.py publishes at ~2 Hz plus inference time, so an
    // age limit at or below 0.5 s would make a healthy detector look lost.
    // INVARIANT: detection_lost_s > detection_max_age_s. Then, when
    // TurnToHeading fails as "lost", IsDetected is guaranteed false on the
    // retry and the search runs again. The manager enforces it at startup.
    double detection_max_age_s = 1.5;
    double detection_lost_s = 2.0;
    double min_conf = 0.5;
    // "timed" (distance / speed, open loop) or "odometry". Timed is the
    // default until P-3: odometry x/y from an IMU-only EKF drifts.
    std::string swim_forward_mode = "timed";
};

struct MissionContext
{
    // ---- inputs (manager writes) ----
    double now_s = 0.0;
    // Time since the previous tick, clamped by the manager to 2x the tick
    // period. Leaves accumulate THIS, not wall time, so time spent paused
    // (not ticked) is never counted as swimming.
    double tick_dt_s = 0.0;
    bool have_odom = false;
    double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;   // odom frame; yaw relative to power-on
    std::vector<Detection> detections;            // latest sighting per label

    // ---- outputs (leaves write; manager zeroes them before every tick) ----
    double surge_mps = 0.0;      // never negative: the controller takes abs()
    double yaw_rate_rps = 0.0;
    bool enable = false;
    // depth_m / pitch_rad are not here: depth is deferred (D-3), so the
    // manager publishes 0 for both. The SwimCommand fields and the
    // controller's depth paths are kept, unused.

    LeafConfig cfg;

    // ---- hooks ----
    std::function<void(const std::string &)> log;               // may be empty
    std::function<void(const std::string &)> on_leaf_start;     // may be empty
    std::string active_leaf;   // last leaf that ticked; for ~/status

    void clearOutputs()
    {
        surge_mps = 0.0;
        yaw_rate_rps = 0.0;
        enable = false;
        active_leaf.clear();
    }

    // Freshest sighting of `label` with conf >= min_conf, or nullptr.
    const Detection * sighting(int label, double min_conf) const
    {
        const Detection * best = nullptr;
        for (const auto & d : detections) {
            if (d.label == label && d.conf >= min_conf &&
                (best == nullptr || d.stamp_s > best->stamp_s)) {
                best = &d;
            }
        }
        return best;
    }

    // Record a sighting, replacing the previous one for the same label.
    void recordSighting(const Detection & d)
    {
        for (auto & e : detections) {
            if (e.label == d.label) { e = d; return; }
        }
        detections.push_back(d);
    }

    void info(const std::string & msg) const
    {
        if (log) { log(msg); }
    }
};

}  // namespace bur_autonomy

#endif
