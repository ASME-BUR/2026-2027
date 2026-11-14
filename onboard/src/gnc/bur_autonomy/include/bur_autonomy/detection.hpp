#ifndef BUR_AUTONOMY_DETECTION_HPP
#define BUR_AUTONOMY_DETECTION_HPP

// The ONE place that turns a CVDetection into a bearing.
//
// TODO(decide): P-4 owns the detection contract, and today there is none
// (PLAN.md D-A4). What exists, read 2026-11-07:
//   - bur-vision/yolo.py publishes yolo_msgs/CVDetections (NOT bur_msgs) on
//     "yolo_detections" at ~2 Hz, puts the PIXEL centre of the box in
//     bbox.pose.position.x/y, the box size in bbox.pose.orientation.x/y, and
//     leaves header, width, height and depth unset.
//   - bur-vision-nano publishes nothing to ROS.
// This file assumes the yolo.py layout: bbox.pose.position.x is the pixel x of
// the box centre in an image `image_width_px` wide. When the Jetson publishes
// a 3D position instead, change this function and nothing else.

#include <cmath>

namespace bur_autonomy
{

struct CameraModel
{
    double image_width_px = 1280.0;   // TODO(hw): ZED at the chosen resolution
    double hfov_rad = 1.92;           // TODO(hw): ~110 deg, ZED 2i; check the actual lens
};

// Focal length in pixels from the horizontal field of view.
inline double focalPx(const CameraModel & c)
{
    return (c.image_width_px / 2.0) / std::tan(c.hfov_rad / 2.0);
}

// Bearing (rad) of a box centred at pixel x. A target LEFT of image centre is
// to PORT, which is positive yaw in NWU, hence the minus sign.
inline double detectionBearing(double pixel_x, const CameraModel & c)
{
    return -std::atan((pixel_x - c.image_width_px / 2.0) / focalPx(c));
}

// Inverse of detectionBearing, for the simulators (bench_sim_node, tests).
inline double pixelForBearing(double bearing_rad, const CameraModel & c)
{
    return c.image_width_px / 2.0 - std::tan(bearing_rad) * focalPx(c);
}

inline bool inFieldOfView(double bearing_rad, const CameraModel & c)
{
    return std::fabs(bearing_rad) < c.hfov_rad / 2.0;
}

}  // namespace bur_autonomy

#endif
