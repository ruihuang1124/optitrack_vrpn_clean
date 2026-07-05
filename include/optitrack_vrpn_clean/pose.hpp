#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace optitrack_vrpn_clean
{

struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quat
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

struct Pose
{
  Vec3 position;
  Quat orientation;
};

struct TrackerSample
{
  std::string name;
  std::uint64_t sequence = 0;
  double timestamp_s = 0.0;
  double source_timestamp_s = 0.0;
  Pose optitrack;
  Pose enu;
  Pose ned;
  Vec3 enu_linear_velocity;
  bool velocity_valid = false;
};

Quat normalize(const Quat& q);
Quat multiply(const Quat& lhs, const Quat& rhs);
Quat yawQuaternion(double yaw_rad);
std::array<double, 3> quaternionToRpy(const Quat& q);

Pose convertOptitrackToEnu(const Pose& optitrack_pose);
Pose convertOptitrackToNed(const Pose& optitrack_pose);

TrackerSample makeTrackerSample(const std::string& name,
                                double timestamp_s,
                                double source_timestamp_s,
                                const Pose& optitrack_pose,
                                const TrackerSample* previous_sample);

std::string sanitizeTrackerName(const std::string& name);

}  // namespace optitrack_vrpn_clean
