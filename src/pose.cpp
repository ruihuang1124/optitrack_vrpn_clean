#include <optitrack_vrpn_clean/pose.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>

namespace optitrack_vrpn_clean
{
namespace
{

constexpr double kVelocityMinDt = 1.0e-6;

}  // namespace

Quat normalize(const Quat& q)
{
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm <= 0.0)
  {
    return {};
  }
  return {q.x / norm, q.y / norm, q.z / norm, q.w / norm};
}

Quat multiply(const Quat& lhs, const Quat& rhs)
{
  return {
    lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
    lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
    lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
    lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
  };
}

Quat yawQuaternion(double yaw_rad)
{
  const double half_yaw = 0.5 * yaw_rad;
  return {0.0, 0.0, std::sin(half_yaw), std::cos(half_yaw)};
}

std::array<double, 3> quaternionToRpy(const Quat& q_in)
{
  const Quat q = normalize(q_in);

  const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  double pitch = 0.0;
  if (std::abs(sinp) >= 1.0)
  {
    pitch = std::copysign(M_PI / 2.0, sinp);
  }
  else
  {
    pitch = std::asin(sinp);
  }

  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  return {roll, pitch, yaw};
}

Pose convertOptitrackToEnu(const Pose& optitrack_pose)
{
  Pose enu_pose;

  enu_pose.position.x = optitrack_pose.position.z;
  enu_pose.position.y = optitrack_pose.position.x;
  enu_pose.position.z = optitrack_pose.position.y;

  const Quat enu_to_rfu = {
    optitrack_pose.orientation.z,
    optitrack_pose.orientation.x,
    optitrack_pose.orientation.y,
    optitrack_pose.orientation.w,
  };
  const Quat rfu_to_flu = yawQuaternion(M_PI_2);
  enu_pose.orientation = normalize(multiply(enu_to_rfu, rfu_to_flu));

  return enu_pose;
}

Pose convertOptitrackToNed(const Pose& optitrack_pose)
{
  Pose ned_pose;

  ned_pose.position.x = optitrack_pose.position.x;
  ned_pose.position.y = optitrack_pose.position.z;
  ned_pose.position.z = -optitrack_pose.position.y;

  ned_pose.orientation.x = optitrack_pose.orientation.x;
  ned_pose.orientation.y = optitrack_pose.orientation.z;
  ned_pose.orientation.z = -optitrack_pose.orientation.y;
  ned_pose.orientation.w = optitrack_pose.orientation.w;
  ned_pose.orientation = normalize(ned_pose.orientation);

  return ned_pose;
}

TrackerSample makeTrackerSample(const std::string& name,
                                double timestamp_s,
                                double source_timestamp_s,
                                const Pose& optitrack_pose,
                                const TrackerSample* previous_sample)
{
  TrackerSample sample;
  sample.name = name;
  sample.timestamp_s = timestamp_s;
  sample.source_timestamp_s = source_timestamp_s;
  sample.optitrack = optitrack_pose;
  sample.enu = convertOptitrackToEnu(optitrack_pose);
  sample.ned = convertOptitrackToNed(optitrack_pose);

  if (previous_sample != nullptr)
  {
    const double dt = sample.timestamp_s - previous_sample->timestamp_s;
    if (dt > kVelocityMinDt)
    {
      sample.enu_linear_velocity.x =
        (sample.enu.position.x - previous_sample->enu.position.x) / dt;
      sample.enu_linear_velocity.y =
        (sample.enu.position.y - previous_sample->enu.position.y) / dt;
      sample.enu_linear_velocity.z =
        (sample.enu.position.z - previous_sample->enu.position.z) / dt;
      sample.velocity_valid = true;
    }
  }

  return sample;
}

std::string sanitizeTrackerName(const std::string& name)
{
  std::stringstream stream;
  bool first_character = true;

  for (const char c : name)
  {
    if (std::isalnum(static_cast<unsigned char>(c)) || c == '/')
    {
      stream << c;
    }
    else if (c == '_' && !first_character)
    {
      stream << c;
    }
    else if (c == ' ')
    {
      stream << '_';
    }
    first_character = false;
  }

  return stream.str();
}

}  // namespace optitrack_vrpn_clean
