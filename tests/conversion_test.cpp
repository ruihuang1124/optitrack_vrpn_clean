#include <optitrack_vrpn_clean/pose.hpp>
#include <optitrack_vrpn_clean/time_manager.hpp>

#include <cassert>
#include <cmath>
#include <iostream>

namespace
{

bool near(double a, double b, double eps = 1.0e-9)
{
  return std::abs(a - b) < eps;
}

double quatNorm(const optitrack_vrpn_clean::Quat& q)
{
  return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

}  // namespace

int main()
{
  using namespace optitrack_vrpn_clean;

  Pose optitrack;
  optitrack.position.x = 1.0;
  optitrack.position.y = 2.0;
  optitrack.position.z = 3.0;
  optitrack.orientation.w = 1.0;

  const Pose enu = convertOptitrackToEnu(optitrack);
  assert(near(enu.position.x, 3.0));
  assert(near(enu.position.y, 1.0));
  assert(near(enu.position.z, 2.0));
  assert(near(quatNorm(enu.orientation), 1.0));
  assert(near(enu.orientation.z, std::sqrt(0.5)));
  assert(near(enu.orientation.w, std::sqrt(0.5)));

  const Pose ned = convertOptitrackToNed(optitrack);
  assert(near(ned.position.x, 1.0));
  assert(near(ned.position.y, 3.0));
  assert(near(ned.position.z, -2.0));
  assert(near(ned.orientation.w, 1.0));

  TrackerSample first = makeTrackerSample("body", 10.0, 1.0, optitrack, nullptr);
  first.sequence = 1;

  optitrack.position.z = 4.0;
  const TrackerSample second = makeTrackerSample("body", 10.5, 1.5, optitrack, &first);
  assert(second.velocity_valid);
  assert(near(second.enu_linear_velocity.x, 2.0));
  assert(near(second.enu_linear_velocity.y, 0.0));
  assert(near(second.enu_linear_velocity.z, 0.0));

  timeval tv{};
  tv.tv_sec = 12;
  tv.tv_usec = 345678;
  assert(near(TimeManager::timevalToSeconds(tv), 12.345678));

  std::cout << "conversion_test passed\n";
  return 0;
}
