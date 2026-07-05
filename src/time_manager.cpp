#include <optitrack_vrpn_clean/time_manager.hpp>

#include <chrono>

namespace optitrack_vrpn_clean
{

double wallTimeSeconds()
{
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

double TimeManager::resolveTimestamp(const timeval& stamp)
{
  const double arrival_time_s = wallTimeSeconds();
  const double source_time_s = timevalToSeconds(stamp);

  if (num_samples_ == 0)
  {
    return arrival_time_s;
  }

  if (count_ >= num_samples_)
  {
    return source_time_s + offset_s_;
  }

  const double offset_s = arrival_time_s - source_time_s;
  if (count_ == 0 || offset_s < min_offset_s_)
  {
    min_offset_s_ = offset_s;
  }

  ++count_;
  if (count_ == num_samples_)
  {
    offset_s_ = min_offset_s_;
  }

  return arrival_time_s;
}

void TimeManager::setNumSamples(std::size_t num_samples)
{
  num_samples_ = num_samples;
  min_offset_s_ = 0.0;
  count_ = 0;
  offset_s_ = 0.0;
}

double TimeManager::timevalToSeconds(const timeval& stamp)
{
  return static_cast<double>(stamp.tv_sec) + static_cast<double>(stamp.tv_usec) * 1.0e-6;
}

}  // namespace optitrack_vrpn_clean
