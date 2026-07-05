#pragma once

#include <cstddef>
#include <sys/time.h>

namespace optitrack_vrpn_clean
{

double wallTimeSeconds();

class TimeManager
{
public:
  double resolveTimestamp(const timeval& stamp);
  void setNumSamples(std::size_t num_samples);
  std::size_t sampleCount() const { return count_; }
  bool offsetResolved() const { return num_samples_ > 0 && count_ >= num_samples_; }

  static double timevalToSeconds(const timeval& stamp);

private:
  std::size_t num_samples_ = 100;
  double min_offset_s_ = 0.0;
  std::size_t count_ = 0;
  double offset_s_ = 0.0;
};

}  // namespace optitrack_vrpn_clean
