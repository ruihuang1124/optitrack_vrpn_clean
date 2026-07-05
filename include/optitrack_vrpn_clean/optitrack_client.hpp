#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <vrpn_Connection.h>

#include <optitrack_vrpn_clean/config.hpp>
#include <optitrack_vrpn_clean/output_dispatcher.hpp>
#include <optitrack_vrpn_clean/time_manager.hpp>
#include <optitrack_vrpn_clean/tracker_handler.hpp>

namespace optitrack_vrpn_clean
{

class OptitrackClient
{
public:
  explicit OptitrackClient(Config config);

  int run(const std::atomic_bool* stop_requested = nullptr);
  void stop();

private:
  Config config_;
  std::set<std::string> blacklist_;
  std::shared_ptr<vrpn_Connection> connection_;
  std::map<std::string, std::unique_ptr<TrackerHandler>> trackers_;
  TimeManager time_manager_;
  bool stop_requested_ = false;

  bool connect(const std::atomic_bool* external_stop);
  bool shouldStop(const std::atomic_bool* external_stop) const;
  void pumpVrpn();
  void scanTrackers(OutputDispatcher& output, bool print_only);
  bool acceptsTracker(const std::string& name) const;
};

}  // namespace optitrack_vrpn_clean
