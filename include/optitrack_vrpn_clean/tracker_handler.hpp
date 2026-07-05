#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>

#include <optitrack_vrpn_clean/output_dispatcher.hpp>
#include <optitrack_vrpn_clean/pose.hpp>
#include <optitrack_vrpn_clean/time_manager.hpp>

namespace optitrack_vrpn_clean
{

class TrackerHandler
{
public:
  TrackerHandler(std::string name,
                 std::string host,
                 std::shared_ptr<vrpn_Connection> connection,
                 TimeManager& time_manager,
                 OutputDispatcher& output);

  void mainloop();
  bool hasReceivedSample() const { return has_previous_sample_; }

  static void VRPN_CALLBACK positionCallbackWrapper(void* user_data, vrpn_TRACKERCB info);

private:
  enum Index
  {
    X = 0,
    Y = 1,
    Z = 2,
    W = 3
  };

  std::string name_;
  std::string host_;
  std::string vrpn_address_;
  std::shared_ptr<vrpn_Connection> connection_;
  TimeManager& time_manager_;
  OutputDispatcher& output_;
  vrpn_Tracker_Remote tracker_;
  std::uint64_t sequence_ = 0;
  bool has_previous_sample_ = false;
  TrackerSample previous_sample_;

  void positionCallback(const vrpn_TRACKERCB& info);
};

}  // namespace optitrack_vrpn_clean
