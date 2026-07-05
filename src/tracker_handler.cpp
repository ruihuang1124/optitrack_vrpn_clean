#include <optitrack_vrpn_clean/tracker_handler.hpp>

#include <utility>

namespace optitrack_vrpn_clean
{

TrackerHandler::TrackerHandler(std::string name,
                               std::string host,
                               std::shared_ptr<vrpn_Connection> connection,
                               TimeManager& time_manager,
                               OutputDispatcher& output)
  : name_(std::move(name)),
    host_(std::move(host)),
    vrpn_address_(name_ + "@" + host_),
    connection_(std::move(connection)),
    time_manager_(time_manager),
    output_(output),
    tracker_(vrpn_address_.c_str(), connection_.get())
{
  tracker_.register_change_handler(this, &TrackerHandler::positionCallbackWrapper);
}

void TrackerHandler::mainloop()
{
  tracker_.mainloop();
}

void TrackerHandler::positionCallbackWrapper(void* user_data, vrpn_TRACKERCB info)
{
  auto* handler = reinterpret_cast<TrackerHandler*>(user_data);
  handler->positionCallback(info);
}

void TrackerHandler::positionCallback(const vrpn_TRACKERCB& info)
{
  Pose optitrack_pose;
  optitrack_pose.position.x = info.pos[Index::X];
  optitrack_pose.position.y = info.pos[Index::Y];
  optitrack_pose.position.z = info.pos[Index::Z];
  optitrack_pose.orientation.x = info.quat[Index::X];
  optitrack_pose.orientation.y = info.quat[Index::Y];
  optitrack_pose.orientation.z = info.quat[Index::Z];
  optitrack_pose.orientation.w = info.quat[Index::W];

  TrackerSample sample = makeTrackerSample(name_,
                                           time_manager_.resolveTimestamp(info.msg_time),
                                           TimeManager::timevalToSeconds(info.msg_time),
                                           optitrack_pose,
                                           has_previous_sample_ ? &previous_sample_ : nullptr);
  sample.sequence = ++sequence_;

  output_.handleSample(sample);
  previous_sample_ = sample;
  has_previous_sample_ = true;
}

}  // namespace optitrack_vrpn_clean
