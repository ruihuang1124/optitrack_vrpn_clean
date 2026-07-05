#include <optitrack_vrpn_clean/optitrack_client.hpp>

#include <chrono>
#include <iostream>
#include <thread>
#include <utility>

namespace optitrack_vrpn_clean
{
namespace
{

std::shared_ptr<vrpn_Connection> makeConnection(const std::string& host)
{
  return std::shared_ptr<vrpn_Connection>(
    vrpn_get_connection_by_name(host.c_str()),
    [](vrpn_Connection* connection)
    {
      if (connection != nullptr)
      {
        connection->removeReference();
      }
    });
}

bool senderNameExists(vrpn_Connection* connection, int index)
{
  return connection != nullptr && connection->sender_name(index) != nullptr;
}

}  // namespace

OptitrackClient::OptitrackClient(Config config) : config_(std::move(config))
{
  validateConfig(config_);
  blacklist_.insert(config_.blacklist.begin(), config_.blacklist.end());
  time_manager_.setNumSamples(config_.offset_samples);
}

int OptitrackClient::run(const std::atomic_bool* stop_requested)
{
  OutputDispatcher output(config_);

  if (!connect(stop_requested))
  {
    return 1;
  }

  if (config_.list_only)
  {
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(config_.list_duration_s);
    while (!shouldStop(stop_requested) && std::chrono::steady_clock::now() < deadline)
    {
      connection_->mainloop();
      scanTrackers(output, true);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return 0;
  }

  const auto mainloop_period = std::chrono::duration<double>(1.0 / config_.update_rate_hz);
  const auto tracker_scan_period =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / config_.tracker_scan_rate_hz));

  auto next_mainloop = std::chrono::steady_clock::now();
  auto next_tracker_scan = next_mainloop;
  auto last_connection_warning = std::chrono::steady_clock::time_point{};
  auto last_no_sample_warning = std::chrono::steady_clock::now();

  while (!shouldStop(stop_requested))
  {
    const auto now = std::chrono::steady_clock::now();

    if (!connection_->doing_okay() && now - last_connection_warning > std::chrono::seconds(1))
    {
      std::cerr << "VRPN connection is not OK\n";
      last_connection_warning = now;
    }

    pumpVrpn();

    if (now >= next_tracker_scan)
    {
      scanTrackers(output, false);
      next_tracker_scan = now + tracker_scan_period;
    }

    if (!trackers_.empty() && now - last_no_sample_warning > std::chrono::seconds(3))
    {
      for (const auto& entry : trackers_)
      {
        if (entry.second && !entry.second->hasReceivedSample())
        {
          std::cerr << "No pose samples received yet for tracker \"" << entry.first
                    << "\". Check that the rigid body is actively tracked and that "
                    << "Motive VRPN streaming is sending tracker data.\n";
        }
      }
      last_no_sample_warning = now;
    }

    next_mainloop += std::chrono::duration_cast<std::chrono::steady_clock::duration>(mainloop_period);
    std::this_thread::sleep_until(next_mainloop);
    if (std::chrono::steady_clock::now() > next_mainloop + std::chrono::seconds(1))
    {
      next_mainloop = std::chrono::steady_clock::now();
    }
  }

  return 0;
}

void OptitrackClient::stop()
{
  stop_requested_ = true;
}

bool OptitrackClient::connect(const std::atomic_bool* external_stop)
{
  int attempt = 0;
  while (!shouldStop(external_stop))
  {
    ++attempt;
    connection_ = makeConnection(config_.host);

    if (connection_ && connection_->connected())
    {
      std::cout << "Connected to VRPN server at " << config_.host << '\n';
      return true;
    }

    const bool out_of_retries =
      config_.connect_retries >= 0 && attempt > config_.connect_retries;
    if (out_of_retries)
    {
      std::cerr << "Unable to connect to VRPN server at " << config_.host << '\n';
      return false;
    }

    std::cerr << "Not connected to VRPN server at " << config_.host
              << ". Retrying in " << config_.retry_delay_s << " seconds...\n";
    connection_.reset();
    std::this_thread::sleep_for(std::chrono::duration<double>(config_.retry_delay_s));
  }

  return false;
}

bool OptitrackClient::shouldStop(const std::atomic_bool* external_stop) const
{
  return stop_requested_ || (external_stop != nullptr && external_stop->load());
}

void OptitrackClient::pumpVrpn()
{
  if (trackers_.empty())
  {
    connection_->mainloop();
    return;
  }

  for (const auto& entry : trackers_)
  {
    if (entry.second)
    {
      entry.second->mainloop();
    }
  }
}

void OptitrackClient::scanTrackers(OutputDispatcher& output, bool print_only)
{
  if (!connection_)
  {
    return;
  }

  for (int i = 0; senderNameExists(connection_.get(), i); ++i)
  {
    const std::string name = connection_->sender_name(i);
    if (!acceptsTracker(name))
    {
      continue;
    }

    if (print_only)
    {
      if (trackers_.count(name) == 0)
      {
        std::cout << name << '\n';
        trackers_.emplace(name, nullptr);
      }
      continue;
    }

    if (trackers_.count(name) == 0)
    {
      trackers_.emplace(name,
                        std::make_unique<TrackerHandler>(name,
                                                         config_.host,
                                                         connection_,
                                                         time_manager_,
                                                         output));
      std::cout << "Added tracker for rigid body \"" << name << "\"\n";
    }
  }
}

bool OptitrackClient::acceptsTracker(const std::string& name) const
{
  if (name.empty() || blacklist_.count(name) > 0)
  {
    return false;
  }
  if (!config_.tracker_filter.empty() && name != config_.tracker_filter)
  {
    return false;
  }
  return true;
}

}  // namespace optitrack_vrpn_clean
