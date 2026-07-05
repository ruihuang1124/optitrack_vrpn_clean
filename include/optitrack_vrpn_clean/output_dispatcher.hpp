#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>

#include <netdb.h>

#include <optitrack_vrpn_clean/config.hpp>
#include <optitrack_vrpn_clean/pose.hpp>

namespace optitrack_vrpn_clean
{

class OutputDispatcher
{
public:
  explicit OutputDispatcher(const Config& config);
  ~OutputDispatcher();

  OutputDispatcher(const OutputDispatcher&) = delete;
  OutputDispatcher& operator=(const OutputDispatcher&) = delete;

  void handleSample(const TrackerSample& sample);

private:
  Config config_;
  std::ofstream csv_stream_;
  bool csv_header_written_ = false;
  std::unordered_map<std::string, double> last_print_wall_time_;

  int udp_socket_ = -1;
  sockaddr_storage udp_address_{};
  socklen_t udp_address_len_ = 0;
  std::unordered_map<std::string, double> last_yaml_wall_time_;

  void openCsv();
  void setupUdp();
  void setupYaml();

  void printSample(const TrackerSample& sample);
  void writeCsv(const TrackerSample& sample);
  void sendUdp(const TrackerSample& sample);
  void writeYaml(const TrackerSample& sample);
};

}  // namespace optitrack_vrpn_clean
