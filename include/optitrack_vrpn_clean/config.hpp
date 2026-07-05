#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace optitrack_vrpn_clean
{

struct PrintConfig
{
  bool enabled = true;
  double rate_hz = 20.0;
};

struct CsvConfig
{
  bool enabled = false;
  std::string path = "/tmp/optitrack_vrpn_clean.csv";
  bool flush = true;
};

struct UdpConfig
{
  bool enabled = false;
  std::string host = "127.0.0.1";
  std::uint16_t port = 15001;
};

struct YamlConfig
{
  bool enabled = false;
  std::string directory = "/tmp/optitrack_poses";
  double rate_hz = 100.0;
  std::string pose_frame = "enu";
  bool flush = true;
};

struct Config
{
  std::string host = "192.168.151.100";
  double update_rate_hz = 500.0;
  double tracker_scan_rate_hz = 1.0;
  int connect_retries = 5;
  double retry_delay_s = 1.0;
  std::size_t offset_samples = 100;
  std::string frame = "optitrack";
  std::string tracker_filter;
  std::vector<std::string> blacklist = {"VRPN Control"};
  bool list_only = false;
  double list_duration_s = 3.0;

  PrintConfig print;
  CsvConfig csv;
  UdpConfig udp;
  YamlConfig yaml;
};

Config loadConfigFile(const std::string& path);
void validateConfig(const Config& config);

}  // namespace optitrack_vrpn_clean
