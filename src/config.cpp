#include <optitrack_vrpn_clean/config.hpp>

#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace optitrack_vrpn_clean
{
namespace
{

template <typename T>
void assignIfPresent(const YAML::Node& node, const char* key, T& value)
{
  if (node[key])
  {
    value = node[key].as<T>();
  }
}

void readStringVectorIfPresent(const YAML::Node& node,
                               const char* key,
                               std::vector<std::string>& value)
{
  if (!node[key])
  {
    return;
  }

  value.clear();
  for (const auto& entry : node[key])
  {
    value.push_back(entry.as<std::string>());
  }
}

}  // namespace

Config loadConfigFile(const std::string& path)
{
  Config config;
  const YAML::Node root = YAML::LoadFile(path);

  assignIfPresent(root, "host", config.host);
  assignIfPresent(root, "update_rate_hz", config.update_rate_hz);
  assignIfPresent(root, "tracker_scan_rate_hz", config.tracker_scan_rate_hz);
  assignIfPresent(root, "connect_retries", config.connect_retries);
  assignIfPresent(root, "retry_delay_s", config.retry_delay_s);
  assignIfPresent(root, "offset_samples", config.offset_samples);
  assignIfPresent(root, "frame", config.frame);
  assignIfPresent(root, "tracker_filter", config.tracker_filter);
  readStringVectorIfPresent(root, "blacklist", config.blacklist);

  if (root["print"])
  {
    assignIfPresent(root["print"], "enabled", config.print.enabled);
    assignIfPresent(root["print"], "rate_hz", config.print.rate_hz);
  }

  if (root["csv"])
  {
    assignIfPresent(root["csv"], "enabled", config.csv.enabled);
    assignIfPresent(root["csv"], "path", config.csv.path);
    assignIfPresent(root["csv"], "flush", config.csv.flush);
  }

  if (root["udp"])
  {
    assignIfPresent(root["udp"], "enabled", config.udp.enabled);
    assignIfPresent(root["udp"], "host", config.udp.host);
    assignIfPresent(root["udp"], "port", config.udp.port);
  }

  if (root["yaml"])
  {
    assignIfPresent(root["yaml"], "enabled", config.yaml.enabled);
    assignIfPresent(root["yaml"], "directory", config.yaml.directory);
    assignIfPresent(root["yaml"], "rate_hz", config.yaml.rate_hz);
    assignIfPresent(root["yaml"], "pose_frame", config.yaml.pose_frame);
    assignIfPresent(root["yaml"], "flush", config.yaml.flush);
  }

  validateConfig(config);
  return config;
}

void validateConfig(const Config& config)
{
  if (config.host.empty())
  {
    throw std::runtime_error("host must not be empty");
  }
  if (config.update_rate_hz <= 0.0)
  {
    throw std::runtime_error("update_rate_hz must be positive");
  }
  if (config.tracker_scan_rate_hz <= 0.0)
  {
    throw std::runtime_error("tracker_scan_rate_hz must be positive");
  }
  if (config.retry_delay_s < 0.0)
  {
    throw std::runtime_error("retry_delay_s must be non-negative");
  }
  if (config.print.enabled && config.print.rate_hz <= 0.0)
  {
    throw std::runtime_error("print.rate_hz must be positive when printing is enabled");
  }
  if (config.csv.enabled && config.csv.path.empty())
  {
    throw std::runtime_error("csv.path must not be empty when csv.enabled is true");
  }
  if (config.udp.enabled && config.udp.host.empty())
  {
    throw std::runtime_error("udp.host must not be empty when udp.enabled is true");
  }
  if (config.yaml.enabled && config.yaml.directory.empty())
  {
    throw std::runtime_error("yaml.directory must not be empty when yaml.enabled is true");
  }
  if (config.yaml.enabled && config.yaml.rate_hz <= 0.0)
  {
    throw std::runtime_error("yaml.rate_hz must be positive when yaml.enabled is true");
  }
  if (config.yaml.pose_frame != "enu" &&
      config.yaml.pose_frame != "ned" &&
      config.yaml.pose_frame != "optitrack")
  {
    throw std::runtime_error("yaml.pose_frame must be one of: enu, ned, optitrack");
  }
}

}  // namespace optitrack_vrpn_clean
