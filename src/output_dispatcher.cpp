#include <optitrack_vrpn_clean/output_dispatcher.hpp>

#include <cerrno>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <sys/socket.h>
#include <unistd.h>

#include <optitrack_vrpn_clean/time_manager.hpp>

namespace optitrack_vrpn_clean
{
namespace
{

std::string jsonEscape(const std::string& input)
{
  std::ostringstream out;
  for (const char c : input)
  {
    switch (c)
    {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << c;
        break;
    }
  }
  return out.str();
}

void appendVec3Json(std::ostringstream& out, const Vec3& value)
{
  out << '[' << value.x << ',' << value.y << ',' << value.z << ']';
}

void appendQuatJson(std::ostringstream& out, const Quat& value)
{
  out << '[' << value.x << ',' << value.y << ',' << value.z << ',' << value.w << ']';
}

std::string yamlEscape(const std::string& input)
{
  std::ostringstream out;
  for (const char c : input)
  {
    if (c == '\\' || c == '"')
    {
      out << '\\';
    }
    out << c;
  }
  return out.str();
}

std::string yamlFileNameFromTracker(const std::string& name)
{
  std::ostringstream out;
  for (const char c : name)
  {
    const unsigned char uc = static_cast<unsigned char>(c);
    if (std::isalnum(uc) || c == '_' || c == '-' || c == '.')
    {
      out << c;
    }
    else
    {
      out << '_';
    }
  }

  const std::string file_name = out.str();
  return file_name.empty() ? "tracker" : file_name;
}

const Pose& poseForFrame(const TrackerSample& sample, const std::string& frame)
{
  if (frame == "ned")
  {
    return sample.ned;
  }
  if (frame == "optitrack")
  {
    return sample.optitrack;
  }
  return sample.enu;
}

}  // namespace

OutputDispatcher::OutputDispatcher(const Config& config) : config_(config)
{
  openCsv();
  setupUdp();
  setupYaml();
}

OutputDispatcher::~OutputDispatcher()
{
  if (udp_socket_ >= 0)
  {
    close(udp_socket_);
  }
}

void OutputDispatcher::handleSample(const TrackerSample& sample)
{
  printSample(sample);
  writeCsv(sample);
  sendUdp(sample);
  writeYaml(sample);
}

void OutputDispatcher::openCsv()
{
  if (!config_.csv.enabled)
  {
    return;
  }

  csv_stream_.open(config_.csv.path, std::ios::out | std::ios::trunc);
  if (!csv_stream_)
  {
    throw std::runtime_error("failed to open CSV output: " + config_.csv.path);
  }
}

void OutputDispatcher::setupUdp()
{
  if (!config_.udp.enabled)
  {
    return;
  }

  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;

  addrinfo* result = nullptr;
  const std::string port = std::to_string(config_.udp.port);
  const int rc = getaddrinfo(config_.udp.host.c_str(), port.c_str(), &hints, &result);
  if (rc != 0)
  {
    throw std::runtime_error("failed to resolve UDP target " + config_.udp.host + ":" + port +
                             ": " + gai_strerror(rc));
  }

  for (addrinfo* rp = result; rp != nullptr; rp = rp->ai_next)
  {
    udp_socket_ = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    if (udp_socket_ < 0)
    {
      continue;
    }

    std::memcpy(&udp_address_, rp->ai_addr, rp->ai_addrlen);
    udp_address_len_ = static_cast<socklen_t>(rp->ai_addrlen);
    break;
  }

  freeaddrinfo(result);

  if (udp_socket_ < 0)
  {
    throw std::runtime_error("failed to create UDP socket");
  }
}

void OutputDispatcher::setupYaml()
{
  if (!config_.yaml.enabled)
  {
    return;
  }

  std::error_code ec;
  std::filesystem::create_directories(config_.yaml.directory, ec);
  if (ec)
  {
    throw std::runtime_error("failed to create YAML output directory " +
                             config_.yaml.directory + ": " + ec.message());
  }
}

void OutputDispatcher::printSample(const TrackerSample& sample)
{
  if (!config_.print.enabled)
  {
    return;
  }

  const double now_s = wallTimeSeconds();
  const auto iter = last_print_wall_time_.find(sample.name);
  const double min_period_s = 1.0 / config_.print.rate_hz;
  if (iter != last_print_wall_time_.end() && now_s - iter->second < min_period_s)
  {
    return;
  }
  last_print_wall_time_[sample.name] = now_s;

  std::cout << std::fixed << std::setprecision(6)
            << '[' << sample.name << "] "
            << "t=" << sample.timestamp_s
            << " enu_p=(" << sample.enu.position.x << ", " << sample.enu.position.y << ", "
            << sample.enu.position.z << ")"
            << " enu_q_xyzw=(" << sample.enu.orientation.x << ", " << sample.enu.orientation.y
            << ", " << sample.enu.orientation.z << ", " << sample.enu.orientation.w << ")";
  if (sample.velocity_valid)
  {
    std::cout << " enu_v=(" << sample.enu_linear_velocity.x << ", "
              << sample.enu_linear_velocity.y << ", " << sample.enu_linear_velocity.z << ')';
  }
  std::cout << '\n';
}

void OutputDispatcher::writeCsv(const TrackerSample& sample)
{
  if (!config_.csv.enabled)
  {
    return;
  }

  if (!csv_header_written_)
  {
    csv_stream_ << "tracker,sequence,timestamp_s,source_timestamp_s,"
                << "enu_px,enu_py,enu_pz,enu_qx,enu_qy,enu_qz,enu_qw,"
                << "ned_px,ned_py,ned_pz,ned_qx,ned_qy,ned_qz,ned_qw,"
                << "enu_vx,enu_vy,enu_vz,velocity_valid,"
                << "raw_px,raw_py,raw_pz,raw_qx,raw_qy,raw_qz,raw_qw\n";
    csv_header_written_ = true;
  }

  csv_stream_ << std::fixed << std::setprecision(9)
              << '"' << sample.name << '"' << ','
              << sample.sequence << ','
              << sample.timestamp_s << ','
              << sample.source_timestamp_s << ','
              << sample.enu.position.x << ','
              << sample.enu.position.y << ','
              << sample.enu.position.z << ','
              << sample.enu.orientation.x << ','
              << sample.enu.orientation.y << ','
              << sample.enu.orientation.z << ','
              << sample.enu.orientation.w << ','
              << sample.ned.position.x << ','
              << sample.ned.position.y << ','
              << sample.ned.position.z << ','
              << sample.ned.orientation.x << ','
              << sample.ned.orientation.y << ','
              << sample.ned.orientation.z << ','
              << sample.ned.orientation.w << ','
              << sample.enu_linear_velocity.x << ','
              << sample.enu_linear_velocity.y << ','
              << sample.enu_linear_velocity.z << ','
              << (sample.velocity_valid ? 1 : 0) << ','
              << sample.optitrack.position.x << ','
              << sample.optitrack.position.y << ','
              << sample.optitrack.position.z << ','
              << sample.optitrack.orientation.x << ','
              << sample.optitrack.orientation.y << ','
              << sample.optitrack.orientation.z << ','
              << sample.optitrack.orientation.w << '\n';

  if (config_.csv.flush)
  {
    csv_stream_.flush();
  }
}

void OutputDispatcher::writeYaml(const TrackerSample& sample)
{
  if (!config_.yaml.enabled)
  {
    return;
  }

  const double now_s = wallTimeSeconds();
  const auto iter = last_yaml_wall_time_.find(sample.name);
  const double min_period_s = 1.0 / config_.yaml.rate_hz;
  if (iter != last_yaml_wall_time_.end() && now_s - iter->second < min_period_s)
  {
    return;
  }
  last_yaml_wall_time_[sample.name] = now_s;

  const std::filesystem::path directory(config_.yaml.directory);
  const std::filesystem::path path = directory / (yamlFileNameFromTracker(sample.name) + ".yaml");
  std::filesystem::path tmp_path = path;
  tmp_path += ".tmp";

  std::ofstream out(tmp_path, std::ios::out | std::ios::trunc);
  if (!out.is_open())
  {
    std::cerr << "Failed to open YAML output " << tmp_path.string() << '\n';
    return;
  }

  const Pose& world_pose = poseForFrame(sample, config_.yaml.pose_frame);
  out << std::fixed << std::setprecision(9);
  out << "body_name: \"" << yamlEscape(sample.name) << "\"\n";
  out << "sequence: " << sample.sequence << "\n";
  out << "timestamp_s: " << sample.timestamp_s << "\n";
  out << "source_timestamp_s: " << sample.source_timestamp_s << "\n";
  out << "frame: \"" << yamlEscape(config_.yaml.pose_frame) << "\"\n";
  out << "position_w: [" << world_pose.position.x << ", "
      << world_pose.position.y << ", " << world_pose.position.z << "]\n";
  out << "quaternion_wxyz: [" << world_pose.orientation.w << ", "
      << world_pose.orientation.x << ", " << world_pose.orientation.y << ", "
      << world_pose.orientation.z << "]\n";
  out << "position_enu: [" << sample.enu.position.x << ", "
      << sample.enu.position.y << ", " << sample.enu.position.z << "]\n";
  out << "quaternion_enu_xyzw: [" << sample.enu.orientation.x << ", "
      << sample.enu.orientation.y << ", " << sample.enu.orientation.z << ", "
      << sample.enu.orientation.w << "]\n";
  out << "velocity_enu: [" << sample.enu_linear_velocity.x << ", "
      << sample.enu_linear_velocity.y << ", " << sample.enu_linear_velocity.z << "]\n";
  out << "velocity_valid: " << (sample.velocity_valid ? "true" : "false") << "\n";
  out << "raw_position_optitrack: [" << sample.optitrack.position.x << ", "
      << sample.optitrack.position.y << ", " << sample.optitrack.position.z << "]\n";
  out << "raw_quaternion_xyzw: [" << sample.optitrack.orientation.x << ", "
      << sample.optitrack.orientation.y << ", " << sample.optitrack.orientation.z << ", "
      << sample.optitrack.orientation.w << "]\n";
  if (config_.yaml.flush)
  {
    out.flush();
  }
  out.close();

  std::error_code ec;
  std::filesystem::rename(tmp_path, path, ec);
  if (ec)
  {
    std::filesystem::remove(path, ec);
    ec.clear();
    std::filesystem::rename(tmp_path, path, ec);
  }
  if (ec)
  {
    std::cerr << "Failed to publish YAML output " << path.string() << ": "
              << ec.message() << '\n';
  }
}

void OutputDispatcher::sendUdp(const TrackerSample& sample)
{
  if (!config_.udp.enabled || udp_socket_ < 0)
  {
    return;
  }

  std::ostringstream out;
  out << std::fixed << std::setprecision(9)
      << "{\"name\":\"" << jsonEscape(sample.name) << "\","
      << "\"sequence\":" << sample.sequence << ','
      << "\"timestamp_s\":" << sample.timestamp_s << ','
      << "\"source_timestamp_s\":" << sample.source_timestamp_s << ','
      << "\"enu\":{\"p\":";
  appendVec3Json(out, sample.enu.position);
  out << ",\"q_xyzw\":";
  appendQuatJson(out, sample.enu.orientation);
  out << ",\"v\":";
  appendVec3Json(out, sample.enu_linear_velocity);
  out << "},\"ned\":{\"p\":";
  appendVec3Json(out, sample.ned.position);
  out << ",\"q_xyzw\":";
  appendQuatJson(out, sample.ned.orientation);
  out << "}}\n";

  const std::string payload = out.str();
  const ssize_t sent = sendto(udp_socket_,
                              payload.data(),
                              payload.size(),
                              0,
                              reinterpret_cast<const sockaddr*>(&udp_address_),
                              udp_address_len_);
  if (sent < 0)
  {
    std::cerr << "UDP send failed: " << std::strerror(errno) << '\n';
  }
}

}  // namespace optitrack_vrpn_clean
