#include <optitrack_vrpn_clean/pose.hpp>
#include <optitrack_vrpn_clean/time_manager.hpp>

#include <arpa/inet.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <sys/time.h>
#include <thread>
#include <unordered_map>
#include <unistd.h>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace
{

std::atomic_bool g_stop_requested{false};

using Matrix3 = std::array<std::array<double, 3>, 3>;

struct Options
{
  std::string bind_host = "127.0.0.1";
  int udp_port = 15001;
  std::string base_name = "go2_base";
  std::string ee_name = "piper_ee";
  std::string frame = "enu";
  std::string output_path = "/tmp/mocap_pair_diagnostic.csv";
  double log_rate_hz = 50.0;
  double max_age_s = 0.20;
  double duration_s = -1.0;
  std::string axis_map_text = "x,y,z";
  Matrix3 axis_map{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
  optitrack_vrpn_clean::Quat axis_map_quat{0.0, 0.0, 0.0, 1.0};
  std::unordered_map<std::string, optitrack_vrpn_clean::Quat> local_axis_quats;
  std::unordered_map<std::string, optitrack_vrpn_clean::Vec3> local_offsets;
  std::unordered_map<std::string, optitrack_vrpn_clean::Quat> local_rpy_quats;
  std::unordered_map<std::string, optitrack_vrpn_clean::Vec3> child_offsets;
  bool set_world_from_base = false;
  std::string set_world_from_tracker;
};

struct Sample
{
  optitrack_vrpn_clean::Vec3 position;
  optitrack_vrpn_clean::Quat orientation;
  double receive_time_s = 0.0;
  double packet_timestamp_s = 0.0;
  std::uint64_t sequence = 0;
  bool valid = false;
};

struct PlanarWorldAlignment
{
  bool enabled = false;
  bool initialized = false;
  std::string tracker_name;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double yaw = 0.0;
  optitrack_vrpn_clean::Quat rotation{0.0, 0.0, 0.0, 1.0};
};

void signalHandler(int)
{
  g_stop_requested.store(true);
}

double nowSeconds()
{
  return optitrack_vrpn_clean::wallTimeSeconds();
}

void printUsage(const char* program)
{
  std::cout
    << "Usage: " << program << " [options]\n"
    << "\n"
    << "Options:\n"
    << "  --bind HOST        UDP bind host, default 127.0.0.1\n"
    << "  --udp-port PORT    UDP port from optitrack_vrpn_clean_node, default 15001\n"
    << "  --base NAME        Base rigid body name, default go2_base\n"
    << "  --ee NAME          End-effector rigid body name, default piper_ee\n"
    << "  --frame FRAME      Packet frame to use: enu or ned, default enu\n"
    << "  --out PATH         CSV output path, default /tmp/mocap_pair_diagnostic.csv\n"
    << "  --rate HZ          CSV logging rate, default 50\n"
    << "  --max-age SEC      Max accepted sample age, default 0.20\n"
    << "  --duration SEC     Stop after SEC seconds, default run until Ctrl-C\n"
    << "  --axis-map MAP     Remap packet world axes, e.g. z,-y,x. Default x,y,z\n"
    << "  --local-axis-map TRACKER:MAP\n"
    << "                     Per-tracker local frame correction, e.g. go2_base:z,-x,-y\n"
    << "  --local-offset TRACKER:X,Y,Z\n"
    << "                     Per-tracker local position offset in marker-frame meters\n"
    << "  --local-rpy-deg TRACKER:ROLL,PITCH,YAW\n"
    << "                     Extra per-tracker local RPY correction after --local-axis-map, degrees\n"
    << "  --child-offset TRACKER:X,Y,Z\n"
    << "                     Per-tracker position offset in final corrected child frame, meters\n"
    << "  --set-world-from-base\n"
    << "                     Set planar world x/y/yaw from the first corrected --base sample\n"
    << "  --set-world-from TRACKER\n"
    << "                     Set planar world x/y/yaw from the first corrected TRACKER sample\n"
    << "  -h, --help         Show this help\n";
}

std::string readValue(int& index, int argc, char** argv, const std::string& option)
{
  if (index + 1 >= argc)
  {
    throw std::runtime_error("missing value for " + option);
  }
  ++index;
  return argv[index];
}

std::string trim(const std::string& input)
{
  const auto first = input.find_first_not_of(" \t\r\n");
  if (first == std::string::npos)
  {
    return {};
  }
  const auto last = input.find_last_not_of(" \t\r\n");
  return input.substr(first, last - first + 1);
}

std::vector<std::string> split(const std::string& input, char delimiter)
{
  std::vector<std::string> parts;
  std::stringstream stream(input);
  std::string item;
  while (std::getline(stream, item, delimiter))
  {
    parts.push_back(trim(item));
  }
  return parts;
}

std::pair<char, std::array<double, 3>> parseAxisTerm(std::string term)
{
  term = trim(term);
  double sign = 1.0;
  if (!term.empty() && term.front() == '-')
  {
    sign = -1.0;
    term.erase(term.begin());
  }
  else if (!term.empty() && term.front() == '+')
  {
    term.erase(term.begin());
  }

  if (term.size() != 1 || (term[0] != 'x' && term[0] != 'y' && term[0] != 'z'))
  {
    throw std::runtime_error("axis term must be x, y, z with optional sign");
  }

  std::array<double, 3> vector{0.0, 0.0, 0.0};
  const int index = term[0] == 'x' ? 0 : (term[0] == 'y' ? 1 : 2);
  vector[index] = sign;
  return {term[0], vector};
}

double det3(const Matrix3& matrix)
{
  return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
         matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
         matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
}

Matrix3 parseAxisMap(const std::string& value)
{
  const auto terms = split(value, ',');
  if (terms.size() != 3)
  {
    throw std::runtime_error("--axis-map expects three comma-separated terms, e.g. z,-y,x");
  }

  Matrix3 matrix{};
  std::string used_axes;
  for (std::size_t i = 0; i < 3; ++i)
  {
    const auto parsed = parseAxisTerm(terms[i]);
    used_axes.push_back(parsed.first);
    matrix[i] = parsed.second;
  }

  std::sort(used_axes.begin(), used_axes.end());
  if (used_axes != "xyz")
  {
    throw std::runtime_error("--axis-map must use x, y, and z exactly once");
  }
  if (std::abs(det3(matrix) - 1.0) > 1.0e-9)
  {
    throw std::runtime_error("--axis-map must be a proper right-handed rotation with determinant +1");
  }
  return matrix;
}

std::pair<std::string, Matrix3> parseLocalAxisMap(const std::string& value)
{
  const auto separator = value.find(':');
  if (separator == std::string::npos || separator == 0 || separator + 1 >= value.size())
  {
    throw std::runtime_error("--local-axis-map expects TRACKER:MAP");
  }

  const std::string name = value.substr(0, separator);
  const auto terms = split(value.substr(separator + 1), ',');
  if (terms.size() != 3)
  {
    throw std::runtime_error("--local-axis-map expects three axis terms after TRACKER:");
  }

  std::array<std::array<double, 3>, 3> columns{};
  std::string used_axes;
  for (std::size_t i = 0; i < 3; ++i)
  {
    const auto parsed = parseAxisTerm(terms[i]);
    used_axes.push_back(parsed.first);
    columns[i] = parsed.second;
  }

  std::sort(used_axes.begin(), used_axes.end());
  if (used_axes != "xyz")
  {
    throw std::runtime_error("--local-axis-map must use x, y, and z exactly once");
  }

  Matrix3 matrix{};
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      matrix[row][col] = columns[col][row];
    }
  }

  if (std::abs(det3(matrix) - 1.0) > 1.0e-9)
  {
    throw std::runtime_error("--local-axis-map must be a proper right-handed rotation with determinant +1");
  }
  return {name, matrix};
}

std::pair<std::string, optitrack_vrpn_clean::Vec3> parseLocalOffset(const std::string& value)
{
  const auto separator = value.find(':');
  if (separator == std::string::npos || separator == 0 || separator + 1 >= value.size())
  {
    throw std::runtime_error("--local-offset expects TRACKER:X,Y,Z");
  }

  const std::string name = value.substr(0, separator);
  const auto fields = split(value.substr(separator + 1), ',');
  if (fields.size() != 3)
  {
    throw std::runtime_error("--local-offset expects three numeric values after TRACKER:");
  }
  return {name, {std::stod(fields[0]), std::stod(fields[1]), std::stod(fields[2])}};
}

std::pair<std::string, optitrack_vrpn_clean::Vec3> parseTrackerVector(
  const std::string& value,
  const std::string& option_name)
{
  const auto separator = value.find(':');
  if (separator == std::string::npos || separator == 0 || separator + 1 >= value.size())
  {
    throw std::runtime_error(option_name + " expects TRACKER:X,Y,Z");
  }

  const std::string name = value.substr(0, separator);
  const auto fields = split(value.substr(separator + 1), ',');
  if (fields.size() != 3)
  {
    throw std::runtime_error(option_name + " expects three numeric values after TRACKER:");
  }
  return {name, {std::stod(fields[0]), std::stod(fields[1]), std::stod(fields[2])}};
}

optitrack_vrpn_clean::Vec3 matVec(const Matrix3& matrix, const optitrack_vrpn_clean::Vec3& vector)
{
  return {
    matrix[0][0] * vector.x + matrix[0][1] * vector.y + matrix[0][2] * vector.z,
    matrix[1][0] * vector.x + matrix[1][1] * vector.y + matrix[1][2] * vector.z,
    matrix[2][0] * vector.x + matrix[2][1] * vector.y + matrix[2][2] * vector.z,
  };
}

optitrack_vrpn_clean::Quat quatFromMatrix(const Matrix3& matrix)
{
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  const double trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
  if (trace > 0.0)
  {
    const double scale = std::sqrt(trace + 1.0) * 2.0;
    qw = 0.25 * scale;
    qx = (matrix[2][1] - matrix[1][2]) / scale;
    qy = (matrix[0][2] - matrix[2][0]) / scale;
    qz = (matrix[1][0] - matrix[0][1]) / scale;
  }
  else if (matrix[0][0] > matrix[1][1] && matrix[0][0] > matrix[2][2])
  {
    const double scale = std::sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0;
    qw = (matrix[2][1] - matrix[1][2]) / scale;
    qx = 0.25 * scale;
    qy = (matrix[0][1] + matrix[1][0]) / scale;
    qz = (matrix[0][2] + matrix[2][0]) / scale;
  }
  else if (matrix[1][1] > matrix[2][2])
  {
    const double scale = std::sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0;
    qw = (matrix[0][2] - matrix[2][0]) / scale;
    qx = (matrix[0][1] + matrix[1][0]) / scale;
    qy = 0.25 * scale;
    qz = (matrix[1][2] + matrix[2][1]) / scale;
  }
  else
  {
    const double scale = std::sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0;
    qw = (matrix[1][0] - matrix[0][1]) / scale;
    qx = (matrix[0][2] + matrix[2][0]) / scale;
    qy = (matrix[1][2] + matrix[2][1]) / scale;
    qz = 0.25 * scale;
  }
  return optitrack_vrpn_clean::normalize({qx, qy, qz, qw});
}

optitrack_vrpn_clean::Quat quatFromRpyDeg(double roll_deg, double pitch_deg, double yaw_deg)
{
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kDegToRad = kPi / 180.0;
  const double roll = roll_deg * kDegToRad;
  const double pitch = pitch_deg * kDegToRad;
  const double yaw = yaw_deg * kDegToRad;
  const double cr = std::cos(0.5 * roll);
  const double sr = std::sin(0.5 * roll);
  const double cp = std::cos(0.5 * pitch);
  const double sp = std::sin(0.5 * pitch);
  const double cy = std::cos(0.5 * yaw);
  const double sy = std::sin(0.5 * yaw);
  return optitrack_vrpn_clean::normalize({
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy});
}

optitrack_vrpn_clean::Quat quatFromYaw(double yaw_rad)
{
  return optitrack_vrpn_clean::normalize(
    {0.0, 0.0, std::sin(0.5 * yaw_rad), std::cos(0.5 * yaw_rad)});
}

double yawFromQuat(const optitrack_vrpn_clean::Quat& q_in)
{
  const auto q = optitrack_vrpn_clean::normalize(q_in);
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

Options parseArgs(int argc, char** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help")
    {
      printUsage(argv[0]);
      std::exit(0);
    }
    if (arg == "--bind")
    {
      options.bind_host = readValue(i, argc, argv, arg);
    }
    else if (arg == "--udp-port")
    {
      options.udp_port = std::stoi(readValue(i, argc, argv, arg));
    }
    else if (arg == "--base")
    {
      options.base_name = readValue(i, argc, argv, arg);
    }
    else if (arg == "--ee")
    {
      options.ee_name = readValue(i, argc, argv, arg);
    }
    else if (arg == "--frame")
    {
      options.frame = readValue(i, argc, argv, arg);
    }
    else if (arg == "--out")
    {
      options.output_path = readValue(i, argc, argv, arg);
    }
    else if (arg == "--rate")
    {
      options.log_rate_hz = std::stod(readValue(i, argc, argv, arg));
    }
    else if (arg == "--max-age")
    {
      options.max_age_s = std::stod(readValue(i, argc, argv, arg));
    }
    else if (arg == "--duration")
    {
      options.duration_s = std::stod(readValue(i, argc, argv, arg));
    }
    else if (arg == "--axis-map")
    {
      options.axis_map_text = readValue(i, argc, argv, arg);
      options.axis_map = parseAxisMap(options.axis_map_text);
      options.axis_map_quat = quatFromMatrix(options.axis_map);
    }
    else if (arg == "--local-axis-map")
    {
      const auto parsed = parseLocalAxisMap(readValue(i, argc, argv, arg));
      options.local_axis_quats[parsed.first] = quatFromMatrix(parsed.second);
    }
    else if (arg == "--local-offset")
    {
      const auto parsed = parseLocalOffset(readValue(i, argc, argv, arg));
      options.local_offsets[parsed.first] = parsed.second;
    }
    else if (arg == "--local-rpy-deg")
    {
      const auto parsed = parseTrackerVector(readValue(i, argc, argv, arg), "--local-rpy-deg");
      options.local_rpy_quats[parsed.first] =
        quatFromRpyDeg(parsed.second.x, parsed.second.y, parsed.second.z);
    }
    else if (arg == "--child-offset")
    {
      const auto parsed = parseTrackerVector(readValue(i, argc, argv, arg), "--child-offset");
      options.child_offsets[parsed.first] = parsed.second;
    }
    else if (arg == "--set-world-from-base")
    {
      options.set_world_from_base = true;
    }
    else if (arg == "--set-world-from")
    {
      options.set_world_from_tracker = readValue(i, argc, argv, arg);
    }
    else
    {
      throw std::runtime_error("unknown option: " + arg);
    }
  }

  if (options.udp_port <= 0 || options.udp_port > 65535)
  {
    throw std::runtime_error("--udp-port must be in [1, 65535]");
  }
  if (options.log_rate_hz <= 0.0)
  {
    throw std::runtime_error("--rate must be positive");
  }
  if (options.max_age_s < 0.0)
  {
    throw std::runtime_error("--max-age must be non-negative");
  }
  if (options.frame != "enu" && options.frame != "ned")
  {
    throw std::runtime_error("--frame must be enu or ned");
  }
  if (options.set_world_from_base)
  {
    if (!options.set_world_from_tracker.empty() &&
        options.set_world_from_tracker != options.base_name)
    {
      throw std::runtime_error("--set-world-from-base and --set-world-from disagree");
    }
    options.set_world_from_tracker = options.base_name;
  }
  return options;
}

optitrack_vrpn_clean::Quat conjugate(const optitrack_vrpn_clean::Quat& q)
{
  return {-q.x, -q.y, -q.z, q.w};
}

optitrack_vrpn_clean::Vec3 subtract(const optitrack_vrpn_clean::Vec3& lhs,
                                    const optitrack_vrpn_clean::Vec3& rhs)
{
  return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

optitrack_vrpn_clean::Vec3 rotate(const optitrack_vrpn_clean::Quat& q_in,
                                  const optitrack_vrpn_clean::Vec3& v)
{
  const auto q = optitrack_vrpn_clean::normalize(q_in);
  const optitrack_vrpn_clean::Quat vq{v.x, v.y, v.z, 0.0};
  const auto rotated = optitrack_vrpn_clean::multiply(
    optitrack_vrpn_clean::multiply(q, vq),
    conjugate(q));
  return {rotated.x, rotated.y, rotated.z};
}

optitrack_vrpn_clean::Vec3 add(const optitrack_vrpn_clean::Vec3& lhs,
                               const optitrack_vrpn_clean::Vec3& rhs)
{
  return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

void applyFrameCorrections(const Options& options, const std::string& name, Sample& sample)
{
  sample.position = matVec(options.axis_map, sample.position);
  sample.orientation = optitrack_vrpn_clean::normalize(
    optitrack_vrpn_clean::multiply(options.axis_map_quat, sample.orientation));

  const auto offset_it = options.local_offsets.find(name);
  if (offset_it != options.local_offsets.end())
  {
    sample.position = add(sample.position, rotate(sample.orientation, offset_it->second));
  }

  const auto axis_it = options.local_axis_quats.find(name);
  if (axis_it != options.local_axis_quats.end())
  {
    sample.orientation = optitrack_vrpn_clean::normalize(
      optitrack_vrpn_clean::multiply(sample.orientation, axis_it->second));
  }

  const auto rpy_it = options.local_rpy_quats.find(name);
  if (rpy_it != options.local_rpy_quats.end())
  {
    sample.orientation = optitrack_vrpn_clean::normalize(
      optitrack_vrpn_clean::multiply(sample.orientation, rpy_it->second));
  }

  const auto child_offset_it = options.child_offsets.find(name);
  if (child_offset_it != options.child_offsets.end())
  {
    sample.position = add(sample.position, rotate(sample.orientation, child_offset_it->second));
  }
}

void initializeWorldAlignment(PlanarWorldAlignment& alignment, const Sample& sample)
{
  alignment.initialized = true;
  alignment.origin_x = sample.position.x;
  alignment.origin_y = sample.position.y;
  alignment.yaw = yawFromQuat(sample.orientation);
  alignment.rotation = quatFromYaw(-alignment.yaw);
}

void applyWorldAlignment(const PlanarWorldAlignment& alignment, Sample& sample)
{
  if (!alignment.enabled || !alignment.initialized)
  {
    return;
  }

  const double dx = sample.position.x - alignment.origin_x;
  const double dy = sample.position.y - alignment.origin_y;
  const double c = std::cos(-alignment.yaw);
  const double s = std::sin(-alignment.yaw);
  sample.position.x = c * dx - s * dy;
  sample.position.y = s * dx + c * dy;
  sample.orientation = optitrack_vrpn_clean::normalize(
    optitrack_vrpn_clean::multiply(alignment.rotation, sample.orientation));
}

int makeUdpSocket(const Options& options)
{
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0)
  {
    throw std::runtime_error(std::string("failed to open UDP socket: ") + std::strerror(errno));
  }

  int reuse = 1;
  (void)::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  timeval timeout{};
  timeout.tv_sec = 0;
  timeout.tv_usec = 20000;
  (void)::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<std::uint16_t>(options.udp_port));
  if (options.bind_host.empty() || options.bind_host == "0.0.0.0")
  {
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
  }
  else if (::inet_pton(AF_INET, options.bind_host.c_str(), &addr.sin_addr) != 1)
  {
    ::close(fd);
    throw std::runtime_error("invalid bind host: " + options.bind_host);
  }

  if (::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0)
  {
    const std::string error = std::strerror(errno);
    ::close(fd);
    throw std::runtime_error("failed to bind UDP socket: " + error);
  }
  return fd;
}

bool parsePacket(const std::string& packet, const Options& options, std::string& name, Sample& sample)
{
  const YAML::Node root = YAML::Load(packet);
  if (!root["name"])
  {
    return false;
  }
  const YAML::Node frame_node = root[options.frame];
  if (!frame_node || !frame_node["p"] || !frame_node["q_xyzw"])
  {
    return false;
  }

  const YAML::Node p = frame_node["p"];
  const YAML::Node q = frame_node["q_xyzw"];
  if (!p.IsSequence() || p.size() < 3 || !q.IsSequence() || q.size() < 4)
  {
    return false;
  }

  name = root["name"].as<std::string>();
  sample.position = {p[0].as<double>(), p[1].as<double>(), p[2].as<double>()};
  sample.orientation = optitrack_vrpn_clean::normalize(
    {q[0].as<double>(), q[1].as<double>(), q[2].as<double>(), q[3].as<double>()});
  sample.packet_timestamp_s = root["timestamp_s"] ? root["timestamp_s"].as<double>() : 0.0;
  sample.sequence = root["sequence"] ? root["sequence"].as<std::uint64_t>() : 0;
  sample.receive_time_s = nowSeconds();
  sample.valid = true;
  applyFrameCorrections(options, name, sample);
  return true;
}

void writeHeader(std::ofstream& out)
{
  out << "time_s,base_age_s,ee_age_s,base_seq,ee_seq,"
      << "base_px,base_py,base_pz,base_qx,base_qy,base_qz,base_qw,"
      << "ee_px,ee_py,ee_pz,ee_qx,ee_qy,ee_qz,ee_qw,"
      << "ee_in_base_x,ee_in_base_y,ee_in_base_z,"
      << "ee_in_base_qx,ee_in_base_qy,ee_in_base_qz,ee_in_base_qw\n";
}

void writeRow(std::ofstream& out, const Sample& base, const Sample& ee, double time_s)
{
  const auto base_inv = conjugate(optitrack_vrpn_clean::normalize(base.orientation));
  const auto ee_in_base_p = rotate(base_inv, subtract(ee.position, base.position));
  const auto ee_in_base_q = optitrack_vrpn_clean::normalize(
    optitrack_vrpn_clean::multiply(base_inv, ee.orientation));

  out << std::fixed << std::setprecision(9)
      << time_s << ','
      << time_s - base.receive_time_s << ','
      << time_s - ee.receive_time_s << ','
      << base.sequence << ','
      << ee.sequence << ','
      << base.position.x << ','
      << base.position.y << ','
      << base.position.z << ','
      << base.orientation.x << ','
      << base.orientation.y << ','
      << base.orientation.z << ','
      << base.orientation.w << ','
      << ee.position.x << ','
      << ee.position.y << ','
      << ee.position.z << ','
      << ee.orientation.x << ','
      << ee.orientation.y << ','
      << ee.orientation.z << ','
      << ee.orientation.w << ','
      << ee_in_base_p.x << ','
      << ee_in_base_p.y << ','
      << ee_in_base_p.z << ','
      << ee_in_base_q.x << ','
      << ee_in_base_q.y << ','
      << ee_in_base_q.z << ','
      << ee_in_base_q.w << '\n';
}

}  // namespace

int main(int argc, char** argv)
{
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  try
  {
    const Options options = parseArgs(argc, argv);
    const int udp_fd = makeUdpSocket(options);
    std::ofstream out(options.output_path, std::ios::out | std::ios::trunc);
    if (!out.is_open())
    {
      ::close(udp_fd);
      throw std::runtime_error("failed to open output CSV: " + options.output_path);
    }
    writeHeader(out);

    std::unordered_map<std::string, Sample> samples;
    PlanarWorldAlignment world_alignment;
    world_alignment.enabled = !options.set_world_from_tracker.empty();
    world_alignment.tracker_name = options.set_world_from_tracker;
    const double start_s = nowSeconds();
    double next_log_s = start_s;
    double last_status_s = start_s;
    const double log_period_s = 1.0 / options.log_rate_hz;

    std::cout << "Listening on " << options.bind_host << ':' << options.udp_port
              << " base=" << options.base_name
              << " ee=" << options.ee_name
              << " frame=" << options.frame
              << " axis_map=" << options.axis_map_text
              << " out=" << options.output_path << '\n';
    if (!options.local_axis_quats.empty())
    {
      std::cout << "local_axis_maps:";
      for (const auto& entry : options.local_axis_quats)
      {
        std::cout << ' ' << entry.first;
      }
      std::cout << '\n';
    }
    if (!options.local_offsets.empty())
    {
      std::cout << "local_offsets:";
      for (const auto& entry : options.local_offsets)
      {
        std::cout << ' ' << entry.first << "=("
                  << entry.second.x << ',' << entry.second.y << ',' << entry.second.z << ')';
      }
      std::cout << '\n';
    }
    if (!options.local_rpy_quats.empty())
    {
      std::cout << "local_rpy_deg:";
      for (const auto& entry : options.local_rpy_quats)
      {
        std::cout << ' ' << entry.first;
      }
      std::cout << '\n';
    }
    if (!options.child_offsets.empty())
    {
      std::cout << "child_offsets:";
      for (const auto& entry : options.child_offsets)
      {
        std::cout << ' ' << entry.first << "=("
                  << entry.second.x << ',' << entry.second.y << ',' << entry.second.z << ')';
      }
      std::cout << '\n';
    }
    if (world_alignment.enabled)
    {
      std::cout << "set_world_from=" << world_alignment.tracker_name
                << " mode=planar_xy_yaw\n";
    }

    char buffer[4096];
    while (!g_stop_requested.load())
    {
      if (options.duration_s > 0.0 && nowSeconds() - start_s >= options.duration_s)
      {
        break;
      }

      sockaddr_storage source_addr{};
      socklen_t source_len = sizeof(source_addr);
      const ssize_t received = ::recvfrom(
        udp_fd,
        buffer,
        sizeof(buffer) - 1,
        0,
        reinterpret_cast<sockaddr*>(&source_addr),
        &source_len);
      if (received > 0)
      {
        buffer[received] = '\0';
        try
        {
          std::string name;
          Sample sample;
          if (parsePacket(std::string(buffer, static_cast<size_t>(received)), options, name, sample))
          {
            if (world_alignment.enabled)
            {
              if (!world_alignment.initialized)
              {
                if (name != world_alignment.tracker_name)
                {
                  continue;
                }
                initializeWorldAlignment(world_alignment, sample);
                std::cout << "set_world initialized from " << world_alignment.tracker_name
                          << " origin_xy=(" << world_alignment.origin_x
                          << ',' << world_alignment.origin_y
                          << ") yaw_deg="
                          << world_alignment.yaw * 180.0 / 3.14159265358979323846
                          << '\n';
              }
              applyWorldAlignment(world_alignment, sample);
            }
            samples[name] = sample;
          }
        }
        catch (const std::exception&)
        {
        }
      }
      else if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
      {
        std::cerr << "UDP receive failed: " << std::strerror(errno) << '\n';
      }

      const double now_s = nowSeconds();
      if (now_s >= next_log_s)
      {
        next_log_s += log_period_s;
        const auto base_it = samples.find(options.base_name);
        const auto ee_it = samples.find(options.ee_name);
        if (base_it != samples.end() && ee_it != samples.end())
        {
          const double base_age_s = now_s - base_it->second.receive_time_s;
          const double ee_age_s = now_s - ee_it->second.receive_time_s;
          if (base_age_s <= options.max_age_s && ee_age_s <= options.max_age_s)
          {
            writeRow(out, base_it->second, ee_it->second, now_s);
          }
        }
      }

      if (now_s - last_status_s >= 2.0)
      {
        last_status_s = now_s;
        const bool has_base = samples.count(options.base_name) > 0;
        const bool has_ee = samples.count(options.ee_name) > 0;
        std::cout << "samples: " << options.base_name << '=' << (has_base ? "yes" : "no")
                  << " " << options.ee_name << '=' << (has_ee ? "yes" : "no") << '\n';
      }
    }

    out.flush();
    out.close();
    ::close(udp_fd);
    std::cout << "Wrote " << options.output_path << '\n';
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "mocap_pair_diagnostic: " << e.what() << '\n';
    return 1;
  }
}
