#include <optitrack_vrpn_clean/optitrack_client.hpp>

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

std::atomic_bool g_stop_requested{false};

void signalHandler(int)
{
  g_stop_requested.store(true);
}

void printUsage(const char* program)
{
  std::cout
    << "Usage: " << program << " [options]\n"
    << "\n"
    << "Options:\n"
    << "  -c, --config PATH       Load YAML config file\n"
    << "      --host HOST         VRPN server host or host:port\n"
    << "      --tracker NAME      Only subscribe to one rigid body\n"
    << "      --update-rate HZ    VRPN mainloop rate\n"
    << "      --list              Print discovered tracker names and exit\n"
    << "      --no-print          Disable console pose printing\n"
    << "      --print-rate HZ     Enable console printing at rate\n"
    << "      --csv PATH          Enable CSV logging to PATH\n"
    << "      --udp HOST:PORT     Enable UDP JSON output\n"
    << "      --yaml-dir PATH     Write one YAML pose file per tracker under PATH\n"
    << "      --yaml-rate HZ      YAML output rate per tracker\n"
    << "      --yaml-frame FRAME  YAML position_w/quaternion_wxyz frame: enu, ned, optitrack\n"
    << "  -h, --help              Show this help\n";
}

bool isArg(const std::string& arg, const std::string& long_name, const std::string& short_name = "")
{
  return arg == long_name || (!short_name.empty() && arg == short_name);
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

double readDouble(int& index, int argc, char** argv, const std::string& option)
{
  return std::stod(readValue(index, argc, argv, option));
}

void applyUdpArg(optitrack_vrpn_clean::Config& config, const std::string& value)
{
  const std::size_t separator = value.rfind(':');
  if (separator == std::string::npos || separator == 0 || separator + 1 >= value.size())
  {
    throw std::runtime_error("--udp expects HOST:PORT");
  }

  config.udp.enabled = true;
  config.udp.host = value.substr(0, separator);
  config.udp.port = static_cast<std::uint16_t>(std::stoul(value.substr(separator + 1)));
}

}  // namespace

int main(int argc, char** argv)
{
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  try
  {
    std::string config_path;
    for (int i = 1; i < argc; ++i)
    {
      const std::string arg = argv[i];
      if (isArg(arg, "--help", "-h"))
      {
        printUsage(argv[0]);
        return 0;
      }
      if (isArg(arg, "--config", "-c"))
      {
        config_path = readValue(i, argc, argv, arg);
      }
    }

    optitrack_vrpn_clean::Config config;
    if (!config_path.empty())
    {
      config = optitrack_vrpn_clean::loadConfigFile(config_path);
    }

    for (int i = 1; i < argc; ++i)
    {
      const std::string arg = argv[i];
      if (isArg(arg, "--config", "-c"))
      {
        (void)readValue(i, argc, argv, arg);
      }
      else if (isArg(arg, "--help", "-h"))
      {
        printUsage(argv[0]);
        return 0;
      }
      else if (arg == "--host")
      {
        config.host = readValue(i, argc, argv, arg);
      }
      else if (arg == "--tracker")
      {
        config.tracker_filter = readValue(i, argc, argv, arg);
      }
      else if (arg == "--update-rate")
      {
        config.update_rate_hz = readDouble(i, argc, argv, arg);
      }
      else if (arg == "--list")
      {
        config.list_only = true;
      }
      else if (arg == "--no-print")
      {
        config.print.enabled = false;
      }
      else if (arg == "--print-rate")
      {
        config.print.enabled = true;
        config.print.rate_hz = readDouble(i, argc, argv, arg);
      }
      else if (arg == "--csv")
      {
        config.csv.enabled = true;
        config.csv.path = readValue(i, argc, argv, arg);
      }
      else if (arg == "--udp")
      {
        applyUdpArg(config, readValue(i, argc, argv, arg));
      }
      else if (arg == "--yaml-dir")
      {
        config.yaml.enabled = true;
        config.yaml.directory = readValue(i, argc, argv, arg);
      }
      else if (arg == "--yaml-rate")
      {
        config.yaml.enabled = true;
        config.yaml.rate_hz = readDouble(i, argc, argv, arg);
      }
      else if (arg == "--yaml-frame")
      {
        config.yaml.enabled = true;
        config.yaml.pose_frame = readValue(i, argc, argv, arg);
      }
      else
      {
        throw std::runtime_error("unknown option: " + arg);
      }
    }

    optitrack_vrpn_clean::validateConfig(config);
    optitrack_vrpn_clean::OptitrackClient client(config);
    return client.run(&g_stop_requested);
  }
  catch (const std::exception& e)
  {
    std::cerr << "optitrack_vrpn_clean_node: " << e.what() << '\n';
    return 1;
  }
}
