#include "ConnectionConfiguration.h"

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_rtc_rviz
{

ConnectionConfiguration::ConnectionConfiguration()
: protocol_(Protocol::IPC), host_((bfs::temp_directory_path() / "mc_rtc").string()), sub_suffix_("_pub.ipc"),
  push_suffix_("_rep.ipc")
{
}

ConnectionConfiguration::ConnectionConfiguration(const std::string & host) : ConnectionConfiguration()
{
  host_ = host;
}

namespace
{
std::string n2s(unsigned int i)
{
  std::stringstream ss;
  ss << i;
  return ss.str();
}
} // namespace

ConnectionConfiguration::ConnectionConfiguration(const std::string & host, unsigned int s, unsigned int p)
: ConnectionConfiguration(Protocol::TCP, host, n2s(s), n2s(p))
{
}

ConnectionConfiguration::ConnectionConfiguration(Protocol proto,
                                                 const std::string & host,
                                                 const std::string & sub,
                                                 const std::string & push)
: protocol_(proto), host_(host), sub_suffix_(sub), push_suffix_(push)
{
}

std::string ConnectionConfiguration::toText() const
{
  if(protocol_ == Protocol::IPC) { return "IPC | " + host_; }
  return "TCP | " + host_ + ":" + sub_suffix_ + "|" + push_suffix_;
}

std::string ConnectionConfiguration::sub_uri() const
{
  if(protocol_ == Protocol::IPC) { return "ipc://" + host_ + sub_suffix_; }
  return "tcp://" + host_ + ":" + sub_suffix_;
}

std::string ConnectionConfiguration::push_uri() const
{
  if(protocol_ == Protocol::IPC) { return "ipc://" + host_ + push_suffix_; }
  return "tcp://" + host_ + ":" + push_suffix_;
}

} // namespace mc_rtc_rviz

namespace mc_rtc
{

using ConnectionConfiguration = mc_rtc_rviz::ConnectionConfiguration;
using Protocol = ConnectionConfiguration::Protocol;

ConnectionConfiguration ConfigurationLoader<ConnectionConfiguration>::load(const Configuration & cfg)
{
  Protocol proto = static_cast<std::string>(cfg("type")) == "IPC" ? Protocol::IPC : Protocol::TCP;
  if(proto == Protocol::IPC) { return ConnectionConfiguration(static_cast<std::string>(cfg("host"))); }
  else
  {
    return ConnectionConfiguration(cfg("host"), cfg("sub_suffix"), cfg("push_suffix"));
  }
}

Configuration ConfigurationLoader<ConnectionConfiguration>::save(const ConnectionConfiguration & config)
{
  mc_rtc::Configuration cfg;
  cfg.add("type", config.protocol() == Protocol::IPC ? "IPC" : "TCP");
  cfg.add("host", config.host());
  cfg.add("sub_suffix", config.sub_suffix());
  cfg.add("push_suffix", config.push_suffix());
  return cfg;
}

} // namespace mc_rtc
