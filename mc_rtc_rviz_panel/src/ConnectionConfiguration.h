#pragma once

#include <mc_rtc/Configuration.h>

namespace mc_rtc_rviz
{

struct ConnectionConfiguration
{
  enum class Protocol
  {
    IPC,
    TCP
  };

  /** Constructor with reasonable default URI */
  ConnectionConfiguration();

  /** IPC constructor */
  explicit ConnectionConfiguration(const std::string & file);

  /** TCP constructor */
  ConnectionConfiguration(const std::string & host, unsigned int sub_port, unsigned int push_port);

  /** Constructor with provided values */
  ConnectionConfiguration(Protocol proto, const std::string & host, const std::string & sub, const std::string & push);

  inline Protocol protocol() const { return protocol_; }

  inline const std::string & host() const { return host_; }

  inline const std::string & sub_suffix() const { return sub_suffix_; }

  inline const std::string & push_suffix() const { return push_suffix_; }

  std::string toText() const;

  std::string sub_uri() const;

  std::string push_uri() const;

private:
  Protocol protocol_;
  // File with icp, TCP host with tcp
  std::string host_;
  // File suffix with icp, TCP port with tcp
  std::string sub_suffix_;
  // File suffix with icp, TCP port with tcp
  std::string push_suffix_;
};

} // namespace mc_rtc_rviz

namespace mc_rtc
{
template<>
struct ConfigurationLoader<mc_rtc_rviz::ConnectionConfiguration>
{
  static mc_rtc_rviz::ConnectionConfiguration load(const mc_rtc::Configuration & config);
  static mc_rtc::Configuration save(const mc_rtc_rviz::ConnectionConfiguration & config);
};
} // namespace mc_rtc
