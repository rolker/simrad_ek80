#ifndef SIMRAD_EK80_PARAMETER_MANAGER_H
#define SIMRAD_EK80_PARAMETER_MANAGER_H

#include <simrad_ek80/udp/parameter.h>
#include <simrad_ek80/udp/udp_socket.h>
#include <simrad_ek80/udp/connection.h>
#include <simrad_ek80/udp/request.h>
#include <simrad_ek80/udp/callbacks.h>

namespace simrad
{

/// Manages subscriptions to parameters.
class ParameterManager: public UDPSocket, public Callbacks<simrad::TimePoint>
{
public:
  typedef std::shared_ptr<ParameterManager> Ptr;

  ParameterManager(Connection::Ptr& connection);

  // Make non-copyable
  ParameterManager(const ParameterManager&) = delete;
  ParameterManager& operator=(const ParameterManager&) = delete;
                
  ~ParameterManager();

  Parameter::Ptr subscribe(const std::string& parameter_name, bool use_default = true);
  Parameter::Ptr get(const std::string& parameter_name);
                
  Parameter::Info getInfo(const std::string& parameter_name);
                
  std::string getValue(const std::string& parameter_name);

private:
  void receivePacket(const std::vector<uint8_t>& packet) override;

  Connection::Ptr connection_;

  std::mutex parameters_mutex_;
  std::map<int,Parameter::Ptr> parameters_;
  std::map<std::string, int> parameter_ids_;
};


} // namespace simrad

#endif
