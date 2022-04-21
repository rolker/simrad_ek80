#ifndef SIMRAD_EK80_PARAMETER_MANAGER_H
#define SIMRAD_EK80_PARAMETER_MANAGER_H

#include <simrad_ek80/parameter.h>
#include <simrad_ek80/subscription_port.h>
#include <simrad_ek80/connection.h>
#include <simrad_ek80/request.h>

namespace simrad
{

/// Manages subscriptions to parameters.
class ParameterManager
{
public:
  typedef std::shared_ptr<ParameterManager> Ptr;

  ParameterManager(Connection::Ptr& connection, SubscriptionPort::Ptr& subscription_port);

  // Make non-copyable
  ParameterManager(const ParameterManager&) = delete;
  ParameterManager& operator=(const ParameterManager&) = delete;
                
  ~ParameterManager();

  Parameter::Ptr subscribe(const std::string& parameter_name, bool use_default = true);
                
  Parameter::Info getInfo(const std::string& parameter_name);
                
  std::string get(const std::string& parameter_name);

  void process_packet(const std::vector<uint8_t>& packet);

private:
  SubscriptionPort::Ptr subscription_port_;
  Connection::Ptr connection_;

  std::mutex parameters_mutex_;
  std::map<int,Parameter::Ptr> parameters_;
  std::map<int,std::string> parameter_names_;
};


} // namespace simrad

#endif
