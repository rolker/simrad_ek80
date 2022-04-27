#ifndef SIMRAD_EK80_CLIENT_H
#define SIMRAD_EK80_CLIENT_H

#include <memory>
#include <simrad_ek80/server.h>
#include <simrad_ek80/connection.h>
#include <simrad_ek80/parameter_manager.h>
#include <simrad_ek80/channel.h>

namespace simrad
{

/// Represents a client session with a Simrad server.
/// Establishes a connection and initiates managers for paramters and subscriptions.
class Client
{
public:
  typedef std::shared_ptr<Client> Ptr;

  Client(Server const &s);
        
  static Ptr create(Server const &s);
        
  void connect(std::string uname = "Simrad", std::string pwd = std::string());

  std::vector<Channel::Ptr> getChannels();
  std::shared_ptr<ParameterManager> getParameterManager();
private:
  Connection::Ptr connection_;
  Server server_;

  std::map<std::string, Channel::Ptr> channels_;

  ParameterManager::Ptr parameter_manager_;
  SubscriptionManager::Ptr subscription_manager_;
};

} // namespace simrad

#endif
