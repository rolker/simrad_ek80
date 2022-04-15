#ifndef SIMRAD_EK80_CLIENT_H
#define SIMRAD_EK80_CLIENT_H

#include <memory>
#include <simrad_ek80/server.h>
#include <simrad_ek80/connection.h>
#include <simrad_ek80/parameter_manager.h>
#include <simrad_ek80/platform.h>
//#include <simrad_ek80/transducer.h>
//#include <simrad_ek80/beam_configuration_manager.h>

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

  Platform::Ptr getPlatform();
        
  //Transducer::Ptr getTransducer();
        
  //BeamConfigurationManager::Ptr getBeamConfigurationManager();

private:
  Connection::Ptr connection_;

  Server server_;

  Platform::Ptr platform_;

  //Transducer::Ptr transducer_;
  //BeamConfigurationManager::Ptr beamConfigurationManager_;

  SubscriptionPort::Ptr subscription_port_;
  ParameterManager::Ptr parameter_manager_;

  //SubscriptionManager::Ptr subscriptionManager_;

};

} // namespace simrad

#endif
