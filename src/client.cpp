#include <simrad_ek80/client.h>

namespace simrad
{

Client::Client(Server const &s)
  :server_(s)
{
}

Client::Ptr Client::create(Server const &s)
{
  Ptr ret(new  Client(s));
  return ret;
}

void Client::connect(std::string uname, std::string pwd)
{
  connection_ = Connection::Ptr(new Connection(server_.getAddress(),uname,pwd));
    
  subscription_port_ = SubscriptionPort::Ptr(new SubscriptionPort);
  parameter_manager_ = ParameterManager::Ptr(new ParameterManager(connection_, subscription_port_));
    
  platform_ = Platform::Ptr(new Platform(parameter_manager_));
    
  // beamConfigurationManager_ = BeamConfigurationManager::Ptr(new BeamConfigurationManager(parameterManager_));
    
  // subscriptionManager_ = SubscriptionManager::Ptr(new SubscriptionManager(connection_));
    
  // transducer_ = Transducer::Ptr(new Transducer(parameterManager_, subscriptionManager_));
}

Platform::Ptr Client::getPlatform()
{
  return platform_;
}

// Transducer::Ptr Client::getTransducer()
// {
//   return transducer_;
// }
        
// BeamConfigurationManager::Ptr Client::getBeamConfigurationManager()
// {
//   return beamConfigurationManager_;
// }


} // namespace simrad
