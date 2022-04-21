#ifndef SIMRAD_EK80_TRANSDUCER_H
#define SIMRAD_EK80_TRANSDUCER_H

#include <simrad_ek80/parameter_manager.h>
#include <simrad_ek80/channel.h>

namespace simrad
{

/// Represents a transducer, which may contain multiple channels.
class Transducer
{
public:
  typedef std::shared_ptr<Transducer> Ptr;

  Transducer(ParameterManager::Ptr& parameter_manager, SubscriptionManager::Ptr& subscription_manager);

  std::vector<Channel::Ptr> getChannels();
                
private:
  std::map<std::string, Channel::Ptr> channels_;

  ParameterManager::Ptr parameter_manager_;
  SubscriptionManager::Ptr subcription_manager_;

  ParameterGroup::Map ping_data_;

  void subscribe();
  
  bool subscribed_ = false;
};

} // namespace simrad

#endif
