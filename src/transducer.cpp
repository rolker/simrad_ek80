#include <simrad_ek80/transducer.h>

namespace simrad
{

Transducer::Transducer(ParameterManager::Ptr& parameter_manager, SubscriptionManager::Ptr& subscription_manager)
  :parameter_manager_(parameter_manager), subcription_manager_(subscription_manager), subscribed_(false)
{
  // ping_data_["offsets"] = ParameterGroup::Ptr(new ParameterGroup(parameter_manager));
  // ping_data_["offsets"]->add("x","OwnShip/TransducerOffsetX");
  // ping_data_["offsets"]->add("y","OwnShip/TransducerOffsetY");
  // ping_data_["offsets"]->add("z","OwnShip/TransducerOffsetZ");
  // ping_data_["offsets"]->add("heading","OwnShip/TransducerAlphaZ");
  // ping_data_["offsets"]->add("pitch","OwnShip/TransducerAlphaY");
  // ping_data_["offsets"]->add("roll","OwnShip/TransducerAlphaX");
  
  ping_data_["navigation"] = ParameterGroup::Ptr(new ParameterGroup(parameter_manager));
  
  std::string cs = parameter_manager->get("TransceiverMgr/Channels");
  std::vector<std::string> channelList = split(cs, ",");

  for(auto c: channelList)
    channels_[c] = std::make_shared<Channel>(parameter_manager, subscription_manager, ping_data_, c);
}

std::vector<Channel::Ptr> Transducer::getChannels()
{
  subscribe();
  std::vector<Channel::Ptr> ret;
  for(auto c: channels_)
    ret.push_back(c.second);
  return ret;
}

void Transducer::subscribe()
{
  if(!subscribed_)
  {
    ping_data_["navigation"]->add("lat","TransceiverMgr/Latitude",false);
    ping_data_["navigation"]->add("lon","TransceiverMgr/Longitude",false);
    ping_data_["navigation"]->add("heave","TransceiverMgr/Heave");
    ping_data_["navigation"]->add("roll","TransceiverMgr/Roll");
    ping_data_["navigation"]->add("pitch","TransceiverMgr/Pitch");
    //ping_data_["navigation"]->add("heading","MRUHeading",false);
    subscribed_ = true;
  }
}

} // namespace simrad
