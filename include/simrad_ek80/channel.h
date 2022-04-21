#ifndef SIMRAD_EK80_CHANNEL_H
#define SIMRAD_EK80_CHANNEL_H

#include <simrad_ek80/parameter_manager.h>
#include <simrad_ek80/subscription_manager.h>
#include <simrad_ek80/sample.h>
#include <simrad_ek80/parameter_group.h>

namespace simrad
{

class Channel
{
public:
  typedef std::shared_ptr<Channel> Ptr;
  typedef std::vector<Ptr> List;

  Channel(ParameterManager::Ptr& parameter_manager, SubscriptionManager::Ptr &subscription_manager, const ParameterGroup::Map& parameter_group, const std::string& name);
        
  ~Channel();

  std::string getName();

  //BottomDetectionSubscription::Ptr& getBottomDetection();
        
  //TargetDetectionSubscription::Ptr& getTargetDetection();
        
  SampleSubscription::Ptr& getSamples(float range = 250.0, std::string sample_data_type = "Power");
        
  void updateSubscription(Subscription::Ptr& subscription);

  const ParameterGroup::Map& getParameters();
private:
  void subscribe();

  ParameterManager::Ptr parameter_manager_;
  SubscriptionManager::Ptr subscription_manager_;

  //BottomDetectionSubscription::Ptr bottomDetection;
  //TargetDetectionSubscription::Ptr targetDetection;
  std::map<std::string, SampleSubscription::Ptr> samples_;

  ParameterGroup::Map parameters_;

  std::string name_;
  
  bool subscribed_;

};
} // namespace simrad

#endif
