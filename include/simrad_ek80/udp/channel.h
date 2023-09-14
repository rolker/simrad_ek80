#ifndef SIMRAD_EK80_CHANNEL_H
#define SIMRAD_EK80_CHANNEL_H

#include <simrad_ek80/udp/parameter_manager.h>
#include <simrad_ek80/udp/subscription_manager.h>
#include <simrad_ek80/udp/sample.h>

namespace simrad
{

class Channel
{
public:
  typedef std::shared_ptr<Channel> Ptr;
  typedef std::vector<Ptr> List;

  Channel(ParameterManager::Ptr& parameter_manager, SubscriptionManager::Ptr &subscription_manager, const std::string& name);
        
  ~Channel();

  std::string getName();

  //BottomDetectionSubscription::Ptr& getBottomDetection();
        
  //TargetDetectionSubscription::Ptr& getTargetDetection();

        
  void updateSubscription(Subscription::Ptr& subscription);

  std::shared_ptr<Parameter> getParameter(std::string name);

private:
  void subscribe(SampleSubscription::Ptr subscription);
  void subscribe();

  friend class SampleSubscription;

  ParameterManager::Ptr parameter_manager_;
  SubscriptionManager::Ptr subscription_manager_;

  //BottomDetectionSubscription::Ptr bottomDetection;
  //TargetDetectionSubscription::Ptr targetDetection;
  std::map<std::string, SampleSubscription::Ptr> samples_;

  std::string name_;
  
  bool subscribed_;
};

} // namespace simrad

#endif
