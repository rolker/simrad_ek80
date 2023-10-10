#include <simrad_ek80/udp/channel.h>

namespace simrad
{

Channel::Channel(ParameterManager::Ptr& parameter_manager, SubscriptionManager::Ptr &subscription_manager, const std::string& name)
  :parameter_manager_(parameter_manager), subscription_manager_(subscription_manager), name_(name)
{
}

Channel::~Channel()
{
}

const std::string& Channel::name() const
{
  return name_;
}

std::string Channel::topicName() const
{
  return channelNameToTopicName(name_);
}


// BottomDetectionSubscription::Ptr& Channel::getBottomDetection()
// {
//   if(!bottom_detection_)
//   {
//     subscribe();
//     bottom_detection_ = BottomDetectionSubscription::Ptr(new BottomDetectionSubscription(name_, parameters)));
//     bottom_detection_->setParameter("UpperDetectorLimit","5.0");
//     Subscription::Ptr sp(bottom_detection_);
//     subscription_manager_->subscribe(sp);
//   }
//   return bottom_detection_;
// }

// TargetDetectionSubscription::Ptr& Channel::getTargetDetection()
// {
//   if(!target_detection_)
//   {
//     subscribe();
//     target_detection_ = TargetDetectionSubscription::Ptr(new TargetDetectionSubscription(name_, parameters_));
//     Subscription::Ptr sp(target_detection_);
//     subscription_manager_->subscribe(sp);
//   }
//   return target_detection_;
// }

SampleSubscription::Ptr Channel::subscribe(int range, std::string sample_data_type, int start_range)
{
  subscribe_parameters();
  SampleSubscription::Ptr subscription = std::make_shared<SampleSubscription>(*this, range, sample_data_type, start_range);
  subscription_manager_->subscribe(subscription);
  return subscription;
}

std::shared_ptr<EchogramSubscription> Channel::subscribeToEchogram(float range, float range_start, float bin_size)
{
  subscribe_parameters();
  std::shared_ptr<EchogramSubscription> subscription = std::make_shared<EchogramSubscription>(*this, range, range_start, bin_size);
  subscription_manager_->subscribe(subscription);
  return subscription;
}


void Channel::updateSubscription(Subscription::Ptr& subscription)
{
  if(subscription)
    subscription_manager_->update(subscription);
}

std::shared_ptr<Parameter> Channel::getParameter(std::string name) const
{
  return parameter_manager_->get("TransceiverMgr/"+name_+"/"+name);
}

void Channel::subscribe_parameters()
{
  if(!subscribed_)
  {
    parameter_manager_->subscribe("TransceiverMgr/"+name_+"/Frequency");
    parameter_manager_->subscribe("TransceiverMgr/"+name_+"/BeamType");
    parameter_manager_->subscribe("TransceiverMgr/"+name_+"/SampleInterval");
    parameter_manager_->subscribe("TransceiverMgr/"+name_+"/TransmitPower");
    parameter_manager_->subscribe("TransceiverMgr/"+name_+"/AbsorptionCoefficient");
    parameter_manager_->subscribe("TransceiverMgr/"+name_+"/SoundVelocity");
    parameter_manager_->subscribe("TransceiverMgr/"+name_+"/TransducerName");
    parameter_manager_->subscribe("TransceiverMgr/"+name_+"/EquivalentBeamAngle");
    parameter_manager_->subscribe("TransceiverMgr/"+name_+"/AngleSensitivityAlongship");
    parameter_manager_->subscribe("TransceiverMgr/"+name_+"/AngleSensitivityAthwartship");
    parameter_manager_->subscribe("TransceiverMgr/"+name_+"/BeamWidthAlongship");
    parameter_manager_->subscribe("TransceiverMgr/"+name_+"/BeamWidthAthwartship");
    parameter_manager_->subscribe("TransceiverMgr/"+name_+"/DirectionX");
    parameter_manager_->subscribe("TransceiverMgr/"+name_+"/DirectionY");
    parameter_manager_->subscribe("TransceiverMgr/"+name_+"/Gain");
    parameter_manager_->subscribe("TransceiverMgr/"+name_+"/PulseLength");
    subscribed_ = true;
  }
}


} // namespace simrad
