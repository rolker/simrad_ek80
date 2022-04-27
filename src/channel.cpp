#include <simrad_ek80/channel.h>

namespace simrad
{

Channel::Channel(ParameterManager::Ptr& parameter_manager, SubscriptionManager::Ptr &subscription_manager, const std::string& name)
  :parameter_manager_(parameter_manager), subscription_manager_(subscription_manager), name_(name)
{
}

Channel::~Channel()
{
}

std::string Channel::getName()
{
  return name_;
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

void Channel::subscribe(SampleSubscription::Ptr subscription)
{
  subscribe();
  Subscription::Ptr sp(subscription);
  subscription_manager_->subscribe(sp);
}

void Channel::updateSubscription(Subscription::Ptr& subscription)
{
  if(subscription)
    subscription_manager_->update(subscription);
}

std::shared_ptr<Parameter> Channel::getParameter(std::string name)
{
  return parameter_manager_->get("TransceiverMgr/"+name_+"/"+name);
}

void Channel::subscribe()
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
