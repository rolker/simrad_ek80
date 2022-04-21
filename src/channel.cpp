#include <simrad_ek80/channel.h>

namespace simrad
{

Channel::Channel(ParameterManager::Ptr& parameter_manager, SubscriptionManager::Ptr &subscription_manager, const ParameterGroup::Map& parameter_group, const std::string& name)
  :parameter_manager_(parameter_manager), subscription_manager_(subscription_manager), parameters_(parameter_group), name_(name)
{
  parameters_["channel"] = ParameterGroup::Ptr(new ParameterGroup(parameter_manager));
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

SampleSubscription::Ptr& Channel::getSamples(float range, std::string sample_data_type)
{
  if(samples_.find(sample_data_type) == samples_.end())
  {
    subscribe();
    samples_[sample_data_type] = SampleSubscription::Ptr(new SampleSubscription(name_, parameters_, sample_data_type));
    samples_[sample_data_type]->setParameter("SampleDataType", sample_data_type);
    samples_[sample_data_type]->setParameter("Range", std::to_string(range));
    
    //samples->SetParameter("SampleDataType","Sv");
    Subscription::Ptr sp(samples_[sample_data_type]);
    subscription_manager_->subscribe(sp);
  }
  return samples_[sample_data_type];
}

void Channel::updateSubscription(Subscription::Ptr& subscription)
{
  if(subscription)
    subscription_manager_->update(subscription);
}

const ParameterGroup::Map& Channel::getParameters()
{
  subscribe();
  return parameters_;
}

void Channel::subscribe()
{
  if(!subscribed_)
  {
    parameters_["channel"]->add("frequency","TransceiverMgr/"+name_+"/Frequency");
    parameters_["channel"]->add("beamType","TransceiverMgr/"+name_+"/BeamType");
    parameters_["channel"]->add("sampleInterval","TransceiverMgr/"+name_+"/SampleInterval");
    parameters_["channel"]->add("transmitPower","TransceiverMgr/"+name_+"/TransmitPower");
    parameters_["channel"]->add("absorptionCoefficient","TransceiverMgr/"+name_+"/AbsorptionCoefficient");
    parameters_["channel"]->add("soundSpeed","TransceiverMgr/"+name_+"/SoundVelocity");
    parameters_["channel"]->add("transducerName","TransceiverMgr/"+name_+"/TransducerName");
    parameters_["channel"]->add("equivalentBeamAngle","TransceiverMgr/"+name_+"/EquivalentBeamAngle");
    parameters_["channel"]->add("angleSensitivityY","TransceiverMgr/"+name_+"/AngleSensitivityAlongship");
    parameters_["channel"]->add("angleSensitivityX","TransceiverMgr/"+name_+"/AngleSensitivityAthwartship");
    parameters_["channel"]->add("beamWidthY","TransceiverMgr/"+name_+"/BeamWidthAlongship");
    parameters_["channel"]->add("beamWidthX","TransceiverMgr/"+name_+"/BeamWidthAthwartship");
    parameters_["channel"]->add("directionX","TransceiverMgr/"+name_+"/DirectionX");
    parameters_["channel"]->add("directionY","TransceiverMgr/"+name_+"/DirectionY");
    parameters_["channel"]->add("gain","TransceiverMgr/"+name_+"/Gain");
    parameters_["channel"]->add("pulseLength","TransceiverMgr/"+name_+"/PulseLength");
    //parameters_["channel"]->add("transducerDepth","TransceiverMgr/"+name_+"/TransducerDepth");
    subscribed_ = true;
  }
}


} // namespace simrad
