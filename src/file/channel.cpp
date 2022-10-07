#include <simrad_ek80/file/channel.h>
#include <simrad_ek80/utilities.h>

namespace simrad
{
namespace file
{

void Channel::configure(boost::property_tree::ptree tree)
{
  auto transducer = tree.get_child_optional("Transducer");
  if(transducer.is_initialized())
    transducer_ = transducer->get<std::string>("<xmlattr>.TransducerName");
}

void Channel::updateParameters(boost::property_tree::ptree tree)
{
  auto pulse_form = tree.get<int>("<xmlattr>.PulseForm");
  if(pulse_form != 0)
    throw Exception("Unsupported pulse form: "+std::to_string(pulse_form));

  frequency_ = tree.get<double>("<xmlattr>.Frequency");
  sample_interval_ = tree.get<double>("<xmlattr>.SampleInterval");
  transmit_power_ = tree.get<double>("<xmlattr>.TransmitPower");
  sound_speed_ = tree.get<double>("<xmlattr>.SoundVelocity");
}


void Channel::setTransceiver(std::string transceiver)
{
  transceiver_ = transceiver;
}

const std::string& Channel::transceiver() const
{
  return transceiver_;
}

const std::string& Channel::transducer() const
{
  return transducer_;
}

const double& Channel::frequency() const
{
  return frequency_;
}

const double& Channel::sample_interval() const
{
  return sample_interval_;
}

const double& Channel::transmit_power() const
{
  return transmit_power_;
}

const double& Channel::sound_speed() const
{
  return sound_speed_;
}

} // namepsace file
} // namespace simrad
