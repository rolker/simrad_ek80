#include <simrad_ek80/file/transceiver.h>
#include <iostream>

namespace simrad
{
namespace file
{

void Transceiver::configure(boost::property_tree::ptree tree)
{
  impedance_ = tree.get<double>("<xmlattr>.Impedance");
  rx_sample_frequency_ = tree.get<double>("<xmlattr>.RxSampleFrequency");
}

const double& Transceiver::impedance() const
{
  return impedance_;
}

const double& Transceiver::rx_sample_frequency() const
{
  return rx_sample_frequency_;
}

} // namepsace file
} // namespace simrad
