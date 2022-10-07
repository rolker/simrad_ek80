#ifndef SIMRAD_EK80_FILE_CHANNEL_H
#define SIMRAD_EK80_FILE_CHANNEL_H

#include <string>
#include <boost/property_tree/xml_parser.hpp>

namespace simrad
{

namespace file
{

class Channel
{
public:
  void configure(boost::property_tree::ptree tree);
  void updateParameters(boost::property_tree::ptree tree);

  void setTransceiver(std::string transceiver);
  const std::string& transceiver() const;
  const std::string& transducer() const;
  const double& frequency() const;
  const double& sample_interval() const;
  const double& transmit_power() const;
  const double& sound_speed() const;
private:
  std::string transducer_;
  std::string transceiver_;
  
  double frequency_ = 0.0;
  double sample_interval_;
  double transmit_power_;
  double sound_speed_;
};

} // namespace file
} // namespace simrad

#endif
