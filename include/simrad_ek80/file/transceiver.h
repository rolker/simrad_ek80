#ifndef SIMRAD_EK80_FILE_TRANSCEIVER_H
#define SIMRAD_EK80_FILE_TRANSCEIVER_H

#include <string>
#include <boost/property_tree/xml_parser.hpp>

namespace simrad
{

namespace file
{

class Transceiver
{
public:
  void configure(boost::property_tree::ptree tree);
  const double& impedance() const;
  const double& rx_sample_frequency() const;
private:
  double impedance_;
  double rx_sample_frequency_;
};

} // namespace file
} // namespace simrad

#endif
