#ifndef SIMRAD_EK80_FILE_TRANSDUCER_H
#define SIMRAD_EK80_FILE_TRANSDUCER_H

#include <string>
#include <map>
#include <boost/property_tree/xml_parser.hpp>

namespace simrad
{

namespace file
{

class Transducer
{
public:
  void configure(boost::property_tree::ptree tree);
  double impedance(double frequency) const;

  const double& offset_x() const;
  const double& offset_y() const;
  const double& offset_z() const;
  const double& alpha_x() const;
  const double& alpha_y() const;
  const double& alpha_z() const;

  const double& beamwidth_alongship() const;
  const double& beamwidth_athwartship() const;

private:
  double offset_x_;
  double offset_y_;
  double offset_z_;
  double alpha_x_;
  double alpha_y_;
  double alpha_z_;

  double beamwidth_alongship_;
  double beamwidth_athwartship_;

  std::map<double, double> impedance_by_frequency_;
};

} // namespace file
} // namespace simrad

#endif
