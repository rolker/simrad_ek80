#include <simrad_ek80/file/transducer.h>

namespace simrad
{
namespace file
{

void Transducer::configure(boost::property_tree::ptree tree)
{
  auto offset_x = tree.get_optional<double>("<xmlattr>.TransducerOffsetX");
  if(offset_x.is_initialized())
    offset_x_ = *offset_x;
  auto offset_y = tree.get_optional<double>("<xmlattr>.TransducerOffsetY");
  if(offset_y.is_initialized())
    offset_y_ = *offset_y;
  auto offset_z = tree.get_optional<double>("<xmlattr>.TransducerOffsetZ");
  if(offset_z.is_initialized())
    offset_z_ = *offset_z;

  auto alpha_x = tree.get_optional<double>("<xmlattr>.TransducerAlphaX");
  if(alpha_x.is_initialized())
    alpha_x_ = *alpha_x;
  auto alpha_y = tree.get_optional<double>("<xmlattr>.TransducerAlphaY");
  if(alpha_y.is_initialized())
    alpha_y_ = *alpha_y;
  auto alpha_z = tree.get_optional<double>("<xmlattr>.TransducerAlphaZ");
  if(alpha_z.is_initialized())
    alpha_z_ = *alpha_z;

  auto beamwidth_alongship = tree.get_optional<double>("<xmlattr>.BeamWidthAlongship"); 
  if(beamwidth_alongship.is_initialized())
    beamwidth_alongship_ = *beamwidth_alongship;

  auto beamwidth_athwartship = tree.get_optional<double>("<xmlattr>.BeamWidthAthwartship");
  if(beamwidth_athwartship.is_initialized())
    beamwidth_athwartship_ = *beamwidth_athwartship;

  for(auto fp: tree)
  {
    if(fp.first == "FrequencyPar")
    {
      impedance_by_frequency_[fp.second.get<double>("<xmlattr>.Frequency")] = fp.second.get<double>("<xmlattr>.Impedance");
    }
  }
}

double Transducer::impedance(double frequency) const
{
  auto i = impedance_by_frequency_.lower_bound(frequency);
  if(i == impedance_by_frequency_.end())
    return 0.0;
  if(i->first == frequency)
    return i->second;
  if(i == impedance_by_frequency_.begin())
    return 0.0;
  auto prev = i;
  prev--;
  double p = (frequency-prev->first)/(i->first-prev->first);
  return (i->second*p)+(prev->second*(1.0-p));
}

const double& Transducer::offset_x() const
{
  return offset_x_;
}

const double& Transducer::offset_y() const
{
  return offset_y_;
}

const double& Transducer::offset_z() const
{
  return offset_z_;
}

const double& Transducer::alpha_x() const
{
  return alpha_x_;
}

const double& Transducer::alpha_y() const
{
  return alpha_y_;
}

const double& Transducer::alpha_z() const
{
  return alpha_z_;
}

const double& Transducer::beamwidth_alongship() const
{
  return beamwidth_alongship_;
}

const double& Transducer::beamwidth_athwartship() const
{
  return beamwidth_athwartship_;
}

} // namespace file
} // namespace simrad
