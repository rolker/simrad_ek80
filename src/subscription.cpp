#include <simrad_ek80/subscription.h>

namespace simrad

{

Subscription::Subscription(const Subscription&)
{

}
  
Subscription::Subscription()
{

}

Subscription::Subscription(const std::string& type, const std::string& channel, const ParameterGroup::Map& ping_parameters)
  :type_(type),channel_(channel),ping_parameters_(ping_parameters)
{
}

Subscription::~Subscription()
{

}

void Subscription::setParameter(std::string key, std::string value)
{
  parameters_[key] = value;
}

void Subscription::setID(int id)
{
  id_ = id;
}

int Subscription::getID() const
{
  return id_;
}

std::string Subscription::subscribeString()
{
  std::string ret = type_+",ChannelID="+channel_;
    for(auto parameter: parameters_)
      ret += ',' + parameter.first + "=" + parameter.second;
  return ret;
}

const ParameterGroup::Map& Subscription::getPingParameters() const
{
  return ping_parameters_;
}

} // namespace simrad
