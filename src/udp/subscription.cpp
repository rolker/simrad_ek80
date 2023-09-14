#include <simrad_ek80/udp/subscription.h>

namespace simrad

{

Subscription::Subscription(const Subscription&)
{

}
  
Subscription::Subscription()
{

}

Subscription::Subscription(const std::string& type, const std::string& channel)
  :type_(type),channel_(channel)
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

} // namespace simrad
