#ifndef SIMRAD_EK80_SUBSCRIPTION_H
#define SIMRAD_EK80_SUBSCRIPTION_H

#include <simrad_ek80/parameter_group.h>
#include <mutex>
#include <memory>
#include <vector>


namespace simrad
{

class Subscription
{
public:
  typedef std::shared_ptr<Subscription> Ptr;
  typedef std::weak_ptr<Subscription> WeakPtr;

  Subscription(const std::string& type, const std::string& channel, const ParameterGroup::Map& ping_parameters);
    
  virtual ~Subscription();

  void setParameter(std::string key, std::string value);
    
  void setID(int id);
  int getID() const;

  std::string subscribeString();
    
  const ParameterGroup::Map& getPingParameters() const;

  virtual void addData(std::shared_ptr<std::vector<unsigned char> >& data) = 0;

protected:
  std::string type_;
  std::string channel_;
  std::map<std::string,std::string> parameters_;
  int id_;

  ParameterGroup::Map ping_parameters_;

private:
  Subscription(const Subscription&);
  Subscription();


};


} // namespace simrad

#endif

