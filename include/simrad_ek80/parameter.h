#ifndef SIMRAD_EK80_PARAMETER_H
#define SIMRAD_EK80_PARAMETER_H

#include <simrad_ek80/utilities.h>
#include <simrad_ek80/parameter_update.h>
#include <queue>
#include <mutex>
#include <memory>
#include <functional>

namespace simrad
{

class Parameter
{
public:
  typedef std::shared_ptr<Parameter> Ptr;
  typedef std::weak_ptr<Parameter> WeakPtr;

  struct Info
  {
    std::string name;
    std::string description;
    int type;
    std::string defaultValue;
  };

  Parameter(Info info, bool use_default = true, bool value_is_time = false);

  void update(const ParameterUpdate& update);
                
  template<typename T> T get(T default_value, TimePoint time = TimePoint());
  template<typename T> bool isType();

  const Info& getInfo() const;

  TimePoint getLatestTime();

  void addCallback(std::function<void(TimePoint)> callback);
                
private:
  Info info_;
  bool use_default_;
  bool value_is_time_;

  std::map<TimePoint, std::vector<char> > updates_;
  std::mutex updates_mutex_;

  std::vector<std::function<void(TimePoint)> > callbacks_;
  std::mutex callbacks_mutex_;
};

template<typename T> inline T Parameter::get(T default_value, TimePoint time)
{
  if(!isType<T>())
    throw(Exception("Type mismatch: " + std::to_string(info_.type)));
  {
    std::lock_guard<std::mutex> lock(updates_mutex_);
    if(!updates_.empty())
    {
      if(time != TimePoint())
      {
        if(updates_.begin()->first <= time)
        {
          if (updates_.count(time))
            return *reinterpret_cast<T const*>(updates_[time].data());
          auto i = updates_.upper_bound(time);
          if (i != updates_.end())
          {
            --i;
            return *reinterpret_cast<T const*>(i->second.data());
          }
          return *reinterpret_cast<T const*>(updates_.rbegin()->second.data());
        }
      }
      else
        return *reinterpret_cast<T const*>(updates_.rbegin()->second.data());
    }
  }
  if(use_default_ && !info_.defaultValue.empty())
  {
    std::stringstream ss(info_.defaultValue);
    T ret;
    ss >> ret;
    return ret;
  }
  return default_value;
}


template<> inline bool Parameter::isType<short>()
{
  return (info_.type==2);
}

template<> inline bool Parameter::isType<int>()
{
  return (info_.type==3);
}

template<> inline bool Parameter::isType<int64_t>()
{
  return (info_.type==20);
}

template<> inline bool Parameter::isType<float>()
{
  return (info_.type==4);
}

template<> inline bool Parameter::isType<double>()
{
  return (info_.type==5);
}

template<> inline bool Parameter::isType<uint64_t>()
{
  return (info_.type==6);
}

template<> inline bool Parameter::isType<bool>()
{
  return (info_.type==11);
}

template<> inline bool Parameter::isType<std::string>()
{
  return (info_.type==8);
}

template<> inline bool Parameter::isType<std::vector<double> >()
{
  return (info_.type==0x2005);
}

template<> inline bool Parameter::isType<std::vector<std::string> >()
{
  return (info_.type==0x2008);
}

} // namespace simrad


#endif
