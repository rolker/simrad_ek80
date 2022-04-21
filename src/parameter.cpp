#include <simrad_ek80/parameter.h>

namespace simrad
{

Parameter::Parameter(Info info, bool use_default, bool value_is_time)
  :info_(info), use_default_(use_default), value_is_time_(value_is_time)
{

}

void Parameter::update(const ParameterUpdate& update)
{
  auto t = update.time;
  if(value_is_time_)
    t = fromSimradTime(update);
  {
    std::lock_guard<std::mutex> lock(updates_mutex_);
    updates_[t] = update.value;
  }
  std::lock_guard<std::mutex> lock(callbacks_mutex_);
  for(auto callback: callbacks_)
    callback(t);
}

const Parameter::Info& Parameter::getInfo() const
{
  return info_;
}

TimePoint Parameter::getLatestTime()
{
  std::lock_guard<std::mutex> lock(updates_mutex_);
  if(!updates_.empty())
    return updates_.rbegin()->first;
  return TimePoint();
}

void Parameter::addCallback(std::function<void(TimePoint)> callback)
{
  std::lock_guard<std::mutex> lock(callbacks_mutex_);
  callbacks_.push_back(callback);
}

} // namespace simrad
