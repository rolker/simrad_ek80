#include <simrad_ek80/parameter_group.h>
#include <iostream>

namespace simrad
{

ParameterGroup::ParameterGroup(std::shared_ptr<ParameterManager>& parameter_manager)
  :manager_(parameter_manager)
{
}

bool ParameterGroup::add(const std::string& label, const std::string& parameter, bool use_default)
{
  std::lock_guard<std::mutex> lock(parameters_mutex_);
  try
  {
    parameters_[label] = manager_->subscribe(parameter, use_default);
  }
  catch(InvalidParameterException e)
  {
    std::cerr << "ParameterGroup::add " << e.what() << std::endl;
    return false;
  }
  parameters_[label]->addCallback([this](TimePoint time){this->update(time);});
  return true;
}

void ParameterGroup::update(TimePoint t)
{
  if(last_time_ == TimePoint() && t != TimePoint())
    last_time_ = t;
      
  if(t > last_time_)
  {
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    for(auto callback: callbacks_)
      callback(t);
    last_time_ = t;
  }
}

Parameter::Ptr ParameterGroup::operator[](const std::string& key)
{
  std::lock_guard<std::mutex> lock(parameters_mutex_);
  if(parameters_.find(key) != parameters_.end())
    return parameters_[key];
  return Parameter::Ptr();
}

void ParameterGroup::addCallback(std::function<void(TimePoint)> callback)
{
  std::lock_guard<std::mutex> lock(callbacks_mutex_);
  callbacks_.push_back(callback);
}

std::map<std::string, Parameter::Ptr> ParameterGroup::parameters()
{
  std::lock_guard<std::mutex> lock(parameters_mutex_);
  return parameters_;
}

} // namespace simrad
