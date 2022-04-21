#ifndef SIMRAD_EK80_PARAMETER_GROUP_H
#define SIMRAD_EK80_PARAMETER_GROUP_H

#include <simrad_ek80/parameter_manager.h>

namespace simrad
{

class ParameterGroup
{
public:
  typedef std::shared_ptr<ParameterGroup> Ptr;
  typedef std::map<std::string, ParameterGroup::Ptr> Map;

  ParameterGroup(std::shared_ptr<ParameterManager>& parameter_manager);

  bool add(const std::string& label, const std::string& parameter, bool use_default = true);

  void update(TimePoint t);
                
  Parameter::Ptr operator[](const std::string& key);

  void addCallback(std::function<void(TimePoint)> callback);

  std::map<std::string, Parameter::Ptr> parameters();
private:                
  ParameterManager::Ptr manager_;
  TimePoint last_time_;

  std::map<std::string, Parameter::Ptr> parameters_;
  std::mutex parameters_mutex_;
  
  std::vector<std::function<void(TimePoint)> > callbacks_;
  std::mutex callbacks_mutex_;
};

} // namespace simrad

#endif
