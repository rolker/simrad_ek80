#ifndef SIMRAD_EK80_PLATFORM_H
#define SIMRAD_EK80_PLATFORM_H

#include <simrad_ek80/parameter_manager.h>
#include <simrad_ek80/parameter_group.h>

namespace simrad
{

/// Keeps track of data pertaining to the platform.
/// Data includes navigation, which is time varying
/// and offsets, which should be static.
class Platform
{
public:
  typedef std::shared_ptr<Platform> Ptr;

  Platform(ParameterManager::Ptr& parameter_manager);
    
  const ParameterGroup::Map& getParameters();

private:
  ParameterManager::Ptr parameter_manager_;
  ParameterGroup::Map parameters_;
  bool subscribed_ = false;

  void subscribe();
  
};

} // namespace simrad

#endif
