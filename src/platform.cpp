#include <simrad_ek80/platform.h>

namespace simrad
{

Platform::Platform(ParameterManager::Ptr& parameter_manager)
  :parameter_manager_(parameter_manager)
{

}

const ParameterGroup::Map& Platform::getParameters()
{
  subscribe();
  return parameters_;
}

void Platform::subscribe()
{
  if(!subscribed_)
  {
    parameters_["position"] = ParameterGroup::Ptr(new ParameterGroup(parameter_manager_));
    parameters_["position"]->add("latitude","OwnShip/Latitude",false);
    parameters_["position"]->add("longitude","OwnShip/Longitude",false);
        
    parameters_["heading"] = ParameterGroup::Ptr(new ParameterGroup(parameter_manager_));
    parameters_["heading"]->add("heading","OwnShip/Heading",false);
        
    parameters_["attitude"] = ParameterGroup::Ptr(new ParameterGroup(parameter_manager_));
    parameters_["attitude"]->add("roll","OwnShip/Roll");
    parameters_["attitude"]->add("pitch","OwnShip/Pitch");
    parameters_["attitude"]->add("heave","OwnShip/Heave");
    parameters_["attitude"]->add("motion", "OwnShip/MotionData");

    parameters_["velocity"] = ParameterGroup::Ptr(new ParameterGroup(parameter_manager_));
    parameters_["velocity"]->add("sog", "OwnShip/Speed");
    parameters_["velocity"]->add("cog", "OwnShip/Course");


    subscribed_ = true;
  }
}

} // namespace simrad
