#include <string>
#include <ignition/common/Console.hh>

#include <ignition/plugin/Register.hh>

#include "OmniDrive.hh"

IGNITION_ADD_PLUGIN(
  omni_drive::OmniDrive,
  ignition::gazebo::System,
  omni_drive::OmniDrive::ISystemPostUpdate
)

using namespace omni_drive;

void OmniDrive::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                           const ignition::gazebo::EntityComponentManager &/*_ecm*/)
{
  std::string msg = "Hello, world! Simulation is ";
  if (!_info.paused)
    msg += "not ";
  msg += "paused.";

  ignmsg << msg << std::endl;
  // std::cout << msg << std::endl;

}
