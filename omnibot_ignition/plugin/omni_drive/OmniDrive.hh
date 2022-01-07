#ifndef SYSTEM_PLUGIN_OMNIDRIVE_HH_
#define SYSTEM_PLUGIN_OMNIDRIVE_HH_

#include <ignition/gazebo/System.hh>

namespace omni_drive
{

  class OmniDrive:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemPostUpdate
  {
    public:
      void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                      const ignition::gazebo::EntityComponentManager &_ecm) override;
  };

}


# endif // SYSTEM_PLUGIN_OMNIDRIVE_HH_
