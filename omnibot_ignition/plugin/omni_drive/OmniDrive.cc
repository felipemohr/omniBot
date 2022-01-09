#include "OmniDrive.hh"

using namespace ignition;
using namespace gazebo;
using namespace omni_drive;

/** @brief Velocity command. **/
struct Commands
{
  /** @brief Linear velocity in X-axis. **/
  double lin_x;

  /** @brief Linear velocity in Y-axis. **/
  double lin_y;

  /** @brief Angular velocity in Z-axis. **/
  double ang;

  Commands() : lin_x(0.0), lin_y(0.0), ang(0.0) {}
};

//////////////////////////////////////////////////
OmniDrive::OmniDrive()
  : dataPtr(std::make_unique<OmniDrivePrivate>())
{
}

//////////////////////////////////////////////////
void OmniDrive::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
 
}

//////////////////////////////////////////////////
void OmniDrive::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("OmniDrive::PreUpdate");

}

//////////////////////////////////////////////////
void OmniDrive::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("OmniDrive::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;

}


IGNITION_ADD_PLUGIN(OmniDrive,
                    ignition::gazebo::System,
                    OmniDrive::ISystemConfigure,
                    OmniDrive::ISystemPreUpdate,
                    OmniDrive::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(OmniDrive, "omni_drive::OmniDrive")
