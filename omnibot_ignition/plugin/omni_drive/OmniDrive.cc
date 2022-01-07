#include <string>
#include <ignition/common/Console.hh>

#include <ignition/plugin/Register.hh>

#include "OmniDrive.hh"

using namespace ignition;
using namespace gazebo;
using namespace omni_drive;


// /// \brief Velocity command.
// struct Commands
// {
//   /// \brief Linear velocity in X-axis.
//   double lin_x;

//   /// \brief Linear velocity in Y-axis.
//   double lin_y;

//   /// \brief Angular velocity in Z-axis.
//   double ang;

//   Commands() : lin_x(0.0), lin_y(0.0), ang(0.0) {}
// };


void OmniDrive::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                           const ignition::gazebo::EntityComponentManager &/*_ecm*/)
{
  std::string msg = "Hello, world! Simulation is ";
  if (!_info.paused)
    msg += "not ";
  msg += "paused.";

  ignmsg << msg << std::endl;
  // std::cout << this->frontLeftJointSpeed << std::endl;

}


IGNITION_ADD_PLUGIN(OmniDrive,
                    ignition::gazebo::System,
                    OmniDrive::ISystemConfigure,
                    OmniDrive::ISystemPreUpdate,
                    OmniDrive::ISystemPostUpdate
)

// IGNITION_ADD_PLUGIN_ALIAS(OmniDrive, "ignition::gazebo::omni_drive::OmniDrive")
