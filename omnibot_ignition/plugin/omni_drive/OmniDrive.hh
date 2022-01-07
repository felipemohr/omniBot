#ifndef SYSTEM_PLUGIN_OMNIDRIVE_HH_
#define SYSTEM_PLUGIN_OMNIDRIVE_HH_

#include <memory>

#include <ignition/gazebo/System.hh>

#include <ignition/transport/Node.hh>


using namespace ignition;
using namespace gazebo;

namespace ignition
{
namespace gazebo
{

namespace omni_drive
{
  class OmniDrive:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    public:
      OmniDrive();
      ~OmniDrive();

      void Configure(const Entity &_entity,
                     const std::shared_ptr<const sdf::Element> &_sdf,
                     EntityComponentManager &_ecm,
                     EventManager &_eventMgr);

      void PreUpdate(const UpdateInfo &_info,
                     EntityComponentManager &_ecm);

      void PostUpdate(const UpdateInfo &_info,
                      const EntityComponentManager &_ecm);


    protected:
      /**
       * @brief Callback for velocity subscription.
       * 
       * @param _msg Velocity message.
       */
      void OnCmdVel(const ignition::msgs::Twist &_msg);

      /**
       * @brief Callback for enable/disable subscription.
       * 
       * @param _msg Boolean message.
       */
      void OnEnable(const ignition::msgs::Boolean &_msg);

      /**
       * @brief Update linear and angular velocities.
       * 
       * @param _info System update information.
       * @param _ecm  The EntityComponentManager of the given simulation instance.
       */
      void UpdateVelocity(const ignition::gazebo::UpdateInfo &_info,
                          const ignition::gazebo::EntityComponentManager &_ecm);

      /** @brief Ignition communication node **/
      transport::Node node;

      /** @brief Entity of the front left joint **/
      Entity frontLeftJoint;

      /** @brief Entity of the back left joint **/
      Entity backLeftJoint;

      /** @brief Entity of the front right joint **/
      Entity frontRightJoint;

      /** @brief Entity of the back right joint **/
      Entity backRightJoint;

      /** @brief Name of the front left joint **/
      std::string frontLeftJointName;

      /** @brief Name of the back left joint **/
      std::string backLeftJointName;

      /** @brief Name of the front right joint **/
      std::string frontRightJointName;

      /** @brief Name of the back right joint **/
      std::string backRightJointName;

      /** @brief Calculated speed of front left joint **/
      double frontLeftJointSpeed{0};

      /** @brief Calculated speed of back left joint **/
      double backLeftJointSpeed{0};

      /** @brief Calculated speed of front right joint **/
      double frontRightJointSpeed{0};

      /** @brief Calculated speed of back right joint **/
      double backRightJointSpeed{0};

      /** @brief Distance between right and left wheels **/
      double wheelRightLeftSeparation{1.0};

      /** @brief Distance between front and back wheels **/
      double wheelFrontBackSeparation{1.0};

      /** @brief Wheel radius **/
      double wheelRadius{0.2};



  };


} // namespace omni_drive

} // namespace gazebo
} // namespace ignition


# endif // SYSTEM_PLUGIN_OMNIDRIVE_HH_
