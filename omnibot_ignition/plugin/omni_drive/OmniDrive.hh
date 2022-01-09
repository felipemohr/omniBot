#ifndef IGNITION_GAZEBO_SYSTEMS_OMNIDRIVE_HH_
#define IGNITION_GAZEBO_SYSTEMS_OMNIDRIVE_HH_

#include <memory>

#include <ignition/gazebo/System.hh>

#include <ignition/msgs/odometry.pb.h>

#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <ignition/common/Profiler.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/SpeedLimiter.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace omni_drive
{

  class OmniDrivePrivate
  {
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




  class OmniDrive
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
  {
    /** @brief Constructor **/
    public: OmniDrive();

    /** @brief Destructor **/
    public: ~OmniDrive() override = default;

    /** Documentation inherited **/
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    /** Documentation inherited **/
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    /** Documentation inherited **/
    public: void PostUpdate(
                const UpdateInfo &_info,
                const EntityComponentManager &_ecm) override;

    /** Private data pointer **/
    private: std::unique_ptr<OmniDrivePrivate> dataPtr;
  };
  }
}
}
}

#endif
