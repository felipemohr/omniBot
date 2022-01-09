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
    public:
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

      /** @brief Entity of the rear left joint **/
      Entity rearLeftJoint;

      /** @brief Entity of the front right joint **/
      Entity frontRightJoint;

      /** @brief Entity of the rear right joint **/
      Entity rearRightJoint;

      /** @brief Name of the front left joint **/
      std::string frontLeftJointName;

      /** @brief Name of the rear left joint **/
      std::string rearLeftJointName;

      /** @brief Name of the front right joint **/
      std::string frontRightJointName;

      /** @brief Name of the rear right joint **/
      std::string rearRightJointName;

      /** @brief Calculated speed of front left joint **/
      double frontLeftJointSpeed{0};

      /** @brief Calculated speed of rear left joint **/
      double rearLeftJointSpeed{0};

      /** @brief Calculated speed of front right joint **/
      double frontRightJointSpeed{0};

      /** @brief Calculated speed of rear right joint **/
      double rearRightJointSpeed{0};

      /** @brief Distance between right and left wheels **/
      double wheelRightLeftSeparation{1.0};

      /** @brief Distance between front and rear wheels **/
      double wheelFrontRearSeparation{1.0};

      /** @brief Wheel radius **/
      double wheelRadius{0.2};

      /** @brief Model interface **/
      Model model{kNullEntity};

      /** @brief The model's canonical link **/
      Link canonicalLink{kNullEntity};

      /** @brief Omni drive tf message publisher **/
      transport::Node::Publisher tfPub;

      /** @brief Linear velocity limiter **/
      std::unique_ptr<ignition::math::SpeedLimiter> limiterLin;

      /** @brief Angular velocity limiter **/
      std::unique_ptr<ignition::math::SpeedLimiter> limiterAng;

      /** @brief Previous control command **/
      Commands last0Cmd;

      /** @brief Previous control command to last0Cmd **/
      Commands last1Cmd;

      /** @brief Last target velocity requested **/
      msgs::Twist targetVel;

      /** @brief Enable/disable state of the controller **/
      bool enabled;

      /** @brief A muterx to protect the target velocity command **/
      std::mutex mutex;

      /** @brief frame_id from sdf **/
      std::string sdfFrameId;

      /** @brief child_frame_id from sdf **/
      std::string sdfChildFrameId;

  };




  class OmniDrive
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
  {
    public: 
      /** @brief Constructor **/
      OmniDrive();

      /** @brief Destructor **/
      ~OmniDrive() override = default;

      /** Documentation inherited **/
      void Configure(const Entity &_entity,
                     const std::shared_ptr<const sdf::Element> &_sdf,
                     EntityComponentManager &_ecm,
                     EventManager &_eventMgr) override;

      /** Documentation inherited **/
      void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                     ignition::gazebo::EntityComponentManager &_ecm) override;

      /** Documentation inherited **/
      void PostUpdate(const UpdateInfo &_info,
                      const EntityComponentManager &_ecm) override;

    private: 
      /** Private data pointer **/
      std::unique_ptr<OmniDrivePrivate> dataPtr;
  };
  
}// namespace omni_drive

} // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
} // namespace gazebo
} // namespace ignition

#endif
