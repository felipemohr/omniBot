#include "OmniDrive.hh"

#include <set>
#include <vector>

#include <ignition/msgs/odometry.pb.h>

#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/components/AngularVelocityCmd.hh"
#include "ignition/gazebo/components/LinearVelocityCmd.hh"

using namespace ignition;
using namespace gazebo;
using namespace omni_drive;


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
  this->dataPtr->model = Model(_entity);

  // Get the canonical link
  std::vector<Entity> links = _ecm.ChildrenByComponents(
    this->dataPtr->model.Entity(), components::CanonicalLink());
  if (!links.empty())
    this->dataPtr->canonicalLink = Link(links[0]);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "OmniDrive plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Ugly, but needed because the sdf::Element::GetElement is not a const
  // function and _sdf is a const shared pointer to a const sdf::Element.
  auto ptr = const_cast<sdf::Element *>(_sdf.get());

  // Get params from SDF
  sdf::ElementPtr sdfElem;

  sdfElem = ptr->GetElement("reference_frame");
  this->dataPtr->referenceFrameName = sdfElem->Get<std::string>();
  this->dataPtr->referenceFrame = this->dataPtr->model.LinkByName(_ecm, this->dataPtr->referenceFrameName);

  sdfElem = ptr->GetElement("front_left_joint");
  this->dataPtr->frontLeftJointName = sdfElem->Get<std::string>();

  sdfElem = ptr->GetElement("rear_left_joint");
  this->dataPtr->rearLeftJointName = sdfElem->Get<std::string>();

  sdfElem = ptr->GetElement("front_right_joint");
  this->dataPtr->frontRightJointName = sdfElem->Get<std::string>();

  sdfElem = ptr->GetElement("rear_right_joint");
  this->dataPtr->rearRightJointName = sdfElem->Get<std::string>();

  this->dataPtr->wheelRightLeftSeparation = _sdf->Get<double>("wheel_right_left_separation",
      this->dataPtr->wheelRightLeftSeparation).first;
  this->dataPtr->wheelFrontRearSeparation = _sdf->Get<double>("wheel_front_rear_separation",
      this->dataPtr->wheelFrontRearSeparation).first;
  this->dataPtr->wheelRadius = _sdf->Get<double>("wheel_radius",
      this->dataPtr->wheelRadius).first;

  // Calculates angle and distance between center of robot and center of the wheels, relative to robot X-axis.
  this->dataPtr->alphaFrontLeftWheel  = atan2(this->dataPtr->wheelRightLeftSeparation, this->dataPtr->wheelFrontRearSeparation);
  this->dataPtr->alphaFrontRightWheel = -this->dataPtr->alphaFrontLeftWheel;
  this->dataPtr->alphaRearLeftWheel   = M_PI/2 + this->dataPtr->alphaFrontLeftWheel;
  this->dataPtr->alphaRearRightWheel  = -this->dataPtr->alphaRearLeftWheel;
  
  this->dataPtr->wheelCenterSeparation = sqrt( pow(this->dataPtr->wheelRightLeftSeparation, 2)/4 + pow(this->dataPtr->wheelFrontRearSeparation, 2)/4 );

  // Instantiate the speed limiters.
  this->dataPtr->limiterLin = std::make_unique<ignition::math::SpeedLimiter>();
  this->dataPtr->limiterAng = std::make_unique<ignition::math::SpeedLimiter>();

  // Parse speed limiter parameters.
  if (_sdf->HasElement("min_velocity_linear"))
  {
    const double minVelLin = _sdf->Get<double>("min_velocity_linear");
    this->dataPtr->limiterLin->SetMinVelocity(minVelLin);
  }
  if (_sdf->HasElement("max_velocity_linear"))
  {
    const double maxVelLin = _sdf->Get<double>("max_velocity_linear");
    this->dataPtr->limiterLin->SetMaxVelocity(maxVelLin);
  }
  if (_sdf->HasElement("min_velocity_angular"))
  {
    const double minVelAng = _sdf->Get<double>("min_velocity_linear");
    this->dataPtr->limiterAng->SetMinVelocity(minVelAng);
  }
  if (_sdf->HasElement("max_velocity_angular"))
  {
    const double maxVelAng = _sdf->Get<double>("max_velocity_linear");
    this->dataPtr->limiterAng->SetMaxVelocity(maxVelAng);
  }
  if (_sdf->HasElement("min_acceleration_linear"))
  {
    const double minAccelLin = _sdf->Get<double>("min_acceleration_linear");
    this->dataPtr->limiterLin->SetMinAcceleration(minAccelLin);
  }
  if (_sdf->HasElement("max_acceleration_linear"))
  {
    const double maxAccelLin = _sdf->Get<double>("max_acceleration_linear");
    this->dataPtr->limiterLin->SetMaxAcceleration(maxAccelLin);
  }
  if (_sdf->HasElement("min_acceleration_angular"))
  {
    const double minAccelAng = _sdf->Get<double>("min_acceleration_angular");
    this->dataPtr->limiterAng->SetMinAcceleration(minAccelAng);
  }
  if (_sdf->HasElement("max_acceleration_angular"))
  {
    const double maxAccelAng = _sdf->Get<double>("max_acceleration_angular");
    this->dataPtr->limiterAng->SetMaxAcceleration(maxAccelAng);
  }
  if (_sdf->HasElement("min_jerk"))
  {
    const double minJerk = _sdf->Get<double>("min_jerk");
    this->dataPtr->limiterLin->SetMinJerk(minJerk);
    this->dataPtr->limiterAng->SetMinJerk(minJerk);
  }
  if (_sdf->HasElement("max_jerk"))
  {
    const double maxJerk = _sdf->Get<double>("max_jerk");
    this->dataPtr->limiterLin->SetMaxJerk(maxJerk);
    this->dataPtr->limiterAng->SetMaxJerk(maxJerk);
  }

  double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50).first;
  if (odomFreq > 0)
  {
    std::chrono::duration<double> odomPer{1 / odomFreq};
    this->dataPtr->odomPubPeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(odomPer);
  }

  // Setup odometry.
  this->dataPtr->odom.SetWheelParams(this->dataPtr->wheelRightLeftSeparation,
                                     this->dataPtr->wheelFrontRearSeparation,
                                     this->dataPtr->wheelRadius);

  // Subscribe to commands
  std::vector<std::string> topics;
  if (_sdf->HasElement("topic"))
  {
    topics.push_back(_sdf->Get<std::string>("topic"));
  }
  topics.push_back("/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel");
  auto topic = validTopic(topics);

  this->dataPtr->node.Subscribe(topic, &OmniDrivePrivate::OnCmdVel,
      this->dataPtr.get());

  // Subscribe to enable/disable
  std::vector<std::string> enableTopics;
  enableTopics.push_back(
    "/model/" + this->dataPtr->model.Name(_ecm) + "/enable");
  auto enableTopic = validTopic(enableTopics);

  if (!enableTopic.empty())
  {
    this->dataPtr->node.Subscribe(enableTopic, &OmniDrivePrivate::OnEnable,
        this->dataPtr.get());
  }
  this->dataPtr->enabled = true;

  std::vector<std::string> odomTopics;
  if (_sdf->HasElement("odom_topic"))
  {
    odomTopics.push_back(_sdf->Get<std::string>("odom_topic"));
  }
  odomTopics.push_back("/model/" + this->dataPtr->model.Name(_ecm) +
      "/odometry");
  auto odomTopic = validTopic(odomTopics);

  this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
      odomTopic);

  std::string tfTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
    "/tf"};
  if (_sdf->HasElement("tf_topic"))
    tfTopic = _sdf->Get<std::string>("tf_topic");
  this->dataPtr->tfPub = this->dataPtr->node.Advertise<msgs::Pose_V>(
      tfTopic);

  ignmsg << "OmniDrive subscribing to twist messages on [" << topic << "]"
         << std::endl;
}

//////////////////////////////////////////////////
void OmniDrive::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                          ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("OmniDrive::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // If the joints haven't been identified yet, look for them
  static std::set<std::string> warnedModels;
  auto modelName = this->dataPtr->model.Name(_ecm);
  if (!this->dataPtr->frontLeftJoint || !this->dataPtr->frontRightJoint || 
      !this->dataPtr->rearLeftJoint  || !this->dataPtr->rearRightJoint)
  {

    bool warned{false};

    auto name = this->dataPtr->frontLeftJointName;
    Entity joint = this->dataPtr->model.JointByName(_ecm, name);
    if (joint != kNullEntity)
      this->dataPtr->frontLeftJoint = joint;
    else if (warnedModels.find(modelName) == warnedModels.end())
    {
      ignwarn << "Failed to find front left joint [" << name << "] for model ["
              << modelName << "]" << std::endl;
      warned = true;
    }

    name = this->dataPtr->frontRightJointName;
    joint = this->dataPtr->model.JointByName(_ecm, name);
    if (joint != kNullEntity)
      this->dataPtr->frontRightJoint = joint;
    else if (warnedModels.find(modelName) == warnedModels.end())
    {
      ignwarn << "Failed to find front right joint [" << name << "] for model ["
              << modelName << "]" << std::endl;
      warned = true;
    }

    name = this->dataPtr->rearLeftJointName;
    joint = this->dataPtr->model.JointByName(_ecm, name);
    if (joint != kNullEntity)
      this->dataPtr->rearLeftJoint = joint;
    else if (warnedModels.find(modelName) == warnedModels.end())
    {
      ignwarn << "Failed to find rear left joint [" << name << "] for model ["
              << modelName << "]" << std::endl;
      warned = true;
    }

    name = this->dataPtr->rearRightJointName;
    joint = this->dataPtr->model.JointByName(_ecm, name);
    if (joint != kNullEntity)
      this->dataPtr->rearRightJoint = joint;
    else if (warnedModels.find(modelName) == warnedModels.end())
    {
      ignwarn << "Failed to find rear right joint [" << name << "] for model ["
              << modelName << "]" << std::endl;
      warned = true;
    }

    if (warned)
      warnedModels.insert(modelName);
  }

  if (!this->dataPtr->frontLeftJoint || !this->dataPtr->frontRightJoint || 
      !this->dataPtr->rearLeftJoint  || !this->dataPtr->rearRightJoint)
    return;
  
  if (warnedModels.find(modelName) != warnedModels.end())
  {
    ignmsg << "Found joints for model [" << modelName
           << "], plugin will start working." << std::endl;
    warnedModels.erase(modelName);
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  Entity joint = this->dataPtr->frontLeftJoint;
  // skip this entity if it has been removed
  if (_ecm.HasEntity(joint))
  {
    // Update wheel velocity
    auto vel = _ecm.Component<components::JointVelocityCmd>(joint);
    if (vel == nullptr)
      _ecm.CreateComponent( joint, components::JointVelocityCmd({this->dataPtr->frontLeftJointSpeed}));
    else
      *vel = components::JointVelocityCmd({this->dataPtr->frontLeftJointSpeed});
  }

  joint = this->dataPtr->frontRightJoint;
  // skip this entity if it has been removed
  if (_ecm.HasEntity(joint))
  {
    // Update wheel velocity
    auto vel = _ecm.Component<components::JointVelocityCmd>(joint);
    if (vel == nullptr)
      _ecm.CreateComponent( joint, components::JointVelocityCmd({this->dataPtr->frontRightJointSpeed}));
    else
      *vel = components::JointVelocityCmd({this->dataPtr->frontRightJointSpeed});
  }

  joint = this->dataPtr->rearLeftJoint;
  // skip this entity if it has been removed
  if (_ecm.HasEntity(joint))
  {
    // Update wheel velocity
    auto vel = _ecm.Component<components::JointVelocityCmd>(joint);
    if (vel == nullptr)
      _ecm.CreateComponent( joint, components::JointVelocityCmd({this->dataPtr->rearLeftJointSpeed}));
    else
      *vel = components::JointVelocityCmd({this->dataPtr->rearLeftJointSpeed});
  }

  joint = this->dataPtr->rearRightJoint;
  // skip this entity if it has been removed
  if (_ecm.HasEntity(joint))
  {
    // Update wheel velocity
    auto vel = _ecm.Component<components::JointVelocityCmd>(joint);
    if (vel == nullptr)
      _ecm.CreateComponent( joint, components::JointVelocityCmd({this->dataPtr->rearRightJointSpeed}));
    else
      *vel = components::JointVelocityCmd({this->dataPtr->rearRightJointSpeed});
  }

  // Create the joint position components if they don't exist.
  auto frontLeftPos = _ecm.Component<components::JointPosition>(this->dataPtr->frontLeftJoint);
  if (!frontLeftPos && _ecm.HasEntity(this->dataPtr->frontLeftJoint))
    _ecm.CreateComponent(this->dataPtr->frontLeftJoint, components::JointPosition());

  auto frontRightPos = _ecm.Component<components::JointPosition>(this->dataPtr->frontRightJoint);
  if (!frontRightPos && _ecm.HasEntity(this->dataPtr->frontRightJoint))
    _ecm.CreateComponent(this->dataPtr->frontRightJoint, components::JointPosition());

  auto rearLeftPos = _ecm.Component<components::JointPosition>(this->dataPtr->rearLeftJoint);
  if (!rearLeftPos && _ecm.HasEntity(this->dataPtr->rearLeftJoint))
    _ecm.CreateComponent(this->dataPtr->rearLeftJoint, components::JointPosition());

  auto rearRightPos = _ecm.Component<components::JointPosition>(this->dataPtr->rearRightJoint);
  if (!rearRightPos && _ecm.HasEntity(this->dataPtr->rearRightJoint))
    _ecm.CreateComponent(this->dataPtr->rearRightJoint, components::JointPosition());

  // Update robot velocity
  math::Vector3d linearVel{this->dataPtr->targetLinVelX, this->dataPtr->targetLinVelY, -9.8};
  math::Vector3d angularVel{0.0, 0.0, this->dataPtr->targetAngVelZ};

  auto linkLinearVelComp = _ecm.Component<components::LinearVelocityCmd>(this->dataPtr->referenceFrame);
  if (!linkLinearVelComp)
  {
    _ecm.CreateComponent(this->dataPtr->referenceFrame,
        components::LinearVelocityCmd({linearVel}));
  }
  else
  {
    *linkLinearVelComp = components::LinearVelocityCmd(linearVel);
  }

  auto linkAngularVelComp = _ecm.Component<components::AngularVelocityCmd>(this->dataPtr->referenceFrame);
  if (!linkAngularVelComp)
  {
    _ecm.CreateComponent(this->dataPtr->referenceFrame,
        components::AngularVelocityCmd({angularVel}));
  }
  else
  {
    *linkAngularVelComp = components::AngularVelocityCmd(angularVel);
  }

}

//////////////////////////////////////////////////
void OmniDrive::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("OmniDrive::PostUpdate");

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->UpdateVelocity(_info, _ecm);
  this->dataPtr->UpdateOdometry(_info, _ecm);

}

//////////////////////////////////////////////////
void OmniDrivePrivate::UpdateOdometry(const ignition::gazebo::UpdateInfo &_info,
                                      const ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("DiffDrive::UpdateOdometry");
  // Initialize, if not already initialized.
  if (!this->odom.Initialized())
  {
    this->odom.Init(std::chrono::steady_clock::time_point(_info.simTime));
    return;
  }

  if (this->leftJoints.empty() || this->rightJoints.empty())
    return;


}

//////////////////////////////////////////////////
void OmniDrivePrivate::UpdateVelocity(const ignition::gazebo::UpdateInfo &_info,
                                      const ignition::gazebo::EntityComponentManager &/*_ecm*/)
{
  IGN_PROFILE("OmniDrive::UpdateVelocity");

  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->targetLinVelX = this->targetVel.linear().x();
    this->targetLinVelY = this->targetVel.linear().y();
    this->targetAngVelZ = this->targetVel.angular().z();
  }

  // Limit the target velocity if needed.
  this->limiterLin->Limit(
      this->targetLinVelX, this->last0Cmd.lin_x, this->last1Cmd.lin_x, _info.dt);
  this->limiterLin->Limit(
      this->targetLinVelY, this->last0Cmd.lin_y, this->last1Cmd.lin_y, _info.dt);
  this->limiterAng->Limit(
      this->targetAngVelZ, this->last0Cmd.ang_z, this->last1Cmd.ang_z, _info.dt);

  // Update history of commands.
  this->last1Cmd = last0Cmd;
  this->last0Cmd.lin_x = this->targetLinVelX;
  this->last0Cmd.lin_y = this->targetLinVelY;
  this->last0Cmd.ang_z = this->targetAngVelZ;

  // Convert the target velocities to joint velocities.
  this->frontLeftJointSpeed  = (1/this->wheelRadius) * (this->targetLinVelX - this->targetLinVelY + this->wheelCenterSeparation*( sin( 3*M_PI_4 - this->alphaFrontLeftWheel ) / sin(-M_PI_4) )*this->targetAngVelZ);
  this->frontRightJointSpeed = (1/this->wheelRadius) * (this->targetLinVelX + this->targetLinVelY + this->wheelCenterSeparation*( sin(-3*M_PI_4 - this->alphaFrontRightWheel) / sin( M_PI_4) )*this->targetAngVelZ);
  this->rearLeftJointSpeed   = (1/this->wheelRadius) * (this->targetLinVelX + this->targetLinVelY + this->wheelCenterSeparation*( sin( M_PI_4   - this->alphaRearLeftWheel  ) / sin( M_PI_4) )*this->targetAngVelZ);
  this->rearRightJointSpeed  = (1/this->wheelRadius) * (this->targetLinVelX - this->targetLinVelY + this->wheelCenterSeparation*( sin(-M_PI_4   - this->alphaRearRightWheel ) / sin(-M_PI_4) )*this->targetAngVelZ);

}

//////////////////////////////////////////////////
void OmniDrivePrivate::OnCmdVel(const msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->enabled)
  {
    this->targetVel = _msg;
  }
}

//////////////////////////////////////////////////
void OmniDrivePrivate::OnEnable(const msgs::Boolean &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->enabled = _msg.data();
  if (!this->enabled)
  {
    math::Vector3d zeroVector{0, 0, 0};
    msgs::Set(this->targetVel.mutable_linear(), zeroVector);
    msgs::Set(this->targetVel.mutable_angular(), zeroVector);
  }
}


IGNITION_ADD_PLUGIN(OmniDrive,
                    ignition::gazebo::System,
                    OmniDrive::ISystemConfigure,
                    OmniDrive::ISystemPreUpdate,
                    OmniDrive::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(OmniDrive, "omni_drive::OmniDrive")
