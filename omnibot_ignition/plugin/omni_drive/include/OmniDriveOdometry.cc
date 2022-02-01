#include <cmath>
#include "OmniDriveOdometry.hh"
#include "ignition/math/RollingMean.hh"

using namespace ignition;
using namespace math;

// The implementation was borrowed from: https://github.com/ros-controls/ros_controllers/blob/melodic-devel/diff_drive_controller/src/odometry.cpp

class ignition::math::OmniDriveOdometryPrivate
{
  /// \brief Integrates the velocities (linear and angular) using 2nd order
  /// Runge-Kutta.
  /// \param[in] _linear Linear velocity.
  /// \param[in] _angular Angular velocity.
  public: void IntegrateRungeKutta2(double _linear, double _angular);

  /// \brief Integrates the velocities (linear and angular) using exact
  /// method.
  /// \param[in] _linear Linear velocity.
  /// \param[in] _angular Angular velocity.
  public: void IntegrateExact(double _linear, double _angular);

  /// \brief Current timestamp.
  public: clock::time_point lastUpdateTime;

  /// \brief Current x position in meters.
  public: double x{0.0};

  /// \brief Current y position in meters.
  public: double y{0.0};

  /// \brief Current heading in radians.
  public: Angle heading;

  /// \brief Current velocity in meter/second.
  public: double linearVel{0.0};

  /// \brief Current angular velocity in radians/second.
  public: Angle angularVel;

  /// \brief Wheel radius in meters.
  public: double wheelRadius{0.2};
      
  /// \brief Distance between right and left wheels in meters.
  public: double wheelRightLeftSeparation{1.0};

  /// \brief Distance between front and rear wheels in meters.
  public: double wheelFrontRearSeparation{1.0};

  /// \brief Previous left wheel position/state in radians.
  public: double leftWheelOldPos{0.0};

  /// \brief Previous right wheel position/state in radians.
  public: double rightWheelOldPos{0.0};

  /// \brief Rolling mean accumulators for the linear velocity
  public: RollingMean linearMean;

  /// \brief Rolling mean accumulators for the angular velocity
  public: RollingMean angularMean;

  /// \brief Initialized flag.
  public: bool initialized{false};
};

//////////////////////////////////////////////////
OmniDriveOdometry::OmniDriveOdometry(size_t _windowSize)
  : dataPtr(new OmniDriveOdometryPrivate)
{
  this->dataPtr->linearMean.SetWindowSize(_windowSize);
  this->dataPtr->angularMean.SetWindowSize(_windowSize);
}

//////////////////////////////////////////////////
OmniDriveOdometry::~OmniDriveOdometry()
{
}

//////////////////////////////////////////////////
void OmniDriveOdometry::Init(const clock::time_point &_time)
{
  // Reset accumulators and timestamp.
  this->dataPtr->linearMean.Clear();
  this->dataPtr->angularMean.Clear();
  this->dataPtr->x = 0.0;
  this->dataPtr->y = 0.0;
  this->dataPtr->heading = 0.0;
  this->dataPtr->linearVel = 0.0;
  this->dataPtr->angularVel = 0.0;
  this->dataPtr->leftWheelOldPos = 0.0;
  this->dataPtr->rightWheelOldPos = 0.0;

  this->dataPtr->lastUpdateTime = _time;
  this->dataPtr->initialized = true;
}

//////////////////////////////////////////////////
bool OmniDriveOdometry::Initialized() const
{
  return this->dataPtr->initialized;
}

//////////////////////////////////////////////////
bool OmniDriveOdometry::Update(const Angle &_frontLeftPos, const Angle &_rearLeftPos,
                               const Angle &_frontRightPos, const Angle &_rearRightPos,
                               const clock::time_point &_time)
{
  // Compute x, y and heading using velocity
  const std::chrono::duration<double> dt =
    _time - this->dataPtr->lastUpdateTime;

  // Get current wheel joint positions:
  const double leftWheelCurPos = *_frontLeftPos * this->dataPtr->wheelRadius;
  const double rightWheelCurPos = *_frontRightPos * this->dataPtr->wheelRadius;

  // Estimate velocity of wheels using old and current position:
  const double leftWheelEstVel = leftWheelCurPos -
                                 this->dataPtr->leftWheelOldPos;

  const double rightWheelEstVel = rightWheelCurPos -
                                  this->dataPtr->rightWheelOldPos;

  // Update old position with current
  this->dataPtr->leftWheelOldPos = leftWheelCurPos;
  this->dataPtr->rightWheelOldPos = rightWheelCurPos;

  // Compute linear and angular diff
  const double linear = (rightWheelEstVel + leftWheelEstVel) * 0.5;
  const double angular = (rightWheelEstVel - leftWheelEstVel) /
    this->dataPtr->wheelFrontRearSeparation;

  this->dataPtr->IntegrateExact(linear, angular);

  // We cannot estimate the speed if the time interval is zero (or near
  // zero).
  if (equal(0.0, dt.count()))
    return false;

  this->dataPtr->lastUpdateTime = _time;

  // Estimate speeds using a rolling mean to filter them out:
  this->dataPtr->linearMean.Push(linear / dt.count());
  this->dataPtr->angularMean.Push(angular / dt.count());

  this->dataPtr->linearVel = this->dataPtr->linearMean.Mean();
  this->dataPtr->angularVel = this->dataPtr->angularMean.Mean();

  return true;
}

//////////////////////////////////////////////////
void OmniDriveOdometry::SetWheelParams(double _wheelRightLeftSeparation,
                                       double _wheelFrontRearSeparation, 
                                       double _wheelRadius)
{
  this->dataPtr->wheelRightLeftSeparation = _wheelRightLeftSeparation;
  this->dataPtr->wheelFrontRearSeparation = _wheelFrontRearSeparation;
  this->dataPtr->wheelRadius = _wheelRadius;
}

//////////////////////////////////////////////////
void OmniDriveOdometry::SetVelocityRollingWindowSize(size_t _size)
{
  this->dataPtr->linearMean.SetWindowSize(_size);
  this->dataPtr->angularMean.SetWindowSize(_size);
}

//////////////////////////////////////////////////
const Angle &OmniDriveOdometry::Heading() const
{
  return this->dataPtr->heading;
}

//////////////////////////////////////////////////
double OmniDriveOdometry::X() const
{
  return this->dataPtr->x;
}

//////////////////////////////////////////////////
double OmniDriveOdometry::Y() const
{
  return this->dataPtr->y;
}

//////////////////////////////////////////////////
double OmniDriveOdometry::LinearVelocity() const
{
  return this->dataPtr->linearVel;
}

//////////////////////////////////////////////////
const Angle &OmniDriveOdometry::AngularVelocity() const
{
  return this->dataPtr->angularVel;
}

//////////////////////////////////////////////////
void OmniDriveOdometryPrivate::IntegrateRungeKutta2(
    double _linear, double _angular)
{
  const double direction = *this->heading + _angular * 0.5;

  // Runge-Kutta 2nd order integration:
  this->x += _linear * std::cos(direction);
  this->y += _linear * std::sin(direction);
  this->heading += _angular;
}

//////////////////////////////////////////////////
void OmniDriveOdometryPrivate::IntegrateExact(double _linear, double _angular)
{
  if (std::fabs(_angular) < 1e-6)
  {
    this->IntegrateRungeKutta2(_linear, _angular);
  }
  else
  {
    // Exact integration (should solve problems when angular is zero):
    const double headingOld = *this->heading;
    const double ratio = _linear / _angular;
    this->heading += _angular;
    this->x += ratio * (std::sin(*this->heading) - std::sin(headingOld));
    this->y += -ratio * (std::cos(*this->heading) - std::cos(headingOld));
  }
}
