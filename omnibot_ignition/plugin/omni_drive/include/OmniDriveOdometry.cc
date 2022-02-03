#include <cmath>
#include "OmniDriveOdometry.hh"
#include "ignition/math/RollingMean.hh"

using namespace ignition;
using namespace math;

class ignition::math::OmniDriveOdometryPrivate
{
  public:
    /** @brief Integrates the velocities (linear and angular) using 2nd order Runge-Kutta
     * \param _linear_x Linear velocity x.
     * \param _linear_y Linear velocity y.
     * \param _angular Angular velocity.
     */
    void IntegrateRungeKutta2(double _linear_x, double _linear_y, double _angular);

    /** 
     * @brief Integrates the velocities (linear and angular) using exact method
     * 
     * @param _linear_x Linear velocity x.
     * @param _linear_y Linear velocity y.
     * @param _angular Angular velocity.
     */    
    void IntegrateExact(double _linear_x, double _linear_y, double _angular);

    /** @brief Current timestamp. **/
    clock::time_point lastUpdateTime;

    /** @brief Current x position in meters. **/
    double x{0.0};

    /** @brief Current y position in meters. **/
    double y{0.0};

    /** @brief Current heading in radians. **/
    Angle heading;

    /** @brief Current velocity x in meter/second. **/
    double linearVelX{0.0};

    /** @brief Current velocity y in meter/second. **/
    double linearVelY{0.0};

    /** @brief Current angular velocity in radians/second. **/
    Angle angularVel;

    /** @brief Wheel radius in meters. **/
    double wheelRadius{0.2};
        
    /** @brief Distance between right and left wheels in meters. **/
    double wheelRightLeftSeparation{1.0};

    /** @brief Distance between front and rear wheels in meters. **/
    double wheelFrontRearSeparation{1.0};

    /** @brief Previous front left wheel position/state in radians. **/
    double frontLeftWheelOldPos{0.0};

    /** @brief Previous rear left wheel position/state in radians. **/
    double rearLeftWheelOldPos{0.0};

    /** @brief Previous front right wheel position/state in radians. **/
    double frontRightWheelOldPos{0.0};

    /** @brief Previous rear right wheel position/state in radians. **/
    double rearRightWheelOldPos{0.0};

    /** @brief Rolling mean accumulators for the linear x velocity **/
    RollingMean linearXMean;

    /** @brief Rolling mean accumulators for the linear y velocity **/
    RollingMean linearYMean;

    /** @brief Rolling mean accumulators for the angular velocity **/
    RollingMean angularMean;

    /** @brief Initialized flag. **/
    bool initialized{false};
};

//////////////////////////////////////////////////
OmniDriveOdometry::OmniDriveOdometry(size_t _windowSize)
  : dataPtr(new OmniDriveOdometryPrivate)
{
  this->dataPtr->linearXMean.SetWindowSize(_windowSize);
  this->dataPtr->linearYMean.SetWindowSize(_windowSize);
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
  this->dataPtr->linearXMean.Clear();
  this->dataPtr->linearYMean.Clear();
  this->dataPtr->angularMean.Clear();
  this->dataPtr->x = 0.0;
  this->dataPtr->y = 0.0;
  this->dataPtr->heading = 0.0;
  this->dataPtr->linearVelX = 0.0;
  this->dataPtr->linearVelY = 0.0;
  this->dataPtr->angularVel = 0.0;
  this->dataPtr->frontLeftWheelOldPos = 0.0;
  this->dataPtr->rearLeftWheelOldPos = 0.0;
  this->dataPtr->frontRightWheelOldPos = 0.0;
  this->dataPtr->rearRightWheelOldPos = 0.0;

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
  const double frontLeftWheelCurPos  = *_frontLeftPos  ;//* this->dataPtr->wheelRadius;
  const double rearLeftWheelCurPos   = *_rearLeftPos   ;//* this->dataPtr->wheelRadius;
  const double frontRightWheelCurPos = *_frontRightPos ;//* this->dataPtr->wheelRadius;
  const double rearRightWheelCurPos  = *_rearRightPos  ;//* this->dataPtr->wheelRadius;

  // Estimate velocity of wheels using old and current position:
  const double frontLeftWheelEstVel = frontLeftWheelCurPos -
                                 this->dataPtr->frontLeftWheelOldPos;

  const double rearLeftWheelEstVel = rearLeftWheelCurPos -
                                 this->dataPtr->rearLeftWheelOldPos;

  const double frontRightWheelEstVel = frontRightWheelCurPos -
                                  this->dataPtr->frontRightWheelOldPos;

  const double rearRightWheelEstVel = rearRightWheelCurPos -
                                  this->dataPtr->rearRightWheelOldPos;

  // Update old position with current
  this->dataPtr->frontLeftWheelOldPos  = frontLeftWheelCurPos;
  this->dataPtr->rearLeftWheelOldPos   = rearLeftWheelCurPos;
  this->dataPtr->frontRightWheelOldPos = frontRightWheelCurPos;
  this->dataPtr->rearRightWheelOldPos  = rearRightWheelCurPos;

  // Compute linear and angular diff
  const double linear_x = (frontLeftWheelEstVel + frontRightWheelEstVel + 
                           rearLeftWheelEstVel + rearRightWheelEstVel) * 
                           (this->dataPtr->wheelRadius / 4);
  const double linear_y = (-frontLeftWheelEstVel + frontRightWheelEstVel + 
                           rearLeftWheelEstVel - rearRightWheelEstVel) * 
                           (this->dataPtr->wheelRadius / 4);
  const double angular  = (-frontLeftWheelEstVel + frontRightWheelEstVel - 
                           rearLeftWheelEstVel + rearRightWheelEstVel) * 
                           (this->dataPtr->wheelRadius / 4) /
                           (this->dataPtr->wheelFrontRearSeparation/2 + 
                            this->dataPtr->wheelRightLeftSeparation/2);

  this->dataPtr->IntegrateExact(linear_x, linear_y, angular);

  // We cannot estimate the speed if the time interval is zero (or near
  // zero).
  if (equal(0.0, dt.count()))
    return false;

  this->dataPtr->lastUpdateTime = _time;

  // Estimate speeds using a rolling mean to filter them out:
  this->dataPtr->linearXMean.Push(linear_x / dt.count());
  this->dataPtr->linearYMean.Push(linear_y / dt.count());
  this->dataPtr->angularMean.Push(angular  / dt.count());

  this->dataPtr->linearVelX = this->dataPtr->linearXMean.Mean();
  this->dataPtr->linearVelY = this->dataPtr->linearYMean.Mean();
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
  this->dataPtr->linearXMean.SetWindowSize(_size);
  this->dataPtr->linearYMean.SetWindowSize(_size);
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
double OmniDriveOdometry::LinearVelocityX() const
{
  return this->dataPtr->linearVelX;
}

//////////////////////////////////////////////////
double OmniDriveOdometry::LinearVelocityY() const
{
  return this->dataPtr->linearVelY;
}

//////////////////////////////////////////////////
const Angle &OmniDriveOdometry::AngularVelocity() const
{
  return this->dataPtr->angularVel;
}

//////////////////////////////////////////////////
void OmniDriveOdometryPrivate::IntegrateRungeKutta2(
    double _linear_x, double _linear_y, double _angular)
{
  const double direction = *this->heading + _angular * 0.5;

  // Runge-Kutta 2nd order integration:
  this->x += _linear_x * std::cos(direction) + _linear_y * std::sin(direction);
  this->y += _linear_x * std::sin(direction) + _linear_y * std::cos(direction);
  this->heading += _angular;
}

//////////////////////////////////////////////////
void OmniDriveOdometryPrivate::IntegrateExact(double _linear_x, double _linear_y, double _angular)
{
  if (std::fabs(_angular) < 1e-6)
  {
    this->IntegrateRungeKutta2(_linear_x, _linear_y, _angular);
  }
  else
  {
    // Exact integration (should solve problems when angular is zero):
    const double headingOld = *this->heading;
    const double ratio = sqrt(pow(_linear_x, 2) + pow(_linear_y, 2)) / _angular;
    this->heading += _angular;
    this->x += ratio * (std::sin(*this->heading) - std::sin(headingOld));
    this->y += -ratio * (std::cos(*this->heading) - std::cos(headingOld));
  }
}
