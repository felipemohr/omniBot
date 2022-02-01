#ifndef IGNITION_MATH_OMNIDRIVEODOMETRY_HH_
#define IGNITION_MATH_OMNIDRIVEODOMETRY_HH_

#include <chrono>
#include <memory>
#include <ignition/math/Angle.hh>
#include <ignition/math/Export.hh>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    // Use a steady clock
    using clock = std::chrono::steady_clock;

    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    // Forward declarations.
    class OmniDriveOdometryPrivate;

    /** \class OmniDriveOdometry OmniDriveOdometry.hh \
     * ignition/math/OmniDriveOdometry.hh
     **/
    /// \brief Computes odometry values based on a set of kinematic
    /// properties and wheel speeds for a diff-drive vehicle.
    ///
    /// A vehicle with a heading of zero degrees has a local
    /// reference frame according to the diagram below.
    ///
    ///       Y
    ///       ^
    ///       |
    ///       |
    ///       O--->X(forward)
    ///
    /// Rotating the right wheel while keeping the left wheel fixed will cause
    /// the vehicle to rotate counter-clockwise. For example (excuse the
    /// lack of precision with ASCII art):
    ///
    ///     Y     X(forward)
    ///     ^     ^
    ///      \   /
    ///       \ /
    ///        O
    ///
    /// **Example Usage**
    ///
    /// \code{.cpp}
    /// ignition::math::OmniDriveOdometry odom;
    /// odom.SetWheelParams(2.0, 0.5, 0.5);
    /// odom.Init(std::chrono::steady_clock::now());
    ///
    /// // ... Some time later
    ///
    /// // Both wheels have rotated the same amount
    /// odom.Update(IGN_DTOR(2), IGN_DTOR(2), std::chrono::steady_clock::now());
    ///
    /// // ... Some time later
    ///
    /// // The left wheel has rotated, the right wheel did not rotate
    /// odom.Update(IGN_DTOR(4), IGN_DTOR(2), std::chrono::steady_clock::now());
    /// \endcode
    class IGNITION_MATH_VISIBLE OmniDriveOdometry
    {
      /// \brief Constructor.
      /// \param[in] _windowSize Rolling window size used to compute the
      /// velocity mean
      public: explicit OmniDriveOdometry(size_t _windowSize = 10);

      /// \brief Destructor.
      public: ~OmniDriveOdometry();

      /// \brief Initialize the odometry
      /// \param[in] _time Current time.
      public: void Init(const clock::time_point &_time);

      /// \brief Get whether Init has been called.
      /// \return True if Init has been called, false otherwise.
      public: bool Initialized() const;

      /// \brief Updates the odometry class with latest wheels and
      /// steerings position
      /// \param[in] _leftPos Left wheel position in radians.
      /// \param[in] _rightPos Right wheel postion in radians.
      /// \param[in] _time Current time point.
      /// \return True if the odometry is actually updated.
      public: bool Update(const Angle &_leftPos, const Angle &_rightPos,
                          const clock::time_point &_time);

      /// \brief Get the heading.
      /// \return The heading in radians.
      public: const Angle &Heading() const;

      /// \brief Get the X position.
      ///  \return The X position in meters
      public: double X() const;

      /// \brief Get the Y position.
      /// \return The Y position in meters.
      public: double Y() const;

      /// \brief Get the linear velocity.
      /// \return The linear velocity in meter/second.
      public: double LinearVelocity() const;

      /// \brief Get the angular velocity.
      /// \return The angular velocity in radian/second.
      public: const Angle &AngularVelocity() const;

      /// \brief Set the wheel parameters including the radius and separation.
      /// \param[in] _wheelRightLeftSeparation Distance between left and right wheels.
      /// \param[in] _wheelFrontRearSeparation Radius of the left wheel.
      /// \param[in] _wheelRadius Radius of the right wheel.
      public: void SetWheelParams(double _wheelRightLeftSeparation,
                      double _wheelFrontRearSeparation, double _wheelRadius);

      /// \brief Set the velocity rolling window size.
      /// \param[in] _size The Velocity rolling window size.
      public: void SetVelocityRollingWindowSize(size_t _size);

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::unique_ptr
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
      /// \brief Private data pointer.
      private: std::unique_ptr<OmniDriveOdometryPrivate> dataPtr;
#ifdef _WIN32
#pragma warning(pop)
#endif
    };
    }
  }
}

#endif
