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

    class OmniDriveOdometryPrivate;
    

    class IGNITION_MATH_VISIBLE OmniDriveOdometry
    {
      public:
        /**
         * @brief Construct a new OmniDriveOdometry object
         * 
         * @param _windowSize Rolling window size used to compute the velocity mean
         */
        explicit OmniDriveOdometry(size_t _windowSize = 10);

        /** @brief Destructor. **/
        ~OmniDriveOdometry();

        /**
         * @brief Initialize the odometry
         * 
         * @param _time Current time.
         */
        void Init(const clock::time_point &_time);

        /**
         * @brief Get whether Init has been called
         * 
         * @return True if Init has been called, false otherwise. 
         */
        bool Initialized() const;

        /**
         * @brief Updates the odometry class with latest wheels and steerings position
         * 
         * @param _frontLeftPos Front left wheel position in radians.
         * @param _rearLeftPos Rear left wheel position in radians.
         * @param _frontRightPos Front right wheel position in radians.
         * @param _rearRightPos Rear right wheel position in radians.
         * @param _time 
         * @return True if the odometry is actually updated.
         */
        bool Update(const Angle &_frontLeftPos, const Angle &_rearLeftPos,
                            const Angle &_frontRightPos, const Angle &_rearRightPos,
                            const clock::time_point &_time);

        /**
         * @brief Get the heading
         * 
         * @return The heading in radians.
         */
        const Angle &Heading() const;

        /**
         * @brief Get the X position
         * 
         * @return The X position in meters. 
         */
        double X() const;

        /**
         * @brief Get the Y position
         * 
         * @return The Y position in meters. 
         */
        double Y() const;

        /**
         * @brief Get the linear velocity X
         * 
         * @return The linear velocity Y in meter/second. 
         */
        double LinearVelocityX() const;

        /**
         * @brief Get the linear velocity Y
         * 
         * @return The linear velocity Y in meter/second. 
         */
        double LinearVelocityY() const;

        /**
         * @brief Get the angular velocity
         * 
         * @return The angular velocity Y in meter/second. 
         */
        const Angle &AngularVelocity() const;

        /**
         * @brief Set the wheel parameters including the radius and separation
         * 
         * @param _wheelRightLeftSeparation Distance between left and right wheels.
         * @param _wheelFrontRearSeparation Distance between front and rear wheels.
         * @param _wheelRadius Radius of the left wheel.
         */
        void SetWheelParams(double _wheelRightLeftSeparation,
                        double _wheelFrontRearSeparation, double _wheelRadius);

        /**
         * @brief Set the velocity rolling window size
         * 
         * @param _size The Velocity rolling window size.
         */
        void SetVelocityRollingWindowSize(size_t _size);

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::unique_ptr
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
      private: 
        /** @brief Private data pointer. **/
        std::unique_ptr<OmniDriveOdometryPrivate> dataPtr;
#ifdef _WIN32
#pragma warning(pop)
#endif
    };
    }
  }
}

#endif
