#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.

#include "turtlelib/rigid2d.hpp"
#include <vector>

namespace turtlelib
{
    /// \brief Robot configuration 
    struct RobotConfig
    {
        /// \brief theta
        double theta = 0.0;

        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;
    };

    /// \brief Robot wheel positions
    struct WheelPosn
    {
        /// \brief left wheel
        double left = 0.0;

        /// \brief right wheel
        double right = 0.0;
    };


    /// \brief Modeling the kinematics of a differential drive robot with a given 
    /// wheel track and wheel radius.
    class DiffDrive
    { 
        double track;           // distance between the wheels
        double r;               // radius of the wheels
        WheelPosn wheels;       // wheel positions
        RobotConfig q;          // robot configuration

    public:
        /// @brief Default diff-drive robot.
        DiffDrive();

        /// @brief A diff-drive robot with a given track width and wheel radius. 
        /// @param track_width - distance between the wheels
        /// @param radius - wheel radius
        DiffDrive(double track_width, double radius);

        /// @brief A diff-drive robot with specified track width, wheel radius, 
        ///        and starting configuration
        /// @param track_width - distance between the wheels
        /// @param radius - wheel radius
        /// @param rc_q - robot configuration (theta, x, y)
        DiffDrive(double track_width, double radius, RobotConfig rc_q);

        /// @brief A diff-drive robot with specified track width, wheel radius,
        ///        and wheel positions
        /// @param track_width - distance between the wheels
        /// @param radius - wheel radius
        /// @param wheel_posns - wheel positions 
        DiffDrive(double track_width, double radius, WheelPosn wheel_posns);

        /// @brief A diff-drive robot with specified track width, wheel radius, 
        ///        wheel positions, and starting configuration
        /// @param track_width - distance between the wheels
        /// @param radius - wheel radius
        /// @param wheel_posns - wheel positions
        /// @param rc_q - robot configuration (theta, x, y)
        DiffDrive(double track_width, double radius, WheelPosn wheel_posns, RobotConfig rc_q);

        /// @brief Getter for wheel positions
        /// @return wheel positions
        WheelPosn getWheels() const;

        /// @brief Getter for robot configuration
        /// @return robot config q
        RobotConfig getConfig() const;

        /// @brief Compute inverse kinematics given a body twist
        /// @param Vb - the body twist to compute wheel positions from
        /// @return wheel positions
        WheelPosn InverseKinematics(Twist2D Vb);

        /// @brief Given new wheel positions, update the robot configuration q
        /// @param new_posns - new wheel positions
        void ForwardKinematics(WheelPosn new_posns);

    };
}

#endif