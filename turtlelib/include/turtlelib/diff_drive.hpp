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
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// \brief theta
        double theta = 0.0;
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
        /// \brief Create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit DiffDrive(double track, double radius);

        /// @brief Compute inverse kinematics
        /// @param Vb - the twist to compute wheel positions from
        /// @return wheel positions
        WheelPosn InverseKinematics(Twist2D Vb);

        /// @brief Given new wheel positions, update the robot configuration q
        /// @param new_posns - new wheel positions
        void ForwardKinematics(WheelPosn new_posns);

    };
}

#endif