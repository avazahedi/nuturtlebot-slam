#ifndef CIRCLEFIT_INCLUDE_GUARD_HPP
#define CIRCLEFIT_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include<iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include "rigid2d.hpp"
#include <vector>

namespace turtlelib
{
    /// \brief A circle consisting of a center (x,y) and radius
    struct Circle
    {
        // /// \brief the center x coordinate
        // double cx = 0.0;

        // /// \brief the center y coordinate
        // double cy = 0.0;

        /// @brief center x and y
        Vector2D center {0.0, 0.0};

        /// @brief radius
        double radius = 0.0;
    };

    /// @brief Fit a circle to a given cluster
    /// @param cluster vector of Vector2Ds
    /// @return the fitted circle
    Circle circle_fit(std::vector<turtlelib::Vector2D> cluster);

}

#endif
