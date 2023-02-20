#include "turtlelib/diff_drive.hpp"
#include <cmath>
#include <iostream>

namespace turtlelib{

    DiffDrive::DiffDrive(): track{0.16}, r{0.033}, wheels{0.0, 0.0}, q{0.0, 0.0, 0.0} {}

    DiffDrive::DiffDrive(double track_width, double radius): track{track_width}, r{radius}, 
                         wheels{0.0, 0.0}, q{0.0, 0.0, 0.0} {}

    DiffDrive::DiffDrive(double track_width, double radius, RobotConfig rc_q): track{track_width}, 
                         r{radius}, wheels{0.0, 0.0}, q{rc_q.theta, rc_q.x, rc_q.y} {}

    DiffDrive::DiffDrive(double track_width, double radius, WheelPosn wheel_posns): 
                         track{track_width}, r{radius}, wheels{wheel_posns.left, 
                         wheel_posns.right}, q{0.0, 0.0, 0.0} {}

    DiffDrive::DiffDrive(double track_width, double radius, WheelPosn wheel_posns, 
                         RobotConfig rc_q): track{track_width}, r{radius}, 
                         wheels{wheel_posns.left, wheel_posns.right}, 
                         q{rc_q.theta, rc_q.x, rc_q.y} {}

    WheelPosn DiffDrive::getWheels() const
    {
         return wheels;
    }

    void DiffDrive::setWheels(WheelPosn wps)
    {
        wheels.left = wps.left;
        wheels.right = wps.right;
    }

    RobotConfig DiffDrive::getConfig() const
    {
        return q;
    }

    void DiffDrive::setConfig(RobotConfig config)
    {
        q = config;
    }

    Twist2D DiffDrive::getTwist(WheelPosn new_posns) const
    {
        WheelPosn dphi {new_posns.left-wheels.left, new_posns.right-wheels.right};
        Twist2D Vb;
        double D = track/2;

        // eqn 2.1
        Vb.w = (r/2)*(dphi.right-dphi.left)/D;
        Vb.x = (r/2)*(dphi.left+dphi.right);
        Vb.y = 0.0;

        return Vb;
    }

    WheelPosn DiffDrive::InverseKinematics(Twist2D Vb)
    {
        WheelPosn ik;
        double D = track/2.0;

        if (Vb.y != 0.0)
        {
            throw std::logic_error(
              std::string("Failed: Wheels slipping: y-velocity not equal to 0."));
        }

        // eqns 1.1 and 1.2
        ik.left = (-D/r)*Vb.w + Vb.x/r;
        ik.right = (D/r)*Vb.w + Vb.x/r;

        return ik;
    }        


    void DiffDrive::ForwardKinematics(WheelPosn new_posns)
    {
        // need the difference in wheel positions
        WheelPosn dphi {new_posns.left-wheels.left, new_posns.right-wheels.right};
        Twist2D Vb;
        double D = track/2;

        // eqn 2.1
        Vb.w = (r/2)*(dphi.right-dphi.left)/D;
        Vb.x = (r/2)*(dphi.left+dphi.right);
        Vb.y = 0.0;

        // eqn 2.2
        Transform2D Tbbp = integrate_twist(Vb);
        double thetab = Tbbp.rotation();
        Vector2D dvec = Tbbp.translation();

        // eqns 2.3 - 2.5
        RobotConfig new_q;
        new_q.theta = q.theta + thetab;
        new_q.x = q.x + dvec.x*cos(q.theta) - dvec.y*sin(q.theta);
        new_q.y = q.y + dvec.x*sin(q.theta) + dvec.y*cos(q.theta);

        q = new_q;
    }

        
};