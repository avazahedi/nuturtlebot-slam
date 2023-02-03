#include "turtlelib/diff_drive.hpp"
#include <cmath>
#include <iostream>

namespace turtlelib{

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

    RobotConfig DiffDrive::getConfig() const
    {
        return q;
    }

    WheelPosn DiffDrive::InverseKinematics(Twist2D Vb)
    {
        WheelPosn ik;
        double D = track/2;

        if (Vb.y != 0)
        {
            throw std::logic_error(
              std::string("Failed: Wheels slipping: y-velocity not equal to 0."));
        }

        ik.left = (-D/r)*Vb.w + Vb.x/r;
        ik.right = (D/r)*Vb.w + Vb.x/r;

        return ik;
    }        


    void DiffDrive::ForwardKinematics(WheelPosn new_posns)
    {
        WheelPosn dphi = new_posns;
        // WheelPosn dphi {new_posns.left-wheels.left, new_posns.right-wheels.right};
        Twist2D Vb;
        double D = track/2;
        Vb.w = (r/2)*(dphi.right-dphi.left)/D;
        Vb.x = (r/2)*(dphi.left+dphi.right);
        Vb.y = 0;

        Transform2D Tbbp = integrate_twist(Vb);
        double thetab = Tbbp.rotation();
        Vector2D dvec = Tbbp.translation();
        RobotConfig new_q;
        new_q.theta = q.theta + thetab;
        new_q.x = q.x + dvec.x*cos(q.theta) - dvec.y*sin(q.theta);
        new_q.y = q.y + dvec.x*sin(q.theta) + dvec.y*cos(q.theta);

        q = new_q;
    }

        
};