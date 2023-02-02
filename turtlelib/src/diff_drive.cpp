#include "diff_drive.hpp"
#include <cmath>

namespace turtlelib{

    DiffDrive::DiffDrive(double track, double radius){};

    WheelPosn DiffDrive::InverseKinematics(Twist2D Vb)
    {
        WheelPosn ik;
        double D = track/2;
        ik.left = (-D/r)*Vb.w + Vb.x/r;
        ik.left = (D/r)*Vb.w + Vb.x/r;

        return ik;
    }        


    void DiffDrive::ForwardKinematics(WheelPosn new_posns)
    {
        Twist2D Vb;
        double D = track/2;
        Vb.w = (r/2)*(new_posns.left-new_posns.right)/D;
        Vb.x = (r/2)*(new_posns.left+new_posns.right);
        Vb.y = 0;

        Transform2D Tbbp = integrate_twist(Vb);
        double thetab = Tbbp.rotation();
        Vector2D dvec = Tbbp.translation();
        RobotConfig new_q;
        new_q.theta = q.theta + Vb.w;
        new_q.x = q.x + Vb.x*cos(q.theta) - Vb.y*sin(q.theta);
        new_q.y = q.y + Vb.x*sin(q.theta) + Vb.y*cos(q.theta);

        q = new_q;
    }

        
};