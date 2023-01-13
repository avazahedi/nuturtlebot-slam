/// \file
/// \brief Two-dimensional rigid body transformations.

#include <iostream>
#include "rigid2d.hpp"
#include <cmath>

namespace turtlelib {

    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        os << "[" << v.x << " " << v.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
        char c = is.peek();
        if (c == '[') {
            is.get();
        }
        is >> v.x;
        is.get();
        is >> v.y;
        
        std::cin.ignore(100, '\n');
        return is;
    }

    std::ostream & operator<<(std::ostream & os, const Twist2D & t)
    {
        os << "[" << t.w << " " << t.x << " " << t.y << "]";
        return os;
    }


    std::istream & operator>>(std::istream & is, Twist2D & t)
    {
        char c = is.peek();
        if (c == '[') {
            is.get();
        }
        is >> t.w;
        is.get();
        is >> t.x;
        is.get();
        is >> t.y;

        std::cin.ignore(100, '\n');
        return is;
    }


    Transform2D::Transform2D(): tvec{0.0, 0.0}, phi{0.0} {}

    Transform2D::Transform2D(Vector2D trans): tvec{trans.x, trans.y}, phi{0.0} {}

    Transform2D::Transform2D(double radians): tvec{0.0, 0.0}, phi{radians} {}

    Transform2D::Transform2D(Vector2D trans, double radians): tvec{trans.x, trans.y}, phi{radians} {}

    Vector2D Transform2D::operator()(Vector2D v) const
    {
        Vector2D new_v;
        new_v.x = cos(phi)*v.x - sin(phi)*v.y + tvec.x;
        new_v.y = sin(phi)*v.x + cos(phi)*v.y + tvec.y;

        return new_v;
    }

    Transform2D Transform2D::inv() const
    {
        Transform2D inverse;
        inverse.phi = -phi;
        inverse.tvec.x = -tvec.x*cos(phi)-tvec.y*sin(phi);
        inverse.tvec.y = -tvec.y*cos(phi)+tvec.x*sin(phi);

        return inverse;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {
        Transform2D tmp;
        tmp.phi = phi;
        tmp.tvec.x = tvec.x;
        tmp.tvec.y = tvec.y;

        phi = acos(cos(tmp.phi)*cos(rhs.phi) - sin(tmp.phi)*sin(rhs.phi));
        tvec.x = cos(tmp.phi)*rhs.tvec.x - sin(tmp.phi)*rhs.tvec.y + tmp.tvec.x;
        tvec.y = sin(tmp.phi)*rhs.tvec.x + cos(tmp.phi)*rhs.tvec.y + tmp.tvec.y;

        return *this;
    }

    Vector2D Transform2D::translation() const
    {
        Vector2D trans;
        trans.x = tvec.x;
        trans.y = tvec.y;

        return trans;
    }


    double Transform2D::rotation() const
    {
        return phi;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
    {
        os << "deg: " << rad2deg(tf.phi) << " x: " << tf.tvec.x << " y: " << tf.tvec.y;
        return os;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf)
    {
        double deg;
        Vector2D vec;

        if (is.peek() == 'd')
        {
            is.get();
            is.get();
            is.get();
            is.get();
            is >> deg;
            is.get();
            is.get();
            is.get();
            is >> vec.x;
            is.get();
            is.get();
            is.get();
            is >> vec.y;
        }
        else
        {
            is >> deg;
            is.get();
            is >> vec.x;
            is.get();
            is >> vec.y;
        }

        double rad = deg2rad(deg);

        Transform2D t = Transform2D(vec, rad);

        tf = t;

        std::cin.ignore(100, '\n');
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
    {
        return lhs*=rhs;
    }


}


int main(void) {
    // std::cout << turtlelib::deg2rad(180) << "\n";
    // struct turtlelib::Vector2D v;
    // v.x = 1.4;
    // v.y = 2.8;
    // std::cout << v << std::endl;

    // struct turtlelib::Vector2D w;
    // std::cin >> w;
    // std::cout << w << std::endl;

    // struct turtlelib::Twist2D t;
    // t.w = 0.9;
    // t.x = 1.6;
    // t.y = 1.2;
    // std::cout << t << std::endl;

    // struct turtlelib::Twist2D t1;
    // std::cin >> t1;
    // std::cout << t1 << std::endl;   

    struct turtlelib::Vector2D v;
    v.x = 0.;
    v.y = 1.;

    double rad = 3.14;
    turtlelib::Transform2D tf = turtlelib::Transform2D(v, rad);

    turtlelib::Transform2D x;
    std::cin >> x;
    std::cout << x << std::endl;

    std::cout << tf*x << std::endl;

    // turtlelib::Transform2D Tab(1.5708, v);

    return 0;
}