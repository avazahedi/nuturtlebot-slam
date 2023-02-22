/// \file
/// \brief Two-dimensional rigid body transformations.

#include <iostream>
#include "turtlelib/rigid2d.hpp"

namespace turtlelib {

    // Vector2D

    Vector2D & Vector2D::operator+=(const Vector2D & rhs)
    {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator-=(const Vector2D & rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator*=(const double & scalar)
    {
        x *= scalar;
        y *= scalar;
        return *this;
    }

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

        is >> v.x >> v.y;
        
        is.clear();
        is.ignore(100, '\n');
        return is;
    }

    Vector2D operator+(Vector2D lhs, const Vector2D & rhs)
    {
        return lhs+=rhs;
    }

    Vector2D operator-(Vector2D lhs, const Vector2D & rhs)
    {
        return lhs-=rhs;
    }

    Vector2D operator*(Vector2D v, const double & scalar)
    {
        return v*=scalar;
    }

    Vector2D operator*(const double & scalar, Vector2D v)
    {
        return v*=scalar;
    }

    double dot(Vector2D v1, Vector2D v2)
    {
        double dp = v1.x*v2.x + v1.y*v2.y;
        return dp;
    }

    double magnitude(Vector2D v)
    {
        double mag = pow(pow(v.x, 2) + pow(v.y, 2), 0.5);
        return mag;
    }

    double angle(Vector2D v1, Vector2D v2)
    {
        double ang = atan2(v1.x*v2.y-v1.y*v2.x, v1.x*v2.x+v1.y*v2.y);
        return ang;
    }


    // Twist2D
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

        is >> t.w >> t.x >> t.y;

        is.clear();
        is.ignore(100, '\n');
        return is;
    }

    // Transform2D
    Transform2D::Transform2D(): tvec{0.0, 0.0}, phi{0.0} {}

    Transform2D::Transform2D(Vector2D trans): tvec{trans.x, trans.y}, phi{0.0} {}

    Transform2D::Transform2D(double radians): tvec{0.0, 0.0}, phi{radians} {}

    Transform2D::Transform2D(Vector2D trans, double radians): tvec{trans.x, trans.y}, phi{radians} {}

    Vector2D Transform2D::operator()(Vector2D v) const
    {
        return {cos(phi)*v.x - sin(phi)*v.y + tvec.x, sin(phi)*v.x + cos(phi)*v.y + tvec.y};
    }

    Transform2D Transform2D::inv() const
    {
        double new_phi = -phi;
        Vector2D new_v;
        new_v.x = -tvec.x*cos(phi)-tvec.y*sin(phi);
        new_v.y = -tvec.y*cos(phi)+tvec.x*sin(phi);

        Transform2D inverse {new_v, new_phi};

        return inverse;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {
        double tmp_phi = phi;
        double tmp_x = tvec.x;
        double tmp_y = tvec.y;

        phi = tmp_phi + rhs.phi;
        tvec.x = cos(tmp_phi)*rhs.tvec.x - sin(tmp_phi)*rhs.tvec.y + tmp_x;
        tvec.y = sin(tmp_phi)*rhs.tvec.x + cos(tmp_phi)*rhs.tvec.y + tmp_y;

        return *this;
    }

    Vector2D Transform2D::translation() const
    {
        return {tvec.x, tvec.y};
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
        double deg = 0.0;
        Vector2D vec{};

        if (is.peek() == 'd')
        {
            std::string s1;
            std::string s2;
            std::string s3;

            is >> s1 >> deg >> s2 >> vec.x >> s3 >> vec.y;
        }
        else
        {
            is >> deg >> vec.x >> vec.y;
        }

        const auto rad = deg2rad(deg);

        const auto t = Transform2D(vec, rad);

        tf = t;

        is.clear();
        is.ignore(100, '\n');
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
    {
        return lhs*=rhs;
    }

    Twist2D Transform2D::operator()(Twist2D t) const
    {
        return {t.w, tvec.y*t.w + cos(phi)*t.x - sin(phi)*t.y, -tvec.x*t.w + sin(phi)*t.x + cos(phi)*t.y};
    }

    Vector2D normalize(const Vector2D v)
    {
        const auto magnitude = pow(pow(v.x, 2) + pow(v.y, 2), 0.5);
        return {v.x / magnitude, v.y / magnitude};
    }

    Transform2D integrate_twist(Twist2D t)
    {
        Vector2D sb_v;

        if (t.w == 0)
        {
            sb_v.x = t.x;
            sb_v.y = t.y;
            Transform2D T_bbp = Transform2D(sb_v);
            return T_bbp;
        }
        else
        {
            sb_v.x = t.y/t.w;
            sb_v.y = -t.x/t.w;
            Transform2D Tsb = Transform2D(sb_v);
            Transform2D Tbs = Tsb.inv();
            Transform2D T_ssp = Transform2D(t.w);
            Transform2D T_bbp = Tbs*T_ssp*Tsb;
            return T_bbp;
        }

    }

}
