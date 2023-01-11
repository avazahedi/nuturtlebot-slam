/// \file
/// \brief Two-dimensional rigid body transformations.

#include <iostream>
#include "rigid2d.hpp"

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
        is.get();

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
        is.get();

        return is;
    }

}


int main(void) {
    std::cout << turtlelib::deg2rad(180) << "\n";

    struct turtlelib::Vector2D v;
    v.x = 1.4;
    v.y = 2.8;

    std::cout << v << std::endl;

    struct turtlelib::Vector2D w;
    std::cin >> w;

    std::cout << w << std::endl;

    struct turtlelib::Twist2D t;
    t.w = 0.9;
    t.x = 1.6;
    t.y = 1.2;

    std::cout << t << std::endl;

    struct turtlelib::Twist2D t1;
    std::cin >> t1;

    std::cout << t1 << std::endl;   

    return 0;
}