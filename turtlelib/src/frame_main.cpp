#include <iostream>
#include "turtlelib/rigid2d.hpp"

int main() {

    // Transform2D operations
    turtlelib::Transform2D Tab;
    turtlelib::Transform2D Tbc;

    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> Tab;
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> Tbc;

    turtlelib::Transform2D Tac = Tab*Tbc;
    turtlelib::Transform2D Tba = Tab.inv();
    turtlelib::Transform2D Tcb = Tbc.inv();
    turtlelib::Transform2D Tca = Tac.inv();
    
    std::cout << "T_{a,b}: " << Tab << std::endl;
    std::cout << "T_{b,a}: " << Tba << std::endl;
    std::cout << "T_{b,c}: " << Tbc << std::endl;
    std::cout << "T_{c,b}: " << Tcb << std::endl;
    std::cout << "T_{a,c}: " << Tac << std::endl;
    std::cout << "T_{c,a}: " << Tca << std::endl;

    // Vector2D operations
    turtlelib::Vector2D v_b;

    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> v_b;
    std::cout << "v_bhat: " << turtlelib::normalize(v_b) << std::endl;
    std::cout << "v_a: " << Tab(v_b) << std::endl;
    std::cout << "v_b: " << v_b << std::endl;
    std::cout << "v_c: " << Tcb(v_b) << std::endl;

    // Twist2D operations
    turtlelib::Twist2D V_b;
    std::cout << "Enter twist V_b:" << std::endl;
    std::cin >> V_b;
    std::cout << "V_a " << Tab(V_b) << std::endl;
    std::cout << "V_b " << V_b << std::endl;
    std::cout << "V_c " << Tcb(V_b) << std::endl;

    return 0;
}