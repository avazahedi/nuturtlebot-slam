/// \file
/// \brief The circle node publishes cmd_vel commands to cause the robot to drive in a circle.
///
/// PARAMETERS:
///     wheel_radius (double): wheel radius
///     track_width (double): track width (distance between the wheels)
///     frequency (int): frequency of publishing cmd_vel commands
/// PUBLISHES:
///     cmd_vel (geometry_msgs::msg::Twist): publishes cmd_vel commands
/// SERVICES:
///     control (nuturtle_control/Control.srv): controls the robot angular velocity and arc radius
///     reverse (std_srvs/Empty.srv): reverses the direction of robot movement
///     stop (std_srvs/Empty.srv): stops the robot

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nuturtle_control/srv/control.hpp"

using namespace std::chrono_literals;

/// \brief The Circle class inherits the Node class and publishes cmd_vel commands to cause the
///        robot to drive in a circle of a specified radius at a specified speed.
class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle")
  {
    declare_parameter("frequency", 100);
    frequency = get_parameter("frequency").get_parameter_value().get<int>();

    stop_flag = 0;

    // publisher
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // services
    control_srv_ = create_service<nuturtle_control::srv::Control>(
      "control", std::bind(
        &Circle::control_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    reverse_srv_ = create_service<std_srvs::srv::Empty>(
      "reverse", std::bind(
        &Circle::reverse_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    stop_srv_ = create_service<std_srvs::srv::Empty>(
      "stop", std::bind(
        &Circle::stop_callback, this,
        std::placeholders::_1, std::placeholders::_2));


    // timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000/frequency), std::bind(&Circle::timer_callback, this));
  }

private:
  /// \brief Timer callback that runs continuously on the provided frequency
  void timer_callback()
  {
    if (stop_flag == 0)
    {
        cmd_vel_pub_->publish(vel_msg);
    }
  }

  /// \brief Callback for control service, which publishes controls to cmd_vel
  /// \param request - service request providing the velocity and radius
  /// \param response - unused
  void control_callback(
    const std::shared_ptr<nuturtle_control::srv::Control::Request> request,
    std::shared_ptr<nuturtle_control::srv::Control::Response>)
  {
    stop_flag = 0;
    vel_msg.angular.z = request->velocity;
    vel_msg.linear.x = request->radius * request->velocity;
  }

  /// @brief Callback for the reverse service, which reverses the direction of the robot
  /// @param  request - unused
  /// @param  response - unused
  void reverse_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    vel_msg.angular.z *= -1;
    vel_msg.linear.x *= -1;
  }

  /// @brief Callback for the stop service, which stops the robot
  /// @param request - unused
  /// @param response - unused
  void stop_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    geometry_msgs::msg::Twist t;
    cmd_vel_pub_->publish(t);
    stop_flag = 1;  // stop the robot
  }

  int frequency;
  int stop_flag;

  geometry_msgs::msg::Twist vel_msg;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
