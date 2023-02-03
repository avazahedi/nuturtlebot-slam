/// \file
/// \brief The nusim node provides a simulated robot environment.
///
/// PARAMETERS:
///     rate (int): frequency of timer callback (defaults to 200 Hz)
///     x0: initial x-coordinate of the robot (m)
///     y0: initial y-coordinate of the robot (m)
///     theta0: initial rotation of the robot (rad)
///     obstacles
///         x: list of x-coordinates of obstacles (m)
///         y: list of y-coordinates of obstacles (m)
///         r: radius of obstacles (m)
/// PUBLISHES:
///     obstacles (visualization_msgs/MarkerArray): publishes the marker array of all current
///                                                 obstacles every iteration
///     timestep (std_msgs/UInt64): publishes the current timestep every iteration
/// SUBSCRIBES:
///     No subscriptions
/// SERVERS:
///     reset (Empty): resets the timestep to 0 and teleports the robot back to its
///                    initial configuration
///     Teleport (nusim/srv/Teleport.srv): teleports the robot to the specified x, y, theta
/// CLIENTS:
///     No clients

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
// #include "std_srvs/srv/empty.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"
// #include "tf2/LinearMath/Quaternion.h"
// #include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

/// \brief The NUSim class inherits the Node class and creates a simulated robot environment.
class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control"), count_{0}
  {
    declare_parameter("wheel_radius", -1);
    declare_parameter("track_width", -1);
    declare_parameter("motor_cmd_max", -1);
    declare_parameter("motor_cmd_per_rad_sec", -1);
    declare_parameter("encoder_ticks_per_rad", -1);
    declare_parameter("collision_radius", -1);

    double radius = get_parameter("wheel_radius").get_parameter_value().get<double>();
    double track = get_parameter("track_width").get_parameter_value().get<double>();
    int motor_cmd_max = get_parameter("motor_cmd_max").get_parameter_value().get<int>();
    int motor_cmd_prs = get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<int>();
    int encoder_ticks = get_parameter("encoder_ticks_per_rad").get_parameter_value().get<int>();
    double collision_rad = get_parameter("collision_radius").get_parameter_value().get<double>();

    if (radius==-1 || track==-1 || motor_cmd_max==-1 || motor_cmd_prs==-1 || encoder_ticks==-1 
        || collision_rad==-1)
    {
        int error = 0;
        throw(error);
    }



    // // obstacles publisher
    // obstacles_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);

    // // timestep publisher
    // timestep_pub_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    // // reset service
    // reset_srv_ = create_service<std_srvs::srv::Empty>(
    //   "~/reset", std::bind(
    //     &NUSim::reset_callback, this,
    //     std::placeholders::_1, std::placeholders::_2));

    // // teleport service
    // teleport_srv_ = create_service<nusim::srv::Teleport>(
    //   "~/teleport", std::bind(
    //     &NUSim::teleport_callback, this,
    //     std::placeholders::_1, std::placeholders::_2));

    // // transform broadcaster
    // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


    // timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(5), std::bind(&TurtleControl::timer_callback, this));
  }

private:
  /// \brief Timer callback that runs continuously on the provided frequency
  void timer_callback()
  {
    // auto message = std_msgs::msg::UInt64();
    // message.data = count_;
    // timestep_pub_->publish(message);
    // count_++;

    // geometry_msgs::msg::TransformStamped t;

    // // Read message content and assign it to
    // // corresponding tf variables
    // t.header.stamp = get_clock()->now();
    // t.header.frame_id = "nusim/world";
    // t.child_frame_id = "red/base_footprint";

    // // Set transform translation
    // t.transform.translation.x = x;
    // t.transform.translation.y = y;
    // t.transform.translation.z = 0.0;

    // // Set transform rotation in quaternion
    // tf2::Quaternion q;
    // q.setRPY(0, 0, theta);
    // t.transform.rotation.x = q.x();
    // t.transform.rotation.y = q.y();
    // t.transform.rotation.z = q.z();
    // t.transform.rotation.w = q.w();

    // // Send the transformation
    // tf_broadcaster_->sendTransform(t);

    // // Publish to ~/obstacles
    // obstacles_pub_->publish(obstacles_mkrs);

  }

  double x0;
  double y0;
  double theta0;
  unsigned int count_;
  rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
//   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub_;
//   rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
//   rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_srv_;
//   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
//   double x;
//   double y;
//   double theta;
//   std::vector<double> obstacles_x;
//   std::vector<double> obstacles_y;
//   double obstacles_r;
//   visualization_msgs::msg::MarkerArray obstacles_mkrs;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TurtleControl>());
  } catch (int error) {
    if (error==0)
    {
        RCLCPP_ERROR(
            std::make_shared<TurtleControl>()->get_logger(),
            "Error: Not all necessary parameters are defined.");
    }
  }
  rclcpp::shutdown();
  return 0;
}
