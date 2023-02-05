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
// #include "geometry_msgs/msg/twist.hpp"
// #include "nuturtlebot_msgs/msg/wheel_commands.hpp"
// #include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nav_msgs/msg/odometry.hpp"
// #include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

/// \brief The NUSim class inherits the Node class and creates a simulated robot environment.
class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {
    declare_parameter("body_id", "");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", "");
    declare_parameter("wheel_right", "");

    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);
    radius = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track = get_parameter("track_width").get_parameter_value().get<double>();

    body_id = get_parameter("body_id").get_parameter_value().get<std::string>();
    odom_id = get_parameter("odom_id").get_parameter_value().get<std::string>();
    wheel_left = get_parameter("wheel_left").get_parameter_value().get<std::string>();
    wheel_right = get_parameter("wheel_right").get_parameter_value().get<std::string>();

    if (wheel_left == "" || wheel_right == "" || body_id == "" || radius == -1 || track == -1)
    {
        int error = 0;
        throw(error);
    }

    turtlelib::DiffDrive dd {track, radius};

    // publisher
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // subscriber
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_state", 10,
                  std::bind(&Odometry::js_callback, this, std::placeholders::_1));

    // transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


    // timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&Odometry::timer_callback, this));
  }

private:
  /// \brief Timer callback that runs continuously on the provided frequency
  void timer_callback()
  {

    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = get_clock()->now();
    t.header.frame_id = odom_id;
    t.child_frame_id = body_id;

    // Set transform translation
    turtlelib::RobotConfig q = dd.getConfig();
    t.transform.translation.x = q.x;
    t.transform.translation.y = q.y;
    t.transform.translation.z = 0.0;

    // Set transform rotation in quaternion
    tf2::Quaternion quat;
    quat.setRPY(0, 0, q.theta);
    t.transform.rotation.x = quat.x();
    t.transform.rotation.y = quat.y();
    t.transform.rotation.z = quat.z();
    t.transform.rotation.w = quat.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    // odom_msg
    odom_msg.header.stamp = get_clock()->now();
    odom_msg.header.frame_id = odom_id;
    odom_msg.child_frame_id = body_id;
    odom_msg.pose.pose.position.x = q.x;
    odom_msg.pose.pose.position.y = q.y;
    odom_msg.pose.pose.orientation.x = quat.x();
    odom_msg.pose.pose.orientation.y = quat.y();
    odom_msg.pose.pose.orientation.z = quat.z();
    odom_msg.pose.pose.orientation.w = quat.w();

    // publishers
    odom_pub_->publish(odom_msg);

  }

  void js_callback(const sensor_msgs::msg::JointState & msg)
  {
    turtlelib::WheelPosn wheels;
    if (msg.position.size() > 2)
    {
        wheels.left = msg.position[0];
        wheels.right = msg.position[1];
        dd.ForwardKinematics(wheels);  
    }  
  }

  std::string body_id;
  std::string odom_id;
  std::string wheel_left;
  std::string wheel_right;
  double radius;
  double track;
  turtlelib::DiffDrive dd;

//   turtlelib::Twist2D twist;
//   turtlelib::WheelPosn wheels;
//   nuturtlebot_msgs::msg::WheelCommands wheel_msg;
//   sensor_msgs::msg::JointState joint_state;
  nav_msgs::msg::Odometry odom_msg;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  
//   rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
//   rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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
    rclcpp::spin(std::make_shared<Odometry>());
  } catch (int error) {
    if (error==0)
    {
        RCLCPP_ERROR(
            std::make_shared<Odometry>()->get_logger(),
            "Error: Required parameters not specified.");
    }
  }
  rclcpp::shutdown();
  return 0;
}
