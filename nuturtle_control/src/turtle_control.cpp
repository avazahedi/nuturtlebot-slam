/// \file
/// \brief The turtle_control node enables control of the turtlebot.
///
/// PARAMETERS:
///     wheel_radius (double): wheel radius
///     track_width (double): track width (distance between the wheels)
///     motor_cmd_max (int): maximum ticks for motor commands
///     motor_cmd_per_rad_sec (double): number of rad/s equivalent to 1 motor command tick
///     encoder_ticks_per_rad (int): number of encoder ticks per radian
///     collision_radius (double): radius for collision detection
/// PUBLISHES:
///     wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): publishes left and right wheel velocities
///
///     joint_state (sensor_msgs::msg::JointState): publishes robot joint states
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs/Twist): velocity commands for the body of the robot
///
///     sensor_data (nuturtlebot_msgs::msg::SensorData): velocity of each wheel (ticks)

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

/// \brief The TurtleControl class inherits the Node class and enables control of the turtlebot 
///        via Twist messages on the cmd_vel topic.
class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {
    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);
    declare_parameter("motor_cmd_max", -1);
    declare_parameter("motor_cmd_per_rad_sec", -1.0);
    declare_parameter("encoder_ticks_per_rad", -1);
    declare_parameter("collision_radius", -1.0);

    radius = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track = get_parameter("track_width").get_parameter_value().get<double>();
    motor_cmd_max = get_parameter("motor_cmd_max").get_parameter_value().get<int>();
    motor_cmd_prs = get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    encoder_ticks = get_parameter("encoder_ticks_per_rad").get_parameter_value().get<int>();
    collision_rad = get_parameter("collision_radius").get_parameter_value().get<double>();

    if (radius==-1 || track==-1 || motor_cmd_max==-1 || motor_cmd_prs==-1 || encoder_ticks==-1 
        || collision_rad==-1)
    {
        int error = 0;
        throw(error);
    }

    // time0 = -1.0;

    // initialize positions and velocities to 0
    joint_state.position = {0.0, 0.0};
    joint_state.velocity = {0.0, 0.0};

    // subscriptions
    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, 
                 std::bind(&TurtleControl::cmdvel_callback, this, std::placeholders::_1));

    sensor_sub_ = create_subscription<nuturtlebot_msgs::msg::SensorData>("/sensor_data", 10,
                  std::bind(&TurtleControl::sensor_callback, this, std::placeholders::_1));

    // publishers
    wheel_cmd_pub_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("/wheel_cmd", 10);

    joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&TurtleControl::timer_callback, this));
  }

private:
  /// \brief Timer callback that runs continuously on the provided frequency
  void timer_callback(){}

  /// @brief Callback for subscription to /cmd_vel
  /// @param msg - Twist from /cmd_vel
  void cmdvel_callback(const geometry_msgs::msg::Twist & msg)
  {
    twist.w = msg.angular.z;
    twist.x = msg.linear.x;
    twist.y = msg.linear.y;

    turtlelib::DiffDrive dd {track, radius};

    wheels = dd.InverseKinematics(twist);

    // convert to motor ticks
    wheel_msg.left_velocity = (int32_t)wheels.left/motor_cmd_prs;
    wheel_msg.right_velocity = (int32_t)wheels.right/motor_cmd_prs;
    if (wheel_msg.left_velocity > motor_cmd_max)
    {
      wheel_msg.left_velocity = motor_cmd_max;
    }
    else if (wheel_msg.left_velocity < -motor_cmd_max)
    {
      wheel_msg.left_velocity = -motor_cmd_max;
    }
    if (wheel_msg.right_velocity > motor_cmd_max)
    {
      wheel_msg.right_velocity = motor_cmd_max;
    }
    else if (wheel_msg.right_velocity < -motor_cmd_max)
    {
      wheel_msg.right_velocity = -motor_cmd_max;
    }

    wheel_cmd_pub_->publish(wheel_msg);
  }

  /// @brief Callback for subscription to /sensor_data
  /// @param msg - SensorData from /sensor_data
  void sensor_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    joint_state.name = {"wheel_left_joint", "wheel_right_joint"};
    joint_state.header.stamp = msg.stamp;
    // joint_state.header.frame_id = '';

    if (time0 == -1)
    {
      // initialize positions and velocities to 0
      joint_state.position = {0.0, 0.0};
      joint_state.velocity = {0.0, 0.0};
    }
    else{
      // compute dphi
      double dt = msg.stamp.sec + 1e-9*msg.stamp.nanosec - time0;
      // RCLCPP_INFO_STREAM(get_logger(), "dt: " << dt);
      double dphi_l = (double)msg.left_encoder/encoder_ticks;
      double dphi_r = (double)msg.right_encoder/encoder_ticks;
      // RCLCPP_INFO_STREAM(get_logger(), "sensor data: " << dphi_l << " " << dphi_r);
      joint_state.position = {dphi_l,
                              dphi_r};
      joint_state.velocity = {dphi_l/dt, dphi_r/dt};
    }
    // update "previous" time stamp
    time0 = msg.stamp.sec + 1e-9*msg.stamp.nanosec;
    // RCLCPP_INFO_STREAM(get_logger(), "joint states: " << joint_state.position.at(0) << " " << joint_state.velocity.at(0));
    joint_states_pub_->publish(joint_state);
  }

  double radius;
  double track;
  int motor_cmd_max;
  double motor_cmd_prs;
  int encoder_ticks;
  double collision_rad;
  double time0;

  turtlelib::Twist2D twist;
  turtlelib::WheelPosn wheels;
  nuturtlebot_msgs::msg::WheelCommands wheel_msg;
  sensor_msgs::msg::JointState joint_state;
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_sub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
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
