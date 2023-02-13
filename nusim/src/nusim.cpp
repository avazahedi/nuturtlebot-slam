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
///     arena
///         x_length: length of the arena in the world x direction
///         y_length: length of the arena in the world y direction
/// PUBLISHES:
///     walls (visualization_msgs/MarkerArray): publishes the marker array of the arena walls
///     obstacles (visualization_msgs/MarkerArray): publishes the marker array of all current
///                                                 obstacles every iteration
///     timestep (std_msgs/UInt64): publishes the current timestep every iteration
///     red/sensor_data (nuturtlebot_msgs/SensorData): publishes sensor data for the red turtlebot
/// SUBSCRIBES:
///     red/wheel_cmd (nuturtlebot_msgs/WheelCommands): subscribes to wheel positions for the
///                                                     red turtlebot
/// SERVERS:
///     reset (Empty): resets the timestep to 0 and teleports the robot back to its
///                    initial configuration
///     Teleport (nusim/srv/Teleport.srv): teleports the robot to the specified x, y, theta

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// #include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

/// \brief The NUSim class inherits the Node class and creates a simulated robot environment.
class NUSim : public rclcpp::Node
{
public:
  NUSim()
  : Node("nusim"), count_{0}
  {
    // rate parameter
    declare_parameter("rate", 200);      // default to 200 Hz

    int rate =
      get_parameter("rate").get_parameter_value().get<int>();

    // initial pose
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);

    x0 = get_parameter("x0").get_parameter_value().get<double>();
    y0 = get_parameter("y0").get_parameter_value().get<double>();
    theta0 = get_parameter("theta0").get_parameter_value().get<double>();

    // set initial values to pose
    x = x0;
    y = y0;
    theta = theta0;

    // obstacle lists parameters
    declare_parameter("obstacles.x", std::vector<double> {});
    declare_parameter("obstacles.y", std::vector<double> {});
    declare_parameter("obstacles.r", 0.5);

    obstacles_x = get_parameter("obstacles.x").get_parameter_value().get<std::vector<double>>();
    obstacles_y = get_parameter("obstacles.y").get_parameter_value().get<std::vector<double>>();
    if (obstacles_x.size() != obstacles_y.size()) {
      int error = 0;
      throw (error);
    }

    obstacles_r = get_parameter("obstacles.r").get_parameter_value().get<double>();

    const auto obstacles_z = 0.25;

    // diff drive parameters
    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);
    declare_parameter("motor_cmd_per_rad_sec", -1.0);
    declare_parameter("encoder_ticks_per_rad", -1.0);
    radius = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track = get_parameter("track_width").get_parameter_value().get<double>();
    motor_cmd_prs = get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    encoder_ticks = get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();
    if (radius == -1 || track == -1 || motor_cmd_prs == -1 || encoder_ticks == -1) {
      int error = 1;
      throw(error);
    }

    // arena size parameters
    declare_parameter("arena.x_length", 2.0);
    declare_parameter("arena.y_length", 2.0);
    arena_x = get_parameter("arena.x_length").get_parameter_value().get<double>();
    arena_y = get_parameter("arena.y_length").get_parameter_value().get<double>();

    turtlelib::DiffDrive dd {track, radius};

    prev_wheel_pos.left = 0.0;
    prev_wheel_pos.right = 0.0;

    dt = 1 / (static_cast<double>(rate));

    // walls publisher
    walls_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);

    // obstacles publisher
    obstacles_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);

    // timestep publisher
    timestep_pub_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    // red/sensor_data publisher
    sensor_pub_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);

    // robot path publisher
    robot_path_pub_ = create_publisher<nav_msgs::msg::Path>("red/path", 10);

    // // yellow relative markers publisher (V.1)
    // relative_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);

    // red/wheel_cmd subscriber
    wheelcmd_sub_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd",
      10, std::bind(&NUSim::wheelcmd_callback, this, std::placeholders::_1));

    // reset service
    reset_srv_ = create_service<std_srvs::srv::Empty>(
      "~/reset", std::bind(
        &NUSim::reset_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    // teleport service
    teleport_srv_ = create_service<nusim::srv::Teleport>(
      "~/teleport", std::bind(
        &NUSim::teleport_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    // transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // populate obstacles MarkerArray
    auto marker_stamp = get_clock()->now();
    for (unsigned int i = 0; i < obstacles_x.size(); i++) {
      visualization_msgs::msg::Marker obs;
      obs.header.frame_id = "nusim/world";
      obs.header.stamp = marker_stamp;
      obs.type = visualization_msgs::msg::Marker::CYLINDER;
      obs.id = i + 4; // i
      obs.action = visualization_msgs::msg::Marker::ADD;
      obs.scale.x = 2.0 * obstacles_r;
      obs.scale.y = 2.0 * obstacles_r;
      obs.scale.z = obstacles_z;
      obs.pose.position.x = obstacles_x.at(i);
      obs.pose.position.y = obstacles_y.at(i);
      obs.pose.position.z = obstacles_z / 2.0;
      obs.pose.orientation.x = 0.0;
      obs.pose.orientation.y = 0.0;
      obs.pose.orientation.z = 0.0;
      obs.pose.orientation.w = 1.0;
      obs.color.r = 1.0;
      obs.color.g = 0.0;
      obs.color.b = 0.0;
      obs.color.a = 1.0;
      obstacles_mkrs.markers.push_back(obs);
    }

    // walls
    create_wall(0.0, arena_y / 2.0, 0, 0);
    create_wall(0.0, -arena_y / 2.0, 0, 1);
    create_wall(arena_x / 2.0, 0.0, 1.5707, 2);
    create_wall(-arena_x / 2.0, 0.0, 1.5707, 3);


    // timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / rate), std::bind(&NUSim::timer_callback, this));
  }

private:
  /// \brief Timer callback that runs continuously on the provided frequency
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = count_;
    timestep_pub_->publish(message);
    count_++;

    //// SENSOR_DATA ////
    sensor_data.stamp = get_clock()->now();
    turtlelib::WheelPosn new_wheel_pos;
    turtlelib::WheelPosn wheel_diff;
    wheel_diff.left = vels.left * dt;
    wheel_diff.right = vels.right * dt;
    new_wheel_pos.left = prev_wheel_pos.left + (vels.left * dt);
    new_wheel_pos.right = prev_wheel_pos.right + (vels.right * dt);
    dd.ForwardKinematics(wheel_diff);
    sensor_data.left_encoder = new_wheel_pos.left * encoder_ticks;
    sensor_data.right_encoder = new_wheel_pos.right * encoder_ticks;
    turtlelib::RobotConfig q = dd.getConfig();
    theta = q.theta;
    x = q.x;
    y = q.y;
    // publish /sensor_data
    sensor_pub_->publish(sensor_data);
    // update prev
    prev_wheel_pos = new_wheel_pos;


    //// TRANSFORM ////
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";

    // Set transform translation
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    // Set transform rotation in quaternion
    tf2::Quaternion quat;
    quat.setRPY(0, 0, theta);
    t.transform.rotation.x = quat.x();
    t.transform.rotation.y = quat.y();
    t.transform.rotation.z = quat.z();
    t.transform.rotation.w = quat.w();

    // Add to robot path
    robot_path.header.stamp = get_clock()->now();
    robot_path.header.frame_id = "nusim/world";
    rp_pose.header.stamp = get_clock()->now();
    rp_pose.header.frame_id = "nusim/world";
    rp_pose.pose.position.x = x;
    rp_pose.pose.position.y = y;
    rp_pose.pose.position.z = 0.0;
    rp_pose.pose.orientation.x = quat.x();
    rp_pose.pose.orientation.y = quat.y();
    rp_pose.pose.orientation.z = quat.z();
    rp_pose.pose.orientation.w = quat.w();
    robot_path.poses.push_back(rp_pose);

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    // Publish to ~/obstacles
    obstacles_pub_->publish(obstacles_mkrs);

    // Publish to ~/walls
    walls_pub_->publish(wall_mkrs);

    // Publish robot_path
    robot_path_pub_->publish(robot_path);

  }

  /// @brief Callback for red/wheel_cmd subscription to receive motion commands
  /// @param msg - wheel commands
  void wheelcmd_callback(const nuturtlebot_msgs::msg::WheelCommands & msg)
  {
    vels.left = static_cast<double>(msg.left_velocity) * motor_cmd_prs;
    vels.right = static_cast<double>(msg.right_velocity) * motor_cmd_prs;
  }

  /// \brief Callback for reset service, which resets the timestep count and robot pose
  /// \param request - unused service request
  /// \param response - unused service response
  void reset_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    count_ = 0;
    x = x0;
    y = y0;
    theta = theta0;
  }

  /// \brief Callback for teleport service, which teleports the robot to the specified pose
  /// \param request - service request specifying what x, y, and theta to teleport to
  /// \param response - unused
  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    x = request->x;
    y = request->y;
    theta = request->theta;
  }

  /// @brief Function for creating the arena walls
  /// @param centerx - center of wall x-coordinate
  /// @param centery - center of wall y-coordinate
  /// @param angle - orientation
  /// @param i - wall ID
  void create_wall(double centerx, double centery, double angle, int i)
  {
    visualization_msgs::msg::Marker wall;
    wall.header.frame_id = "nusim/world";
    wall.header.stamp = get_clock()->now();
    wall.type = visualization_msgs::msg::Marker::CUBE;
    wall.id = i;
    wall.action = visualization_msgs::msg::Marker::ADD;
    if (angle == 0) {
      wall.scale.x = arena_x;
      wall.scale.y = 0.1;
    } else {
      wall.scale.x = 0.1;
      wall.scale.y = arena_y;
    }
    wall.scale.z = 0.25;
    wall.pose.position.x = centerx;
    wall.pose.position.y = centery;
    wall.pose.position.z = 0.25 / 2.0;
    wall.color.r = 1.0;
    wall.color.g = 0.0;
    wall.color.b = 0.0;
    wall.color.a = 1.0;
    wall_mkrs.markers.push_back(wall);
  }

  double x0;
  double y0;
  double theta0;
  unsigned int count_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr robot_path_pub_;

  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheelcmd_sub_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_srv_;

  // tf broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double x;
  double y;
  double theta;

  // obstacles
  std::vector<double> obstacles_x;
  std::vector<double> obstacles_y;
  double obstacles_r;
  visualization_msgs::msg::MarkerArray obstacles_mkrs;
  visualization_msgs::msg::MarkerArray wall_mkrs;

  // arena walls
  double arena_x;
  double arena_y;

  // 
  nuturtlebot_msgs::msg::SensorData sensor_data;
  double radius;
  double track;
  double motor_cmd_prs;
  double encoder_ticks;
  turtlelib::DiffDrive dd;
  turtlelib::WheelPosn prev_wheel_pos;
  turtlelib::WheelPosn vels;
  double dt;

  nav_msgs::msg::Path robot_path;
  geometry_msgs::msg::PoseStamped rp_pose;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<NUSim>());
  } catch (int error) {
    if (error == 0) {
      RCLCPP_ERROR(
        std::make_shared<NUSim>()->get_logger(),
        "Failed: Different number of obstacle x-coordinates than obstacle y-coordinates.");
    } else if (error == 1) {
      RCLCPP_ERROR(
        std::make_shared<NUSim>()->get_logger(),
        "Error: Necessary parameters not defined.");
    }
  }
  rclcpp::shutdown();
  return 0;
}
