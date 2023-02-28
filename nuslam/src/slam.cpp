/// \file
/// \brief The odometry node publishes odometry messages and the odometry transform.
///
/// PARAMETERS:
///     wheel_radius (double): wheel radius
///     track_width (double): track width (distance between the wheels)
///     body_id (string): name of the body frame of the robot
///     odom_id (string): name of the odometry frame
///     wheel_left (string): name of the left wheel joint
///     wheel_right (string): name of the right wheel joint
/// PUBLISHES:
///     odom (nav_msgs::msg::Odometry): publishes odometry
/// SUBSCRIBES:
///     joint_states (sensor_msgs/JointState): joint states of the robot
/// SERVICES:
///     initial_pose (nuturtle_control/InitialPose.srv): resets the location of the odometry

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
// #include "turtlelib/diff_drive.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "turtlelib/kalman.hpp"
#include <armadillo>

using namespace std::chrono_literals;

/// \brief The Slam class inherits the Node class and publishes the map to green/odom transform.
class Slam : public rclcpp::Node
{
public:
  Slam()
  : Node("slam")
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

    if (wheel_left == "" || wheel_right == "" || body_id == "" || radius == -1 || track == -1) {
      auto error = std::invalid_argument("Required parameters not set.");
      throw(error);
    }

    turtlelib::DiffDrive dd {track, radius};

    prev_wheel_pos.left = 0.0;
    prev_wheel_pos.right = 0.0;

    odom_msg.header.frame_id = odom_id;
    odom_msg.child_frame_id = body_id;

    // odom publisher
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("green/odom", 10);

    // odom path publisher
    odom_path_pub_ = create_publisher<nav_msgs::msg::Path>("green/path", 10);

    // joint_states subscriber
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&Slam::js_callback, this, std::placeholders::_1));

    // fake_sensor subscriber
    fs_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/fake_sensor", 10,
      std::bind(&Slam::fs_callback, this, std::placeholders::_1));

    // initial pose service
    initial_pose_srv_ = create_service<nuturtle_control::srv::InitialPose>(
      "initial_pose", std::bind(
        &Slam::initpose_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    // transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // tf broadcaster between map and odom
    tfmo_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  }

private:
  /// @brief Callback for fake_sensor subscription
  /// @param msg - fake sensor MarkerArray
  void fs_callback(const visualization_msgs::msg::MarkerArray & msg)
  {
    auto obstacles = msg.markers;

    auto q = dd.getConfig();
    ekf.setConfig(q);
    // RCLCPP_INFO_STREAM(get_logger(), "xi_est BEFORE PREDICT:\n" << ekf.getStateEst());
    ekf.predict();

    for (unsigned int j=0; j<obstacles.size(); j++)
    {
        if (obstacles.at(j).action == 0)   // 0 = add, 2 = delete
        {
            // RCLCPP_INFO_STREAM(get_logger(), "xi_est AFTER PREDICT:\n" << ekf.getStateEst());
            ekf.update(obstacles.at(j).pose.position.x, obstacles.at(j).pose.position.y, j);
        }
    }

    arma::vec xi = ekf.getStateEst();
    turtlelib::Vector2D trans{xi(1), xi(2)};
    Tmr = turtlelib::Transform2D(trans,turtlelib::normalize_angle(xi(0)));

  }

  /// @brief Callback for joint_state subscription
  /// @param msg - joint state data
  void js_callback(const sensor_msgs::msg::JointState & msg)
  {
    count_++;

    // new wheel posns
    turtlelib::WheelPosn wheels;
    wheels.left = msg.position.at(0) - prev_wheel_pos.left;
    wheels.right = msg.position.at(1) - prev_wheel_pos.right;
    // update previous wheel posns
    prev_wheel_pos = {msg.position.at(0), msg.position.at(1)};
    // calculate twist and FK
    turtlelib::Twist2D Vb = dd.getTwist(wheels);
    dd.ForwardKinematics(wheels);
    turtlelib::RobotConfig q = dd.getConfig();

    // odom_msg
    odom_msg.header.stamp = get_clock()->now();
    // odom_msg.header.stamp = msg.header.stamp;
    odom_msg.pose.pose.position.x = q.x;
    odom_msg.pose.pose.position.y = q.y;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, q.theta);
    odom_msg.pose.pose.orientation.x = quat.x();
    odom_msg.pose.pose.orientation.y = quat.y();
    odom_msg.pose.pose.orientation.z = quat.z();
    odom_msg.pose.pose.orientation.w = quat.w();
    odom_msg.twist.twist.angular.z = Vb.w;
    odom_msg.twist.twist.linear.x = Vb.x;
    odom_msg.twist.twist.linear.y = Vb.y;


    // update the transform
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = get_clock()->now();
    t.header.frame_id = odom_id;
    t.child_frame_id = body_id;

    // Set transform
    t.transform.translation.x = q.x;
    t.transform.translation.y = q.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = quat.x();
    t.transform.rotation.y = quat.y();
    t.transform.rotation.z = quat.z();
    t.transform.rotation.w = quat.w();

    // update transformation matrix
    Tor = turtlelib::Transform2D( turtlelib::Vector2D{q.x,q.y}, q.theta );

    Tmo = Tmr*(Tor.inv());

    // tf between map and green/odom //
    t2.header.stamp = get_clock()->now();
    t2.header.frame_id = "map";
    t2.child_frame_id = odom_id;

    t2.transform.translation.x = Tmo.translation().x;
    t2.transform.translation.y = Tmo.translation().y;
    t2.transform.translation.z = 0.0;

    tf2::Quaternion quat2;
    quat2.setRPY(0, 0, Tmo.rotation());
    t2.transform.rotation.x = quat2.x();
    t2.transform.rotation.y = quat2.y();
    t2.transform.rotation.z = quat2.z();
    t2.transform.rotation.w = quat2.w();


    if (count_%100 == 0)
    {
      // Add to odom path
      odom_path.header.stamp = get_clock()->now();
      odom_path.header.frame_id = odom_id;
      rp_pose.header.stamp = get_clock()->now();
      rp_pose.header.frame_id = odom_id;
      rp_pose.pose.position.x = q.x;
      rp_pose.pose.position.y = q.y;
      rp_pose.pose.position.z = 0.0;
      rp_pose.pose.orientation.x = quat.x();
      rp_pose.pose.orientation.y = quat.y();
      rp_pose.pose.orientation.z = quat.z();
      rp_pose.pose.orientation.w = quat.w();
      odom_path.poses.push_back(rp_pose);
    }

    // publish odom path
    odom_path_pub_->publish(odom_path);

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    // publish odom
    odom_pub_->publish(odom_msg);

    // tf between map and green/odom
    tfmo_broadcaster_->sendTransform(t2);
  }

  /// \brief Callback for initial_pose service, which resets the location of the odometry
  /// \param request - service request providing the configuration of the robot
  /// \param response - unused
  void initpose_callback(
    const std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
    std::shared_ptr<nuturtle_control::srv::InitialPose::Response>)
  {
    turtlelib::RobotConfig cfg;
    cfg.theta = request->theta;
    cfg.x = request->x;
    cfg.y = request->y;

    dd.setConfig(cfg);
  }

  unsigned int count_ = 0;

  std::string body_id;
  std::string odom_id;
  std::string wheel_left;
  std::string wheel_right;
  double radius;
  double track;
  turtlelib::DiffDrive dd;
  turtlelib::WheelPosn prev_wheel_pos;

  // SLAM
  turtlelib::EKF ekf;
  turtlelib::Transform2D Tor;
  geometry_msgs::msg::TransformStamped t2;
  turtlelib::Transform2D Tmr;
  turtlelib::Transform2D Tmo;


  nav_msgs::msg::Odometry odom_msg;
  nav_msgs::msg::Path odom_path;
  geometry_msgs::msg::PoseStamped rp_pose;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fs_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_pub_;

  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_srv_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfmo_broadcaster_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Slam>());
  } catch (std::exception &e) {
    RCLCPP_ERROR(std::make_shared<Slam>()->get_logger(),
    "Error: Not all necessary parameters are defined.");
  }
  rclcpp::shutdown();
  return 0;
}
