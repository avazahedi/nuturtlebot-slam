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
///     input_noise (double): variance for the zero mean Gaussian noise
///     slip_fraction (double): wheel slippage
///     basic_sensor_variance (double): sensor variance
///     max_range (double): max range for detecting obstacles
///     
///     Lidar parameters:
///     range_min (double): lidar range minimum
///     range_max (double): lidar range maximum
///     angle_increment (double): lidar angle increment
///     num_samples (int): number of samples collected by the lidar per timer iteration
///     resolution (double): resolution of lidar sensor measurements
///     noise_level (double): lidar sensor noise
///
///     DiffDrive parameters:
///     wheel_radius (double): wheel radius
///     track_width (double): track width (distance between the wheels)
///     motor_cmd_max (int): maximum ticks for motor commands
///     motor_cmd_per_rad_sec (double): number of rad/s equivalent to 1 motor command tick
///     encoder_ticks_per_rad (double): number of encoder ticks per radian
///     collision_radius (double): radius for collision detection
/// PUBLISHES:
///     walls (visualization_msgs/MarkerArray): publishes the marker array of the arena walls
///     obstacles (visualization_msgs/MarkerArray): publishes the marker array of all current
///                                                 obstacles every iteration
///     timestep (std_msgs/UInt64): publishes the current timestep every iteration
///     red/sensor_data (nuturtlebot_msgs/SensorData): publishes sensor data for the red turtlebot
///     red/path (nav_msgs/Path): publishes the path of the red (simulated) turtlebot
///     fake_sensor (visualization_msgs/MarkerArray): publishes the simulated sensor data of 
///                                                   the obstacles
///     lidar_sim (sensor_msgs/LaserScan): publishes the simulated lidar data
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
#include <random>
#include <cmath>

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
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

// random number generation (from Jointly Gaussian Distributions notes)
std::mt19937 & get_random()
{
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{}; 
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
}

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
    
    // noise parameters
    declare_parameter("input_noise", 0.0);
    declare_parameter("slip_fraction", 0.0);
    declare_parameter("basic_sensor_variance", 0.0);
    declare_parameter("max_range", 3.0);  // max range for detecting obstacles

    input_noise = get_parameter("input_noise").get_parameter_value().get<double>();
    slip_fraction = get_parameter("slip_fraction").get_parameter_value().get<double>();
    basic_sensor_variance = get_parameter("basic_sensor_variance").get_parameter_value().get<double>();
    max_range = get_parameter("max_range").get_parameter_value().get<double>();

    // lidar parameters
    declare_parameter("range_min", 0.120);
    declare_parameter("range_max", 3.5);
    declare_parameter("angle_increment", 0.0174533);
    declare_parameter("num_samples", 360);
    declare_parameter("resolution", 1.0);
    declare_parameter("noise_level", 0.01); // stddev = 0.01
    lidar_range_min = get_parameter("range_min").get_parameter_value().get<double>();
    lidar_range_max = get_parameter("range_max").get_parameter_value().get<double>();
    angle_incr = get_parameter("angle_increment").get_parameter_value().get<double>();
    num_samples = get_parameter("num_samples").get_parameter_value().get<int>();
    resolution = get_parameter("resolution").get_parameter_value().get<double>();
    noise_level = get_parameter("noise_level").get_parameter_value().get<double>();

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
    obstacles_z = 0.25;

    // diff drive parameters
    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);
    declare_parameter("motor_cmd_per_rad_sec", -1.0);
    declare_parameter("encoder_ticks_per_rad", -1.0);
    declare_parameter("collision_radius", -1.0);
    radius = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track = get_parameter("track_width").get_parameter_value().get<double>();
    motor_cmd_prs = get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    encoder_ticks = get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();
    collision_rad = get_parameter("collision_radius").get_parameter_value().get<double>();
    if (radius == -1 || track == -1 || motor_cmd_prs == -1 || encoder_ticks == -1 
        || collision_rad == -1)
    {
      int error = 1;
      throw(error);
    }

    // arena size parameters
    declare_parameter("arena.x_length", 2.0);
    declare_parameter("arena.y_length", 2.0);
    arena_x = get_parameter("arena.x_length").get_parameter_value().get<double>();
    arena_y = get_parameter("arena.y_length").get_parameter_value().get<double>();
    wall_thickness = 0.1;

    // diff drive
    turtlelib::DiffDrive dd {track, radius};

    prev_wheel_pos.left = 0.0;
    prev_wheel_pos.right = 0.0;

    dt = 1 / (static_cast<double>(rate));

    // normal distribution Gaussian variable
    ndist_pos = std::normal_distribution<>(0.0, pow(input_noise, 0.5));
    ndist_fs = std::normal_distribution<>(0.0, pow(basic_sensor_variance, 0.5));
    ndist_lidar = std::normal_distribution<>(0.0, pow(noise_level, 0.5));
    // uniform distribution
    udist_pos = std::uniform_real_distribution<>(-slip_fraction, slip_fraction);

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

    // fake sensor publisher
    fake_sensor_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("fake_sensor", 10);

    // simulated lidar publisher
    lidar_sim_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("lidar_sim", 10);

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

    // populate obstacles MarkerArray and vectors
    auto marker_stamp = get_clock()->now();
    for (unsigned int i = 0; i < obstacles_x.size(); i++) {
      visualization_msgs::msg::Marker obs;
      obs.header.frame_id = "nusim/world";
      obs.header.stamp = marker_stamp;
      obs.type = visualization_msgs::msg::Marker::CYLINDER;
      obs.id = i + 4; // ids 4-7
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

      turtlelib::Vector2D vec {obstacles_x.at(i), obstacles_y.at(i)};
      obs_vecs.push_back(vec);
    }

    // create walls
    create_wall(0.0, arena_y / 2.0, 0, 0);
    create_wall(0.0, -arena_y / 2.0, 0, 1);
    create_wall(arena_x / 2.0, 0.0, 1.5707, 2);
    create_wall(-arena_x / 2.0, 0.0, 1.5707, 3);

    //// TIMERS ////
    // timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / rate), std::bind(&NUSim::timer_callback, this));

    // fake sensor timer
    fs_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / 5), std::bind(&NUSim::fstimer_callback, this));
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

    double sfl = 0.0;
    double sfr = 0.0;
    if (slip_fraction != 0) 
    {
      sfl = udist_pos(get_random());
      sfr = udist_pos(get_random());
    }

    wheel_diff.left = vels.left * dt * (1.0+sfl);
    wheel_diff.right = vels.right * dt * (1.0+sfr);
    new_wheel_pos.left = prev_wheel_pos.left + (vels.left * dt);
    new_wheel_pos.right = prev_wheel_pos.right + (vels.right * dt);
    dd.ForwardKinematics(wheel_diff);
    // Collision detection
    collision_detect();
    // update config
    turtlelib::RobotConfig q = dd.getConfig();
    theta = q.theta;
    x = q.x;
    y = q.y;
    // update sensor data
    sensor_data.left_encoder = new_wheel_pos.left * encoder_ticks;
    sensor_data.right_encoder = new_wheel_pos.right * encoder_ticks;
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
    if (count_%100 == 0)
    {
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
    }

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    // Publish to ~/obstacles
    obstacles_pub_->publish(obstacles_mkrs);

    // Publish to ~/walls
    walls_pub_->publish(wall_mkrs);

    // Publish robot_path
    robot_path_pub_->publish(robot_path);

  }

  /// @brief 5Hz timer callback for simulation data
  void fstimer_callback()
  {
    // publish to /fake_sensor at a frequency of 5 Hz
    visualization_msgs::msg::MarkerArray fake_sensor_data;

    turtlelib::Vector2D robot_pos {dd.getConfig().x, dd.getConfig().y};
    double robot_phi = dd.getConfig().theta;
    turtlelib::Transform2D tf_world_robot {robot_pos, robot_phi};
    turtlelib::Transform2D tf_robot_world = tf_world_robot.inv();

    auto marker_stamp = get_clock()->now();
    for (unsigned int i = 0; i < obstacles_x.size(); i++) {
      turtlelib::Vector2D obs_vec = {obstacles_x.at(i), obstacles_y.at(i)};
      turtlelib::Vector2D rel_obs = tf_robot_world(obs_vec);

      visualization_msgs::msg::Marker fake_obs;
      fake_obs.header.frame_id = "red/base_footprint";
      fake_obs.header.stamp = marker_stamp;
      fake_obs.type = visualization_msgs::msg::Marker::CYLINDER;
      fake_obs.id = i + 8; // ids 8-11
      auto dist = pow(pow(rel_obs.x,2) + pow(rel_obs.y,2),0.5);
      if (dist > max_range)
      {
        fake_obs.action = visualization_msgs::msg::Marker::DELETE;
      }
      else
      {
        fake_obs.action = visualization_msgs::msg::Marker::ADD;
      }
      fake_obs.scale.x = 2.0 * obstacles_r;
      fake_obs.scale.y = 2.0 * obstacles_r;
      fake_obs.scale.z = obstacles_z;
      fake_obs.pose.position.x = rel_obs.x + ndist_fs(get_random());
      fake_obs.pose.position.y = rel_obs.y + ndist_fs(get_random());
      fake_obs.pose.position.z = obstacles_z / 2.0;
      fake_obs.pose.orientation.x = 0.0;
      fake_obs.pose.orientation.y = 0.0;
      fake_obs.pose.orientation.z = 0.0;
      fake_obs.pose.orientation.w = 1.0;
      fake_obs.color.r = 1.0;
      fake_obs.color.g = 1.0;
      fake_obs.color.b = 0.0;
      fake_obs.color.a = 1.0;
      fake_sensor_data.markers.push_back(fake_obs);
    }

    // simulated lidar data
    sim_lidar();

    // publish simulated data
    fake_sensor_pub_->publish(fake_sensor_data);

    lidar_sim_pub_->publish(lidar_sim_data);
  }

  /// @brief Simulate lidar data
  void sim_lidar()
  {
    std::vector<float> ranges(num_samples); // ranges for LaserScan msg
    turtlelib::RobotConfig q = dd.getConfig();

    // scan for obstacles
    for (unsigned int t=0; t<num_samples; t++)  // each angle increment
    {
      double max_x = q.x + cos(t*angle_incr+q.theta)*max_range;
      double max_y = q.y + sin(t*angle_incr+q.theta)*max_range;
      double m = (max_y-q.y)/(max_x-q.x);
      double min_dist = max_range+1.0;
      for (unsigned int i=0; i<obstacles_x.size(); i++) // each obstacle
      {
        double alpha = q.y-m*q.x-obstacles_y.at(i);
        double a = 1.0 + pow(m,2);
        double b = 2.0*(alpha*m - obstacles_x.at(i));
        double c = pow(obstacles_x.at(i),2) + pow(alpha,2) - pow(obstacles_r,2);

        double det = pow(b,2)-4*a*c;
        if (det == 0)
        {
          double x = -b/(2*a);  // x soln
          double y = m*(x-q.x)+q.y; // y soln
          double dist = turtlelib::distance_btw(x, y, q.x, q.y);
          if (dist < min_dist)
          {
            if ((x-q.x)/(max_x-q.x) > 0 && (y-q.y)/(max_y-q.y) > 0)
            {
              ranges.at(t) = dist;
              min_dist = dist;
            }
          }
        }
        else if (det > 0)
        {
          double x1 = (-b+std::sqrt(det)) / (2*a);
          double x2 = (-b-std::sqrt(det)) / (2*a);
          double y1 = m*(x1-q.x)+q.y;
          double y2 = m*(x2-q.x)+q.y;
          double dist1 = turtlelib::distance_btw(x1, y1, q.x, q.y);
          double dist2 = turtlelib::distance_btw(x2, y2, q.x, q.y);
          double dist = std::min(dist1, dist2);
          if (dist < min_dist)
          {
            if (dist == dist1 && (x1-q.x)/(max_x-q.x) > 0 && (y1-q.y)/(max_y-q.y) > 0)
            {
              ranges.at(t) = dist + ndist_lidar(get_random());
              min_dist = dist;
            }
            else if ((x2-q.x)/(max_x-q.x) > 0 && (y2-q.y)/(max_y-q.y) > 0)  // dist = dist2
            {
              ranges.at(t) = dist + ndist_lidar(get_random());
              min_dist = dist;
            }
          }
        }
      }

      // scan for walls
      double w1x = arena_x/2.0 - wall_thickness/2.0;
      double w1y = m*(w1x-q.x)+q.y;
      double w1_dist = turtlelib::distance_btw(w1x, w1y, q.x, q.y);
      double w2x = -arena_x/2.0 + wall_thickness/2.0;
      double w2y = m*(w2x-q.x)+q.y;
      double w2_dist = turtlelib::distance_btw(w2x, w2y, q.x, q.y);
      double w3y = arena_y/2.0 - wall_thickness/2.0;
      double w3x = (w3y-q.y)/m + q.x;
      double w3_dist = turtlelib::distance_btw(w3x, w3y, q.x, q.y);
      double w4y = -arena_y/2.0 + wall_thickness/2.0;
      double w4x = (w4y-q.y)/m + q.x;
      double w4_dist = turtlelib::distance_btw(w4x, w4y, q.x, q.y);

      double dist = std::min({w1_dist, w2_dist, w3_dist, w4_dist});
      if (dist < min_dist)
      {
        double wx = 0.0, wy = 0.0;
        if (dist == w1_dist) {wx = w1x; wy = w1y;}
        else if (dist == w2_dist) {wx = w2x; wy = w2y;}
        else if (dist == w3_dist) {wx = w3x; wy = w3y;}
        else if (dist == w4_dist) {wx = w4x; wy = w4y;}
        if ((wx-q.x)/(max_x-q.x) > 0 && (wy-q.y)/(max_y-q.y) > 0)
        {
          ranges.at(t) = dist + ndist_lidar(get_random());
          min_dist = dist;
        }
      }
    }

    lidar_sim_data.header.stamp = get_clock()->now();
    lidar_sim_data.header.stamp.nanosec -= 2e8;
    lidar_sim_data.header.frame_id = "red/base_scan";
    lidar_sim_data.angle_min = 0.0;
    lidar_sim_data.angle_max = 2*turtlelib::PI;
    lidar_sim_data.angle_increment = angle_incr;
    lidar_sim_data.time_increment = 0.000558;
    lidar_sim_data.scan_time = 0.2;
    lidar_sim_data.range_min = lidar_range_min;
    lidar_sim_data.range_max = lidar_range_max;
    lidar_sim_data.ranges = ranges;
  }

  /// @brief Check for a collision with obstacles
  void collision_detect()
  {
    turtlelib::Vector2D rb_pos {dd.getConfig().x, dd.getConfig().y};
    double ctheta = dd.getConfig().theta;
    for (unsigned int i = 0; i < obs_vecs.size(); i++)
    {
      auto current = obs_vecs.at(i);
      double dist_or = std::sqrt(pow((current.x-rb_pos.x), 2)+pow((current.y-rb_pos.y),2));
      if (dist_or < (obstacles_r + collision_rad))
      {
        turtlelib::Vector2D obs_robot = current - rb_pos;
        turtlelib::Vector2D unit_vec = turtlelib::normalize(obs_robot);
        double dist_to_move = obstacles_r + collision_rad - dist_or;
        rb_pos.x -= dist_to_move*unit_vec.x;
        rb_pos.y -= dist_to_move*unit_vec.y;
        turtlelib::RobotConfig new_q {ctheta, rb_pos.x, rb_pos.y};
        dd.setConfig(new_q);
        break; // assuming we are only colliding with one obstacle
      }
    }
  }

  /// @brief Callback for red/wheel_cmd subscription to receive motion commands
  /// @param msg - wheel commands
  void wheelcmd_callback(const nuturtlebot_msgs::msg::WheelCommands & msg)
  {
    double wl = 0.0, wr = 0.0;

    if (msg.left_velocity != 0 && input_noise != 0)
    {
      wl = ndist_pos(get_random());
    }

    if (msg.right_velocity != 0 && input_noise != 0)
    {
      wr = ndist_pos(get_random());
    }

    vels.left = static_cast<double>(msg.left_velocity) * motor_cmd_prs + wl;
    vels.right = static_cast<double>(msg.right_velocity) * motor_cmd_prs + wr;
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
    wall.id = i;  // ids 0-3
    wall.action = visualization_msgs::msg::Marker::ADD;
    if (angle == 0) {
      wall.scale.x = arena_x;
      wall.scale.y = wall_thickness;
    } else {
      wall.scale.x = wall_thickness;
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
  rclcpp::TimerBase::SharedPtr fs_timer_;

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr robot_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sim_pub_;

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
  double obstacles_z = 0.25;  // const auto obstacles_z = 0.25;
  visualization_msgs::msg::MarkerArray obstacles_mkrs;
  visualization_msgs::msg::MarkerArray wall_mkrs;
  std::vector<turtlelib::Vector2D> obs_vecs;

  // arena walls
  double arena_x;
  double arena_y;
  double wall_thickness;

  // robot
  nuturtlebot_msgs::msg::SensorData sensor_data;
  double radius;
  double track;
  double motor_cmd_prs;
  double encoder_ticks;
  double collision_rad;
  turtlelib::DiffDrive dd;
  turtlelib::WheelPosn prev_wheel_pos;
  turtlelib::WheelPosn vels;
  double dt;

  // noise
  double input_noise;
  double slip_fraction;
  double basic_sensor_variance;
  double max_range;

  // path
  nav_msgs::msg::Path robot_path;
  geometry_msgs::msg::PoseStamped rp_pose;

  // distributions
  std::normal_distribution<> ndist_pos;
  std::uniform_real_distribution<> udist_pos;
  std::normal_distribution<> ndist_fs;  // normal dist for fake_sensor
  std::normal_distribution<> ndist_lidar;  // normal dist for simulated lidar

  // lidar parameters
  double lidar_range_min;
  double lidar_range_max;
  double angle_incr;
  unsigned int num_samples;
  double resolution;
  double noise_level;
  sensor_msgs::msg::LaserScan lidar_sim_data;
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
