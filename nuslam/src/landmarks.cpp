/// \file
/// \brief The Slam class inherits the Node class and publishes the map to green/odom transform.
///        It also performs SLAM using the Extended Kalman Filter and is shown by the green robot.
///
/// PUBLISHES:
///     landmarks (visualization_msgs/MarkerArray): publishes the landmarks from circle-fitting
///     clusters (visualization_msgs/MarkerArray): publishes the detected clusters (disabled)
/// SUBSCRIBES:
///     lidar_sim (sensor_msgs/LaserScan): laser scan from the robot

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtlelib/circle_fit.hpp"
#include <armadillo>

using namespace std::chrono_literals;

/// \brief The Slam class inherits the Node class and publishes the map to green/odom transform.
class Landmarks : public rclcpp::Node
{
public:
  Landmarks()
  : Node("landmarks")
  {
    thresh = 0.03; // 3cm threshold

    // landmark locations publisher
    landmarks_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/landmarks", 10);

    clusters_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/clusters", 10);

    // laser scan subscriber
    laser_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar_sim", 10,
      std::bind(&Landmarks::ls_callback, this, std::placeholders::_1));

  }

private:
  double thresh;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmarks_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clusters_pub_;


  /// @brief Callback for lidar_sim subscription
  /// @param msg - lidar sim LaserScan message
  void ls_callback(const sensor_msgs::msg::LaserScan & msg)
  {
    std::vector<std::vector<turtlelib::Vector2D>> clusters;
    std::vector<turtlelib::Vector2D> current_cluster;

    for (size_t i=0; i < msg.ranges.size(); i++)
    {
        double d = msg.ranges.at(i);
        if (d != 0.0)   // found a non-zero range
        {
            double theta = turtlelib::deg2rad(i);
            double x = d*cos(theta);
            double y = d*sin(theta);

            // if first point of first cluster
            if (clusters.empty() && current_cluster.empty())
            {
                turtlelib::Vector2D vec{x,y};
                current_cluster.push_back(vec);
                continue;
            }

            double x_prev = msg.ranges.at(i-1)*cos(turtlelib::deg2rad(i-1));
            double y_prev = msg.ranges.at(i-1)*sin(turtlelib::deg2rad(i-1));
            double dist = turtlelib::distance_btw(x, y, x_prev, y_prev);
            if (dist <= thresh)
            {
                turtlelib::Vector2D vec{x,y};
                current_cluster.push_back(vec);
            }
            else
            {
                // only include the current cluster if it has > 4 points
                if (current_cluster.size() >= 5)
                {
                    clusters.push_back(current_cluster);
                }
                // reset the current cluster and add this point
                current_cluster.clear();
                turtlelib::Vector2D vec{x,y};
                current_cluster.push_back(vec);
            }

        }
    } // end of for loop

    // check for wrap-around
    // check the last point in the last cluster with the first point in the first cluster
    // if they are within the distance threshold, combine the last cluster and the first cluster

    if (clusters.size() >= 2)    // only check if we have some clusters to begin with
    {
        // last point in the last cluster
        auto last_cluster = clusters.back();
        turtlelib::Vector2D last = last_cluster.back();

        // first point in first cluster
        auto first_cluster = clusters.at(0);
        turtlelib::Vector2D first = first_cluster.at(0);

        auto wrap_dist = turtlelib::distance_btw(last.x, last.y, first.x, first.y);

        if (wrap_dist <= thresh)
        {
            RCLCPP_INFO_STREAM(get_logger(), "WRAP AROUND");

            // concatenate the last cluster and first cluster
            // append the first cluster to the end of the last one
            last_cluster.insert( last_cluster.end(), first_cluster.begin(), first_cluster.end() );
            clusters.erase( clusters.begin() ); // delete the first cluster
            clusters.erase ( std::prev(clusters.end()) );   // delete the last cluster
            clusters.push_back(last_cluster);   // add the new combined cluster
        }
    }

    // // visualize clusters
    // auto marker_stamp = get_clock()->now();
    // visualization_msgs::msg::MarkerArray cmarkers;
    // int id_val = 0;

    // for (size_t j=0; j<clusters.size(); j++) {
    //     for (size_t k=0; k<clusters.at(j).size(); k++) {
    //         // create markers here
    //         turtlelib::Vector2D clust_pt = clusters.at(j).at(k);
    //         visualization_msgs::msg::Marker mkr;
    //         mkr.header.frame_id = "green/base_scan";
    //         mkr.header.stamp = marker_stamp;
    //         mkr.type = visualization_msgs::msg::Marker::CYLINDER;
    //         mkr.id = id_val;
    //         mkr.action = visualization_msgs::msg::Marker::ADD;
    //         mkr.scale.x = 0.05;
    //         mkr.scale.y = 0.05;
    //         mkr.scale.z = 0.5;
    //         mkr.pose.position.x = clust_pt.x;
    //         mkr.pose.position.y = clust_pt.y;
    //         mkr.pose.position.z = mkr.scale.z / 2.0;
    //         mkr.pose.orientation.x = 0.0;
    //         mkr.pose.orientation.y = 0.0;
    //         mkr.pose.orientation.z = 0.0;
    //         mkr.pose.orientation.w = 1.0;
    //         mkr.color.r = 1.0;
    //         mkr.color.g = 0.0;
    //         mkr.color.b = 1.0;
    //         mkr.color.a = 1.0;
    //         cmarkers.markers.push_back(mkr);

    //         id_val++;
    //     }
    // }
    // clusters_pub_->publish(cmarkers);


    // fit circles to clusters and visualize
    auto circle_ts = get_clock()->now();
    visualization_msgs::msg::MarkerArray circle_mkrs;
    int idc = 0;
    for (size_t i=0; i < clusters.size(); i++)
    {
        turtlelib::Circle circle = turtlelib::circle_fit(clusters.at(i));
        if (circle.radius <= 0.055 && circle.radius >= 0.025) // actually a circle we care about
        {
            visualization_msgs::msg::Marker cmkr;
            cmkr.header.frame_id = "green/base_scan";
            cmkr.header.stamp = circle_ts;
            // cmkr.header.stamp.nanosec += 2e4;
            cmkr.type = visualization_msgs::msg::Marker::CYLINDER;
            cmkr.id = idc;
            cmkr.action = visualization_msgs::msg::Marker::ADD;
            cmkr.scale.x = 2.0*circle.radius;
            cmkr.scale.y = 2.0*circle.radius;
            cmkr.scale.z = 0.1;
            cmkr.pose.position.x = circle.center.x;
            cmkr.pose.position.y = circle.center.y;
            cmkr.pose.position.z = 0.0;
            cmkr.pose.orientation.x = 0.0;
            cmkr.pose.orientation.y = 0.0;
            cmkr.pose.orientation.z = 0.0;
            cmkr.pose.orientation.w = 1.0;
            cmkr.color.r = 1.0;
            cmkr.color.g = 0.0;
            cmkr.color.b = 1.0;
            cmkr.color.a = 1.0;
            circle_mkrs.markers.push_back(cmkr);
            idc++;
        }
    }

    landmarks_pub_->publish(circle_mkrs);

  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}
