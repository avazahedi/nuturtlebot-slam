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

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class NUSim : public rclcpp::Node
{
  public:
    NUSim()
    : Node("nusim"), count_{0}
    {
        // rate parameter
        declare_parameter("rate", 200);  // default to 200 Hz

        int rate =
          get_parameter("rate").get_parameter_value().get<int>();

        // initial pose
        declare_parameter("x0", 0.0);
        declare_parameter("y0", 0.0);
        declare_parameter("theta0", 0.0);

        x0 = get_parameter("x0").get_parameter_value().get<double>();
        y0 = get_parameter("y0").get_parameter_value().get<double>();
        theta0 = get_parameter("theta0").get_parameter_value().get<double>();
        // RCLCPP_INFO(get_logger(), "x0: %lf y0: %lf theta0: %lf", x0, y0, theta0);

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
        if (obstacles_x.size() != obstacles_y.size())
        {
            throw std::runtime_error(std::string("Failed: Different number of obstacle x-coordinates than obstacle y-coordinates."));
        }

        obstacles_r = get_parameter("obstacles.r").get_parameter_value().get<double>();

        double obstacles_z = 0.25;

        // obstacles publisher
        obstacles_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);

        // timestep publisher
        timestep_pub_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

        // reset service
        reset_srv_ = create_service<std_srvs::srv::Empty>(
            "~/reset", std::bind(&NUSim::reset_callback, this, 
            std::placeholders::_1, std::placeholders::_2));

        // teleport service
        teleport_srv_ = create_service<nusim::srv::Teleport>(
            "~/teleport", std::bind(&NUSim::teleport_callback, this, 
            std::placeholders::_1, std::placeholders::_2));

        // transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // populate obstacles MarkerArray
        for (unsigned int i=0; i<obstacles_x.size(); i++)
        {
            visualization_msgs::msg::Marker obs;
            obs.header.frame_id = "nusim/world";
            obs.header.stamp = get_clock()->now();
            obs.type = visualization_msgs::msg::Marker::CYLINDER;
            obs.id = i;
            obs.action = visualization_msgs::msg::Marker::ADD;
            obs.scale.x = 2*obstacles_r;
            obs.scale.y = 2*obstacles_r;
            obs.scale.z = obstacles_z;
            obs.pose.position.x = obstacles_x[i];
            obs.pose.position.y = obstacles_y[i];
            obs.pose.position.z = obstacles_z/2;
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

        // timer 
        timer_ = create_wall_timer(
        std::chrono::milliseconds(1000/rate), std::bind(&NUSim::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        auto message = std_msgs::msg::UInt64();
        message.data = count_;
        // RCLCPP_INFO(get_logger(), "Publishing timestep %u", count_);
        timestep_pub_->publish(message);
        count_++;

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
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Send the transformation
        tf_broadcaster_->sendTransform(t);

        // Publish to ~/obstacles
        obstacles_pub_->publish(obstacles_mkrs);

    }

    void reset_callback(std_srvs::srv::Empty::Request::SharedPtr,
                std_srvs::srv::Empty::Response::SharedPtr)
    {
        count_ = 0;
        x = x0;
        y = y0;
        theta = theta0;
    }

    void teleport_callback(const std::shared_ptr<nusim::srv::Teleport::Request> request,
                            std::shared_ptr<nusim::srv::Teleport::Response>)
    {
        x = request->x;
        y = request->y;
        theta = request->theta;
    }

    double x0;
    double y0;
    double theta0;
    unsigned int count_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
    rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_srv_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    double x;
    double y;
    double theta;
    std::vector<double> obstacles_x;
    std::vector<double> obstacles_y;
    double obstacles_r;
    visualization_msgs::msg::MarkerArray obstacles_mkrs;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NUSim>());
    rclcpp::shutdown();
    return 0;
}