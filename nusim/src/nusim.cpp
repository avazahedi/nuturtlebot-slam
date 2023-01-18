#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"
#include "nusim/srv/teleport.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class NUSim : public rclcpp::Node
{
  public:
    NUSim()
    : Node("nusim"), x0{0}, y0{10}, theta0{0}, count_{0}
    {
        // rate parameter
        declare_parameter("rate", 200);  // default to 200 Hz

        int rate =
          get_parameter("rate").get_parameter_value().get<int>();

        // set initial values to pose
        pose.x = x0;
        pose.y = y0;
        pose.theta = theta0;

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
        t.transform.translation.x = pose.x;
        t.transform.translation.y = pose.y;
        t.transform.translation.z = 0.0;

        // Set transform rotation in quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, pose.theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Send the transformation
        tf_broadcaster_->sendTransform(t);

    }

    void reset_callback(std_srvs::srv::Empty::Request::SharedPtr,
                std_srvs::srv::Empty::Response::SharedPtr)
    {
        count_ = 0;
        pose.x = x0;
        pose.y = y0;
        pose.theta = theta0;
    }

    void teleport_callback(const std::shared_ptr<nusim::srv::Teleport::Request> request,
                            std::shared_ptr<nusim::srv::Teleport::Response>)
    {
        pose.x = request->x;
        pose.y = request->y;
        pose.theta = request->theta;
    }

    double x0;
    double y0;
    double theta0;
    unsigned int count_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
    rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_srv_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    turtlesim::msg::Pose pose;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NUSim>());
    rclcpp::shutdown();
    return 0;
}