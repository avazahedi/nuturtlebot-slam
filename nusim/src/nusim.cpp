#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        this->declare_parameter("rate", 200);  // default to 200 Hz

        int rate =
          this->get_parameter("rate").get_parameter_value().get<int>();

        timestep_pub_ = this->create_publisher<std_msgs::msg::UInt64>("~timestep", 10);

        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000/rate), std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        auto message = std_msgs::msg::UInt64();
        message.data = count_++;
        RCLCPP_INFO(this->get_logger(), "Publishing timestep %u", count_);
        timestep_pub_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
    unsigned int count_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}