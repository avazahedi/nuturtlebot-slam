#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class NUSim : public rclcpp::Node
{
  public:
    NUSim()
    : Node("nusim"), count_(0)
    {
        this->declare_parameter("rate", 200);  // default to 200 Hz

        int rate =
          this->get_parameter("rate").get_parameter_value().get<int>();

        timestep_pub_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

        reset_srv_ = create_service<std_srvs::srv::Empty>(
            "~/reset", std::bind(&NUSim::reset_callback, this, 
            std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000/rate), std::bind(&NUSim::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        auto message = std_msgs::msg::UInt64();
        message.data = count_;
        RCLCPP_INFO(this->get_logger(), "Publishing timestep %u", count_);
        timestep_pub_->publish(message);
        count_++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
    unsigned int count_;

    void reset_callback(std_srvs::srv::Empty::Request::SharedPtr req,
                std_srvs::srv::Empty::Response::SharedPtr res)
    {
        (void)req;  // get rid of unused parameter warnings
        (void)res;
        count_ = 0;
    }
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NUSim>());
    rclcpp::shutdown();
    return 0;
}