#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
public:
  Nusim() : Node("nusim"), timestep_(0)
  {
    // Declare rate parameter with a default value of 200
    declare_parameter("rate", 200);
    rate_ = get_parameter("rate").get_parameter_value().get<int>();

    // Create a publisher for UInt64 messages on the "~/timestep" topic.
    timestep_publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    // Create a timer that triggers the timer_callback function with a frequency of rate Hz
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / rate_), std::bind(&Nusim::timer_callback, this));

    // Create reset service
    reset_service = this->create_service<std_srvs::srv::Empty>(
        "~/reset", std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_++;

    // Publish the UInt64 message
    timestep_publisher_->publish(message);

    // Also print the message to the command line
    std::string message_str = std::to_string(message.data);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_str.c_str());
  }

  void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                      std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    timestep_ = 0;
  }


  // Declare member variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service;

  int timestep_;
  int rate_;


};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
