#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
public:
    Nusim()
    : Node("nusim"), timestep_(0)
    {
        // Create a publisher for UInt64 messages on the "~/timestep" topic.
        timestep_publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

        // Create a timer that triggers the timer_callback function every 500 milliseconds.
        timer_ = this->create_wall_timer(
            500ms, std::bind(&Nusim::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::UInt64();
        message.data = timestep_++;

        std::string message_str = std::to_string(message.data);

        // Publish the UInt64 message.
        timestep_publisher_->publish(message);

        // Print the message to the command line.
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_str.c_str());
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_; // Corrected publisher type
    int timestep_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nusim>());
    rclcpp::shutdown();
    return 0;
}
