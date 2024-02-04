#include <cstdio>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {

    // Timers
    timer_ =
      this->create_wall_timer(
        std::chrono::milliseconds(1000 / 200),
        std::bind(&TurtleControl::timer_callback, this));
  }
private:
  /// \brief Main timer callback function
  void timer_callback()
  {
    RCLCPP_INFO_STREAM(get_logger(), "The node now appears to be up!");
  }


// Declare member variables
rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}



