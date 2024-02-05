#include <cstdio>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {
    // Parameters
    declare_parameter("rate", 200);
    declare_parameter("wheel_radius", -5.0);
    declare_parameter("track_width", -5.0);
    declare_parameter("motor_cmd_max", -5.0);
    declare_parameter("motor_cmd_per_rad_sec", -5.0);

  
    rate_ = get_parameter("rate").get_parameter_value().get<int>();
    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();
    motor_cmd_max_ = get_parameter("motor_cmd_max").get_parameter_value().get<double>();
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();



    // Publishers
    wheel_cmd_pub = this->create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
    joint_states_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Subscribers
    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1));
    sensor_data_sub = this->create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(&TurtleControl::sensor_data_callback, this, std::placeholders::_1));

    // Timers
    timer_1 =
      this->create_wall_timer(
        std::chrono::milliseconds(1000 / rate_),
        std::bind(&TurtleControl::timer_callback, this));

    // Setup Functions
    check_params();
  }


private:
  /// \brief Main timer callback function
  void timer_callback()
  {
    // RCLCPP_INFO_STREAM(get_logger(), "The node now appears to be up!");
    // RCLCPP_INFO(get_logger(), "Rate parameter: %d", rate_);



  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg) const
  {
    // RCLCPP_INFO(this->get_logger(), "I heard: linear.x=%f, angular.z=%f",
    //             msg.linear.x, msg.angular.z);
  }

  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData & msg) const
  {

  }



  void check_params()
  {
    if (wheel_radius_ < 0 || track_width_ < 0)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Not all required parameters are defined in diff_params.yaml.");
      rclcpp::shutdown();
    }
  }

  // Need a function that takes in the Twists, and updates the robot config
  void twist_to_config()
  {

  }


/// Declare member variables ///

// Publishers
rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;

// Subscribers
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub;

// Timers
rclcpp::TimerBase::SharedPtr timer_1;

// Values
int rate_;
double wheel_radius_;
double track_width_;
double motor_cmd_max_;
double motor_cmd_per_rad_sec_;

};







int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}