#include <cstdio>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"


using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {
    // Parameters
    declare_parameter("wheel_radius", -5.0);
    declare_parameter("track_width", -5.0);
    declare_parameter("motor_cmd_max", -5.0);
    declare_parameter("motor_cmd_per_rad_sec", -5.0);

  
    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();
    motor_cmd_max_ = get_parameter("motor_cmd_max").get_parameter_value().get<double>();
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();



    // Publishers
    wheel_cmd_pub = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
    // joint_states_pub = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Subscribers
    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1));
    // sensor_data_sub = create_subscription<nuturtlebot_msgs::msg::SensorData>(
    //   "sensor_data", 10, std::bind(&TurtleControl::sensor_data_callback, this, std::placeholders::_1));

    // Setup Functions
    check_params();

  }


private:

  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
  {
    
    RCLCPP_INFO(this->get_logger(), "cmd_vel_callback started with linear.x: %f and angular.z: %f", msg.linear.x, msg.angular.z);

    // Make an instance of the DiffDrive class
    turtlelib::DiffDrive robot(track_width_, wheel_radius_, {0.0, 0.0}, {0.0, 0.0, 0.0});

    // Construct a Twist2D using received cmd_vel message
    turtlelib::Twist2D twist;
    twist.omega = msg.angular.z;
    twist.x = msg.linear.x;
    twist.y = 0.0; // Could set this to 0.0

    // Perform inverse kinematics to get the wheel commands
    turtlelib::Wheels required_wheels = robot.inverse_kinematics(twist);

    // Create a WheelCommands message
    nuturtlebot_msgs::msg::WheelCommands wheel_cmd_;

    // Load the wheel_cmd message
    wheel_cmd_.left_velocity = required_wheels.phi_l;
    wheel_cmd_.right_velocity = required_wheels.phi_r;

    // Publish the wheel_cmd message
    wheel_cmd_pub->publish(wheel_cmd_);

  }

  // void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData & msg) const
  // {

  // }



  void check_params()
  {
    if (wheel_radius_ < 0 || track_width_ < 0)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Not all required parameters are defined in diff_params.yaml.");
      rclcpp::shutdown();
    }
  }




/// Declare member variables ///

// Publishers
rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub;
// rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;

// Subscribers
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
// rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub;

// Variables
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