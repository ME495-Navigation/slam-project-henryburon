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
    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);
    declare_parameter("motor_cmd_max", -1.0);
    declare_parameter("motor_cmd_per_rad_sec", -1.0);
    declare_parameter("encoder_ticks_per_rad", -1.0);
  
    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();
    motor_cmd_max_ = get_parameter("motor_cmd_max").get_parameter_value().get<double>();
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();


    // Publishers
    wheel_cmd_pub = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
    joint_states_pub = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Subscribers
    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1));
    sensor_data_sub = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(&TurtleControl::sensor_data_callback, this, std::placeholders::_1));

    // Setup Functions
    check_params();

    robot = turtlelib::DiffDrive(track_width_, wheel_radius_, {0.0, 0.0}, {0.0, 0.0, 0.0});


  }


private:

  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
  {
    
    // RCLCPP_INFO(this->get_logger(), "cmd_vel_callback started with linear.x: %f and angular.z: %f", msg.linear.x, msg.angular.z);

    // Make an instance of the DiffDrive class
    

    // Construct a Twist2D using received cmd_vel message
    turtlelib::Twist2D twist;
    twist.omega = msg.angular.z;
    twist.x = msg.linear.x;
    twist.y = 0.0; // Could set this to 0.0

    // RCLCPP_ERROR(this->get_logger(),"[cmd_vel] Angular: %f    x: %f", twist.omega, twist.x);

    // Perform inverse kinematics to get the wheel commands
    turtlelib::Wheels required_wheels = robot.inverse_kinematics(twist);

    // RCLCPP_ERROR(this->get_logger(),"[after IK] Left: %f    Right: %f", required_wheels.phi_l, required_wheels.phi_r);

    // Create a WheelCommands message
    nuturtlebot_msgs::msg::WheelCommands wheel_cmd_;

    // Load the wheel_cmd message
    wheel_cmd_.left_velocity = required_wheels.phi_l;
    wheel_cmd_.right_velocity = required_wheels.phi_r;

    nuturtlebot_msgs::msg::WheelCommands wheel_cmd_temp;
    wheel_cmd_temp.left_velocity = static_cast<int>(wheel_cmd_.left_velocity / motor_cmd_per_rad_sec_); // and multiply by rate?
    wheel_cmd_temp.right_velocity = static_cast<int>(wheel_cmd_.right_velocity / motor_cmd_per_rad_sec_);

    // RCLCPP_ERROR(this->get_logger(),"[after static cast] Left vel: %d    Right vel: %d", wheel_cmd_temp.left_velocity, wheel_cmd_temp.right_velocity);


    // Ensure motor (wheel) commands are within specified interval
    if (wheel_cmd_temp.left_velocity > motor_cmd_max_)
    {
      wheel_cmd_temp.left_velocity = motor_cmd_max_;
    }
    else if (wheel_cmd_temp.left_velocity < -motor_cmd_max_)
    {
      wheel_cmd_temp.left_velocity = -motor_cmd_max_;
    }
    if (wheel_cmd_temp.right_velocity > motor_cmd_max_)
    {
      wheel_cmd_temp.right_velocity = motor_cmd_max_;
    }
    else if (wheel_cmd_temp.right_velocity < -motor_cmd_max_)
    {
      wheel_cmd_temp.right_velocity = -motor_cmd_max_;
    }


    // RCLCPP_ERROR(this->get_logger(),"[published wheel_cmd] Left vel: %d    Right vel: %d", wheel_cmd_temp.left_velocity, wheel_cmd_temp.right_velocity);

    // Publish the wheel_cmd message
    wheel_cmd_pub->publish(wheel_cmd_temp);

  }

  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    // Create a JointState message
    sensor_msgs::msg::JointState js;

    // Create the message (header, name, position, velocity, effort)
    js.header.stamp = msg.stamp;
    js.name = {"wheel_left_joint", "wheel_right_joint"};

    // Load position and velocity
    if (flag_stamp < 0.0) // If it's the first reading...
    {
      js.position = {0.0, 0.0};
      js.velocity = {0.0, 0.0};
    }
    else
    {
      // Time since last reading
      double elapsed_time = msg.stamp.sec + msg.stamp.nanosec * 1e-9 - flag_stamp; // Convert units to seconds

      js.position = 
        {msg.left_encoder / encoder_ticks_per_rad_,
         msg.right_encoder / encoder_ticks_per_rad_};

      if (elapsed_time > 0.0)
      {
        js.velocity = {js.position.at(0) / elapsed_time,
                       js.position.at(1) / elapsed_time};
      }

      // Handle division by zero scenario
      else
      {
        js.velocity = {0.0, 0.0};
      }
    }

    // Set stamp and publish joint states
    flag_stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9; // Can also use: get_clock()->now()
    joint_states_pub->publish(js);


    
  }



  void check_params()
  {
    if (wheel_radius_ < 0.0 
        || track_width_ < 0.0
        || motor_cmd_max_ < 0.0
        || motor_cmd_per_rad_sec_ < 0.0
        || encoder_ticks_per_rad_ < 0.0)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Not all required parameters are defined in diff_params.yaml.");
      rclcpp::shutdown();
    }
  }





/// Declare member variables ///

// Publishers
rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;

// Subscribers
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub;

// Variables
int rate_;
double wheel_radius_;
double track_width_;
double motor_cmd_max_;
double motor_cmd_per_rad_sec_;
double encoder_ticks_per_rad_;
double flag_stamp = -1.0;
turtlelib::DiffDrive robot;


};










int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}