#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "turtlelib/diff_drive.hpp"


using namespace std::chrono_literals;

class Odometry : public rclcpp::Node
{
public:
    Odometry()
    : Node("odometry")
    {
        // Parameters
        declare_parameter("body_id", "");
        declare_parameter("odom_id", "odom");
        declare_parameter("wheel_left", "");
        declare_parameter("wheel_right", "");

        body_id_ = get_parameter("body_id").get_value<std::string>();
        odom_id_ = get_parameter("odom_id").get_value<std::string>();
        wheel_left_ = get_parameter("wheel_left").get_value<std::string>();
        wheel_right_ = get_parameter("wheel_right").get_value<std::string>();

        // Publishers
        odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // Subscribers
        joint_states_sub = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&Odometry::joint_states_callback, this, std::placeholders::_1));

        // Initialize variables
        old_radian_.position = {0.0, 0.0};

    // Setup functions
    check_odom_params();
  }

private:

    void joint_states_callback(const sensor_msgs::msg::JointState & msg)
    {
      // Update internal odometry state
      wheels_.phi_l = msg.position.at(0) - old_radian_.position.at(0); // Find delta wheels
      wheels_.phi_r = msg.position.at(1) - old_radian_.position.at(1);
      dd_robot_.forward_kinematic_update(wheels_);

      // Reset old_radian
      old_radian_.position = {msg.position.at(0), msg.position.at(1)};

      // Publish an odometry message on the odom topic

      

    }

    /// \brief Checks if required parameters are defined
    void check_odom_params()
    {
        if (body_id_.empty()
            || odom_id_.empty()
            || wheel_left_.empty()
            || wheel_right_.empty())
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Not all required parameters are defined in diff_params.yaml.");
            rclcpp::shutdown(); 
        }
    }


/// Declare member variables ///

// Publishers
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

// Subscribers
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;

// Variables
std::string body_id_;
std::string odom_id_;
std::string wheel_left_;
std::string wheel_right_;
turtlelib::DiffDrive dd_robot_;
turtlelib::Wheels wheels_;
sensor_msgs::msg::JointState old_radian_;
// nav_msgs::msg


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Odometry>());
    rclcpp::shutdown();
    return 0;
}


// switch to the .empty() for finding the parameters
// const auto
// type cast
// always initialize variables
// never do using namespace in header file