#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"

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



    // Setup functions
    check_odom_params();
  }

private:

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

// Variables
std::string body_id_;
std::string odom_id_;
std::string wheel_left_;
std::string wheel_right_;


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Odometry>());
    rclcpp::shutdown();
    return 0;
}