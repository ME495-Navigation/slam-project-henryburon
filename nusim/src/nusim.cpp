#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nusim/srv/teleport.hpp"

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
public:
  Nusim() : Node("nusim"), timestep_(0)
  {
    // Declare parameters. There may be another way to do this...
    declare_parameter("rate", 200);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);


    rate_ = get_parameter("rate").get_parameter_value().get<int>();
    x0_ = get_parameter("x0").get_parameter_value().get<double>();
    y0_ = get_parameter("y0").get_parameter_value().get<double>();
    theta0_ = get_parameter("theta0").get_parameter_value().get<double>();

    // Create a publisher for UInt64 messages on the "~/timestep" topic.
    timestep_publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    // Create a timer that triggers the timer_callback function with a frequency of rate Hz
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / rate_), std::bind(&Nusim::timer_callback, this));

    // Create reset service
    reset_service = this->create_service<std_srvs::srv::Empty>(
        "~/reset", std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Create teleport service
    teleport_service = this->create_service<nusim::srv::Teleport>(
      "~/teleport", std::bind(&Nusim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize the the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    x_ = x0_;
    y_ = y0_;
    theta_ = theta0_;
  }

private:

  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_++;

    // Publish the UInt64 message
    timestep_publisher_->publish(message);

    // Also print the message to the command line
    // std::string message_str = std::to_string(message.data);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_str.c_str());

    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";

    // Turtle only exists in 2D, so we set z coordinate to 0
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;

    // Likewise, turtle can only rotate around one axis -- z
    tf2::Quaternion q;
    q.setRPY(0,0,theta_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transform
    tf_broadcaster_->sendTransform(t);

  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    timestep_ = 0;
    x_ = x0_;
    y_ = y0_;
    theta_ = theta0_;
  }

  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response>)
    {
      x_ = request->x;
      y_ = request->y;
      theta_ = request->theta;
    }



  // Declare member variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_service;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  int timestep_;
  int rate_;
  double x0_;
  double y0_;
  double theta0_;
  double x_;
  double y_;
  double theta_;


};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
