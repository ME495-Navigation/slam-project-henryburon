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
#include "visualization_msgs/msg/marker_array.hpp"

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
    declare_parameter("walls.arena_x_length", 5.0);
    declare_parameter("walls.arena_y_length", 5.0);

    wall_thickness_ = 0.1;


    rate_ = get_parameter("rate").get_parameter_value().get<int>();
    x0_ = get_parameter("x0").get_parameter_value().get<double>();
    y0_ = get_parameter("y0").get_parameter_value().get<double>();
    theta0_ = get_parameter("theta0").get_parameter_value().get<double>();
    arena_x_length_ = get_parameter("walls.arena_x_length").get_parameter_value().get<double>();
    arena_y_length_ = get_parameter("walls.arena_y_length").get_parameter_value().get<double>();


    // Create a publisher for UInt64 messages on the "~/timestep" topic.
    timestep_publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    // Wall publisher
    walls_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);

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

    create_walls_();
  }

private:




  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_++;

    // Publish the UInt64 message
    timestep_publisher_->publish(message);

    // Read message content and assign it to corresponding tf variables
    geometry_msgs::msg::TransformStamped t;
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

    walls_publisher_->publish(wall_markers_array_);

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

  void create_walls_()
  {
    // The walls have a fixed height of 0.25m

    visualization_msgs::msg::Marker wall_mark_1;
    wall_mark_1.header.frame_id = "nusim/world";
    wall_mark_1.header.stamp = get_clock()->now();
    wall_mark_1.id = 1;
    wall_mark_1.type = visualization_msgs::msg::Marker::CUBE;
    wall_mark_1.action = visualization_msgs::msg::Marker::ADD;

    wall_mark_1.pose.position.x = 0.0;
    wall_mark_1.pose.position.y = arena_y_length_/2 + wall_thickness_/2;
    wall_mark_1.pose.position.z = 0.25/2;

    wall_mark_1.scale.x = arena_x_length_ + 2*wall_thickness_;
    wall_mark_1.scale.y = wall_thickness_;
    wall_mark_1.scale.z = 0.25;

    wall_mark_1.color.r = 1.0f;
    wall_mark_1.color.a = 1.0f;

    wall_markers_array_.markers.push_back(wall_mark_1);

    visualization_msgs::msg::Marker wall_mark_2;
    wall_mark_2.header.frame_id = "nusim/world";
    wall_mark_2.header.stamp = get_clock()->now();
    wall_mark_2.id = 2;
    wall_mark_2.type = visualization_msgs::msg::Marker::CUBE;
    wall_mark_2.action = visualization_msgs::msg::Marker::ADD;

    wall_mark_2.pose.position.x = 0.0;
    wall_mark_2.pose.position.y = -(arena_y_length_/2 + wall_thickness_/2);
    wall_mark_2.pose.position.z = 0.25/2;

    wall_mark_2.scale.x = arena_x_length_ + 2*wall_thickness_;
    wall_mark_2.scale.y = wall_thickness_;
    wall_mark_2.scale.z = 0.25;

    wall_mark_2.color.r = 1.0f;
    wall_mark_2.color.a = 1.0f;

    wall_markers_array_.markers.push_back(wall_mark_2);

    visualization_msgs::msg::Marker wall_mark_3;
    wall_mark_3.header.frame_id = "nusim/world";
    wall_mark_3.header.stamp = get_clock()->now();
    wall_mark_3.id = 3;
    wall_mark_3.type = visualization_msgs::msg::Marker::CUBE;
    wall_mark_3.action = visualization_msgs::msg::Marker::ADD;

    wall_mark_3.pose.position.x = arena_x_length_/2 + wall_thickness_/2;
    wall_mark_3.pose.position.y = 0.0;
    wall_mark_3.pose.position.z = 0.25/2;

    wall_mark_3.scale.x = wall_thickness_;
    wall_mark_3.scale.y = arena_y_length_ + 2*wall_thickness_;
    wall_mark_3.scale.z = 0.25;

    wall_mark_3.color.r = 1.0f;
    wall_mark_3.color.a = 1.0f;

    wall_markers_array_.markers.push_back(wall_mark_3);

    visualization_msgs::msg::Marker wall_mark_4;
    wall_mark_4.header.frame_id = "nusim/world";
    wall_mark_4.header.stamp = get_clock()->now();
    wall_mark_4.id = 4;
    wall_mark_4.type = visualization_msgs::msg::Marker::CUBE;
    wall_mark_4.action = visualization_msgs::msg::Marker::ADD;

    wall_mark_4.pose.position.x = -(arena_x_length_/2 + wall_thickness_/2);
    wall_mark_4.pose.position.y = 0.0;
    wall_mark_4.pose.position.z = 0.25/2;

    wall_mark_4.scale.x = wall_thickness_;
    wall_mark_4.scale.y = arena_y_length_ + 2*wall_thickness_;
    wall_mark_4.scale.z = 0.25;

    wall_mark_4.color.r = 1.0f;
    wall_mark_4.color.a = 1.0f;

    wall_markers_array_.markers.push_back(wall_mark_4);
  }



  // Declare member variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_service;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  visualization_msgs::msg::MarkerArray wall_markers_array_;

  int timestep_;
  int rate_;
  double x0_;
  double y0_;
  double theta0_;
  double x_;
  double y_;
  double theta_;
  double arena_x_length_;
  double arena_y_length_;
  double wall_thickness_;


};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
