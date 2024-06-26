/// \file
/// \brief Odometry node for computing and publishing odometry from joint states.
///
/// PARAMETERS:
///    \param body_id (string): The TF frame ID of the robot's body.
///    \param odom_id (string): The TF frame ID of the odometry reference frame.
///    \param wheel_left (string): The name of the left wheel joint.
///    \param wheel_right (string): The name of the right wheel joint.
///    \param wheel_radius (double): The radius of the wheels.
///    \param track_width (double): The distance between the centers of the two wheels.
///
/// PUBLISHES:
///    \param odom (nav_msgs::msg::Odometry): Publishes the computed odometry of the robot.
///
/// SUBSCRIBES:
///    \param red/joint_states (sensor_msgs::msg::JointState): Subscribes to the joint state messages to compute odometry.
///
/// SERVICES:
///    \initial_pose (nuturtle_control::srv::InitialPose): Service to set the initial pose of the robot.

#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "turtlelib/diff_drive.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

/// \brief Computes and publishes odometry from the joint states.
class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {
    // Parameters
    declare_parameter("body_id", "");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", "wheel_left_joint");
    declare_parameter("wheel_right", "wheel_right_joint");
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.160);

    body_id_ = get_parameter("body_id").get_value<std::string>();
    odom_id_ = get_parameter("odom_id").get_value<std::string>();
    wheel_left_ = get_parameter("wheel_left").get_value<std::string>();
    wheel_right_ = get_parameter("wheel_right").get_value<std::string>();
    wheel_radius_ = get_parameter("wheel_radius").get_value<double>();
    track_width_ = get_parameter("track_width").get_value<double>();

    // Publishers
    odom_pub = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    path_pub = create_publisher<nav_msgs::msg::Path>("blue/path", 10);

    // Subscribers
    joint_states_sub = create_subscription<sensor_msgs::msg::JointState>(
      "blue/joint_states", 10,
      std::bind(&Odometry::joint_states_callback, this, std::placeholders::_1));

    // Broadcasters
    odom_body_broadcaster =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Services
    initial_pose_init = create_service<nuturtle_control::srv::InitialPose>(
      "/initial_pose",
      std::bind(
        &Odometry::initial_pose_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    // Initialize variables
    old_radian_.position = {0.0, 0.0};

    // Setup functions
    check_odom_params();
  }

private:
  /// \brief Callback function for the initial_pose service.
  void initial_pose_callback(
    std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
    std::shared_ptr<nuturtle_control::srv::InitialPose::Response>)
  {
    // Set configuration to that which is specified in request
    robot_ = turtlelib::DiffDrive{track_width_,
      wheel_radius_,
      {0.0, 0.0},
      {request->theta, request->x, request->y}};
  }

  /// \brief Callback for the joint state messages.
  /// \param msg The incoming joint state message.
  void joint_states_callback(const sensor_msgs::msg::JointState & msg)
  {
    // Update internal odometry state
    wheels_.phi_l = msg.position.at(0) - old_radian_.position.at(0);     // Find delta wheels
    wheels_.phi_r = msg.position.at(1) - old_radian_.position.at(1);

    robot_.forward_kinematic_update(wheels_);

    // Reset old_radian
    old_radian_.position = {msg.position.at(0), msg.position.at(1)};

    // Publish an odometry message on the odom topic
    odom_msg_.header.stamp = msg.header.stamp;
    odom_msg_.header.frame_id = odom_id_;

    odom_msg_.child_frame_id = body_id_;

    odom_msg_.pose.pose.position.x = robot_.get_robot_config().x;
    odom_msg_.pose.pose.position.y = robot_.get_robot_config().y;

    quat_.setRPY(0.0, 0.0, robot_.get_robot_config().theta);

    odom_msg_.pose.pose.orientation.x = quat_.x();
    odom_msg_.pose.pose.orientation.y = quat_.y();
    odom_msg_.pose.pose.orientation.z = quat_.z();
    odom_msg_.pose.pose.orientation.w = quat_.w();

    // Load twist.linear.x and twist.angular.z
    // Convert delta wheels to a twist

    turtlelib::Twist2D Vb;
    Vb.omega = ((wheel_radius_ / (track_width_)) * (wheels_.phi_r - wheels_.phi_l));
    Vb.x = (wheel_radius_ / 2.0) * (wheels_.phi_l + wheels_.phi_r);

    odom_msg_.twist.twist.linear.x = Vb.x;
    odom_msg_.twist.twist.angular.z = Vb.omega;

    odom_pub->publish(odom_msg_);

    // Broadcast the transformation from the odometry frame to the robot body frame
    broadcast_odom_body();
  }

  /// \brief Broadcasts the transform from the odometry frame to the robot body frame.
  void broadcast_odom_body()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = get_clock()->now();
    t.header.frame_id = odom_id_;
    t.child_frame_id = body_id_;

    t.transform.translation.x = robot_.get_robot_config().x;
    t.transform.translation.y = robot_.get_robot_config().y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, robot_.get_robot_config().theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    odom_body_broadcaster->sendTransform(t);

    // Publish path
    path_msg_.header.stamp = get_clock()->now();
    path_msg_.header.frame_id = "nusim/world";

    geometry_msgs::msg::PoseStamped pose_stamp_;
    pose_stamp_.pose.position.x = t.transform.translation.x;
    pose_stamp_.pose.position.y = t.transform.translation.y;
    pose_stamp_.pose.position.z = 0.0;
    pose_stamp_.pose.orientation.x = q.x();
    pose_stamp_.pose.orientation.y = q.y();
    pose_stamp_.pose.orientation.z = q.z();
    pose_stamp_.pose.orientation.w = q.w();
    path_msg_.poses.push_back(pose_stamp_);

    // Publish path
    path_pub->publish(path_msg_);

  }

  /// \brief Checks if required parameters are defined
  void check_odom_params()
  {
    if (body_id_.empty() ||
      odom_id_.empty() ||
      wheel_left_.empty() ||
      wheel_right_.empty())
    {
      RCLCPP_ERROR_STREAM(
        this->get_logger(), "Not all required parameters are defined in diff_params.yaml.");
      rclcpp::shutdown();
    }
  }

// Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

// Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;

// Broadcasters
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_body_broadcaster;

// Services
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_init;

// Variables
  std::string body_id_;
  std::string odom_id_;
  std::string wheel_left_;
  std::string wheel_right_;
  turtlelib::DiffDrive robot_;
  turtlelib::Wheels wheels_;
  sensor_msgs::msg::JointState old_radian_;
  nav_msgs::msg::Odometry odom_msg_;
  tf2::Quaternion quat_;
  double wheel_radius_;
  double track_width_;
  nav_msgs::msg::Path path_msg_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
