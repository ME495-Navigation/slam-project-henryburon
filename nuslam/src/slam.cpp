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
///    \param initial_pose (double): The initial pose of the robot.
///
/// PUBLISHES:
///    \param green/odom (nav_msgs::msg::Odometry): Publishes the computed odometry of the robot.
///    \param green/path (nav_msgs::msg::Path): Publishes the path of the green robot.
///
/// SUBSCRIBES:
///    \param red/joint_states (sensor_msgs::msg::JointState): Subscribes to the joint state messages to compute odometry.
///
/// SERVICES:
///    \initial_pose (nuturtle_control::srv::InitialPose): Service to set the initial pose of the robot.

#include <chrono>
#include <string>
#include <armadillo>
#include <sstream>
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
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

/// \brief Computes and publishes odometry from the joint states.
class Slam : public rclcpp::Node
{
public:
  Slam()
  : Node("slam")
  {
    // Parameters
    declare_parameter("body_id", "");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", "green/wheel_left_link");
    declare_parameter("wheel_right", "green/wheel_right_link");
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.160);
    declare_parameter("obstacles.x", std::vector<double>{});
    declare_parameter("obstacles.y", std::vector<double>{});
    declare_parameter("obstacles.r", 0.038);
    declare_parameter("collision_radius", 0.11);


    body_id_ = get_parameter("body_id").get_value<std::string>();
    odom_id_ = get_parameter("odom_id").get_value<std::string>();
    wheel_left_ = get_parameter("wheel_left").get_value<std::string>();
    wheel_right_ = get_parameter("wheel_right").get_value<std::string>();
    wheel_radius_ = get_parameter("wheel_radius").get_value<double>();
    track_width_ = get_parameter("track_width").get_value<double>();
    obstacles_x_ = get_parameter("obstacles.x").get_parameter_value().get<std::vector<double>>();
    obstacles_y_ = get_parameter("obstacles.y").get_parameter_value().get<std::vector<double>>();
    obstacles_r_ = get_parameter("obstacles.r").get_parameter_value().get<double>();
    collision_radius_ = get_parameter("collision_radius").get_value<double>();

    // Publishers
    green_odom_pub = create_publisher<nav_msgs::msg::Odometry>("green/odom", 10);
    green_path_pub = create_publisher<nav_msgs::msg::Path>("green/path", 10);
    green_obstacles_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
      "green/obstacles",
      10);

    // Subscribers
    joint_states_sub = create_subscription<sensor_msgs::msg::JointState>(
      "blue/joint_states", 10,
      std::bind(&Slam::joint_states_callback, this, std::placeholders::_1));
    fake_sensor_sub = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/fake_sensor", 10,
      std::bind(&Slam::fake_sensor_callback, this, std::placeholders::_1));

    // Broadcasters
    odom_body_broadcaster =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    green_odom_broadcast = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Services
    initial_pose_init = create_service<nuturtle_control::srv::InitialPose>(
      "/initial_pose",
      std::bind(
        &Slam::initial_pose_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    // Initialize variables
    old_radian_.position = {0.0, 0.0};

    // Setup functions
    check_odom_params();
    // create_green_obstacles(); // SLAM estimate for obstacles

    // Initialize the SLAM variables
    n_obstacles = obstacles_x_.size();
    // state_estimate = arma::vec(3 + 2 * n_obstacles, arma::fill::zeros); // [theta, x, y, x1, y1, x2, y2, ...]
    state_estimate = arma::vec(3 + 2 * n_obstacles, arma::fill::zeros); // [theta, x, y, x1, y1, x2, y2, ...]
    // for (size_t i = 3; i < state_estimate.size(); ++i) {
    //     state_estimate(i) = 99;
    // }
    old_state_estimate = arma::vec(3 + 2 * n_obstacles, arma::fill::zeros);
    delta_state = arma::vec(3 + 2 * n_obstacles, arma::fill::zeros);
    // intialize covariance matrix with large values along diagonals to indicate
    // uncertainty in the initial state (high values = high uncertainty)
    covariance = 99999 * arma::mat(3 + 2 * n_obstacles, 3 + 2 * n_obstacles, arma::fill::eye); // a 9x9 matrix, when 3 obstacles.
    init_obs = arma::vec(n_obstacles, arma::fill::zeros);
    R = 1e-3 * arma::mat(2 * n_obstacles, 2 * n_obstacles, arma::fill::eye); // measurement noise covariance matrix
    // should probably put Q up here, too

    obstacle_sensed = false;

  }

private:
  /// @brief Callback for the fake sensor messages which are used to update the SLAM state.
  /// @param sensor_data The incoming fake sensor message.
  void fake_sensor_callback(const visualization_msgs::msg::MarkerArray::SharedPtr sensor_data)
  {

    // update the state estimate

    turtlelib::Vector2D trans;
    trans.x = robot_.get_robot_config().x;
    trans.y = robot_.get_robot_config().y;
    T_ob.set_translation(trans);
    T_ob.set_rotation(robot_.get_robot_config().theta);

    // get map to body transform
    T_mb = T_mo * T_ob;
    state_estimate.at(0) = turtlelib::normalize_angle(T_mb.rotation());
    state_estimate.at(1) = T_mb.translation().x;
    state_estimate.at(2) = T_mb.translation().y;

    // get change in estimate since last iteration
    delta_state = state_estimate - old_state_estimate;

    // 3. propagate the uncertainty using the linearized state transition model

    // get the Jacobian of the state transition model (At)
    arma::mat A_t = arma::mat(3 + 2 * n_obstacles, 3 + 2 * n_obstacles, arma::fill::eye);

    // fill in At with the odom update
    A_t(1, 0) = -delta_state.at(2);
    A_t(2, 0) = delta_state.at(1);

    // calc the process noise (Q_bar)
    arma::mat Q = 1e-3 * arma::mat(3, 3, arma::fill::eye); // small value indicating small uncertainty
    arma::mat Q_bar = arma::mat(3 + 2 * n_obstacles, 3 + 2 * n_obstacles, arma::fill::zeros);

    // fill in Q_bar with Q
    Q_bar.submat(0, 0, 2, 2) = Q;

    // propagate the uncertainty
    covariance = A_t * covariance * A_t.t() + Q_bar;

    // 4. Update

    // ########## for each sensor measurement... ##########
    for (int i = 0; i < static_cast<int>(sensor_data->markers.size()); i++) { // i = 0, 1, 2
      if (sensor_data->markers.at(i).action == 0) { // 0 = ADD => sensed obstacle. Only continue if robot senses the obstacle
        obstacle_sensed = true;

        // Compute the theoretical measurement, given the current state estimate
        arma::vec z_j = arma::vec(2, arma::fill::zeros);

        // get the sensed obstacle position and id
        double mx = sensor_data->markers.at(i).pose.position.x; // absolute position of the obstacle
        double my = sensor_data->markers.at(i).pose.position.y;
        // int id = sensor_data->markers.at(i).id;

        // find r_j and phi_j (relative distance and bearing to obstacle i)
        const auto r_j =
          std::sqrt(
          std::pow(
            (mx - state_estimate.at(1)),
            2) + std::pow((my - state_estimate.at(2)), 2));                                                               // relative measurements from (14)--Should they be absolute?
        const auto phi_j =
          turtlelib::normalize_angle(
          std::atan2(
            (my - state_estimate.at(2)),
            (mx - state_estimate.at(1))) - state_estimate.at(0));

        // the measurement model relates the system states to the measurements. z_j = h_j
        z_j.at(0) = r_j;
        z_j.at(1) = phi_j;

        // check if this obstacle has been initialized yet...
        if (init_obs.at(i) == 0) {
          // if not yet initialized, initialize it
          init_obs.at(i) = 1;
          // add to state estimate vector
          // x
          state_estimate.at(3 + 2 * i) = state_estimate.at(1) + r_j * std::cos(
            phi_j + state_estimate.at(
              0));                                                                                            // formula (23)
          // y
          state_estimate.at(3 + 2 * i + 1) = state_estimate.at(2) + r_j * std::sin(
            phi_j + state_estimate.at(
              0));                                                                                                // formula (24)

          RCLCPP_INFO(this->get_logger(), "Initialized obstacle: %d", i);
        }

        // get the Jacobian of the measurement model: H_t

        // relative x and y distances
        const auto d_x = mx - state_estimate.at(1);
        const auto d_y = my - state_estimate.at(2);

        // estimated squared distance betwee the robot and landmark j at time t
        const auto d = std::pow(d_x, 2) + std::pow(d_y, 2);

        arma::mat block1 = arma::zeros(2, 3);
        block1(0, 1) = -d_x / std::sqrt(d);
        block1(0, 2) = -d_y / std::sqrt(d);
        block1(1, 0) = -1;
        block1(1, 1) = d_y / d;
        block1(1, 2) = -d_x / d;

        arma::mat block2;

        if (i != 0) {
          block2 = arma::zeros(2, 2 * i);
        }

        arma::mat block3 = arma::zeros(2, 2);
        block3(0, 0) = d_x / std::sqrt(d);
        block3(0, 1) = d_y / std::sqrt(d);
        block3(1, 0) = -d_y / d;
        block3(1, 1) = d_x / d;

        arma::mat block4;

        if (i != n_obstacles - 1) {
          block4 = arma::zeros(2, 2 * (n_obstacles - i - 1));
        }

        // join the 4 blocks to create H_j: the derviative (Jacobian) of the measurement model with respect to the state
        arma::mat H_j = arma::join_horiz(block1, block2, block3, block4);

        // Compute the Kalman gain from the linearized measurement model

        arma::mat Ri = R.submat(2 * i, 2 * i, 2 * i + 1, 2 * i + 1);
        arma::mat K_i = covariance * H_j.t() * arma::inv(H_j * covariance * H_j.t() + Ri);

        // Compute posterior state update (need z_j and z_j_hat)
        // hat reflects the change in the state
        const auto r_j_hat = std::sqrt(d_x * d_x + d_y * d_y);
        const auto phi_j_hat = turtlelib::normalize_angle(
          std::atan2(d_y, d_x) - state_estimate.at(
            0));

        arma::vec z_j_hat = arma::vec(2, arma::fill::zeros);
        z_j_hat.at(0) = r_j_hat;
        z_j_hat.at(1) = phi_j_hat;

        // update the state estimate
        state_estimate = state_estimate + K_i * (z_j - z_j_hat);
        state_estimate.at(0) = turtlelib::normalize_angle(state_estimate.at(0));

        // Compute the posterior covariance update
        covariance =
          (arma::mat(
            3 + 2 * n_obstacles, 3 + 2 * n_obstacles,
            arma::fill::eye) - K_i * H_j) * covariance;
      }

    }
    if (obstacle_sensed == true) {
      detect_collisions();
    }

    turtlelib::Vector2D trans2;
    trans2.x = robot_.get_robot_config().x;
    trans2.y = robot_.get_robot_config().y;
    T_ob.set_translation(trans);
    T_ob.set_rotation(robot_.get_robot_config().theta);

    T_mb =
      turtlelib::Transform2D{turtlelib::Vector2D{state_estimate.at(1), state_estimate.at(2)},
      state_estimate.at(0)};
    T_mo = T_mb * T_ob.inv();

    create_green_obstacles(); // SLAM estimate for obstacles
    broadcast_green_odom_body(); // SLAM estimate for robot

    // update the old state estimate as final step
    old_state_estimate = state_estimate;
  }

  /// \brief Detects collision between the robot and the obstacles
  void detect_collisions()
  {
    x_detect = state_estimate.at(1) + offset_x;
    y_detect = state_estimate.at(2) + offset_y;

    theta_detect = state_estimate.at(0);

    // check if the robot is colliding with any of the (actual) obstacles
    for (int i = 0; i < n_obstacles; ++i) {
      double distance = measure_distance(
        x_detect, y_detect, state_estimate.at(
          3 + 2 * i), state_estimate.at(4 + 2 * i));

      if (obstacles_markers_array_.markers.at(i).action == visualization_msgs::msg::Marker::ADD) {

        if (distance < collision_radius_) {

          // calculate the distance to move
          double move_distance = (collision_radius_ + obstacles_r_) - distance;       // equal to the intersection of the circles

          // get the direction to move
          turtlelib::Vector2D dir_vec =
            turtlelib::Vector2D{x_detect, y_detect} - turtlelib::Vector2D{state_estimate.at(
              3 + 2 * i), state_estimate.at(4 + 2 * i)};                                                                                                                 // vector from obstacle to robot

          // normalize
          turtlelib::Vector2D dir_vec_norm = turtlelib::normalize_vector(dir_vec);

          // move but maintain the robot's orientation
          turtlelib::Vector2D new_pos =
            turtlelib::Vector2D{x_detect, y_detect} + dir_vec_norm * move_distance;
          offset_x += new_pos.x - x_detect;
          offset_y += new_pos.y - y_detect;
          offset_theta = 0.0;
        }
      }
    }
  }

  /// \brief Measures the distance between two points
  double measure_distance(double x1, double y1, double x2, double y2)
  {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
  }


  /// \brief Broadcasts the transform from the odometry frame to the robot body frame
  void broadcast_green_odom_body()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = get_clock()->now();
    t.header.frame_id = odom_id_;
    t.child_frame_id = body_id_;

    t.transform.translation.x = state_estimate.at(1) + offset_x;
    t.transform.translation.y = state_estimate.at(2) + offset_y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, state_estimate.at(0) + offset_theta);
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
    green_path_pub->publish(path_msg_);

  }

  /// \brief Creates the green (SLAM) obstacles in the simulation
  void create_green_obstacles()
  {
    obstacles_markers_array_.markers.clear();
    const auto num_markers = obstacles_x_.size();

    for (long unsigned int i = 0; i < num_markers; i++) {
      visualization_msgs::msg::Marker obs;
      obs.header.frame_id = "map";
      obs.header.stamp = get_clock()->now();
      obs.id = i;
      obs.type = visualization_msgs::msg::Marker::CYLINDER;

      if (init_obs.at(i) == 0) {
        obs.action = visualization_msgs::msg::Marker::DELETE;
      } else {
        obs.action = visualization_msgs::msg::Marker::ADD;
      }

      obs.pose.position.x = state_estimate.at(3 + 2 * i);
      obs.pose.position.y = state_estimate.at(3 + 2 * i + 1);
      obs.pose.position.z = 0.125;

      obs.scale.x = 2.0 * obstacles_r_;
      obs.scale.y = 2.0 * obstacles_r_;
      obs.scale.z = 0.25;

      obs.color.g = 1.0;
      obs.color.a = 1.0;
      obstacles_markers_array_.markers.push_back(obs);

      green_obstacles_pub->publish(obstacles_markers_array_);

    }
  }

  /// \brief Callback for the joint state messages.
  /// \param msg The incoming joint state message.
  void joint_states_callback(const sensor_msgs::msg::JointState & msg)
  {
    // Update internal odometry state
    wheels_.phi_l = msg.position.at(0) - old_radian_.position.at(0);     // Find delta wheels
    wheels_.phi_r = msg.position.at(1) - old_radian_.position.at(1);

    // sets the new config
    robot_.forward_kinematic_update(wheels_);

    // Reset old_radian
    old_radian_.position = {msg.position.at(0), msg.position.at(1)};

    // Publish the odometry message
    pub_odom();

    // Broadcast the transformation from the odom frame to the robot frame
    broadcast_map_green_odom();
  }

  void broadcast_map_green_odom()
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = get_clock()->now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "green/odom";
    tf.transform.translation.x = T_mo.translation().x + 0.02;
    tf.transform.translation.y = T_mo.translation().y;
    tf.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, T_mo.rotation());
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    green_odom_broadcast->sendTransform(tf);
  }

  void pub_odom()
  {
    // Publish an odometry message on the odom topic
    odom_msg_.header.frame_id = odom_id_;

    odom_msg_.child_frame_id = body_id_;

    odom_msg_.pose.pose.position.x = robot_.get_robot_config().x + offset_x;
    odom_msg_.pose.pose.position.y = robot_.get_robot_config().y + offset_y;

    quat_.setRPY(0.0, 0.0, robot_.get_robot_config().theta);

    odom_msg_.pose.pose.orientation.x = quat_.x();
    odom_msg_.pose.pose.orientation.y = quat_.y();
    odom_msg_.pose.pose.orientation.z = quat_.z();
    odom_msg_.pose.pose.orientation.w = quat_.w();

    turtlelib::Twist2D Vb;
    Vb.omega = ((wheel_radius_ / (track_width_)) * (wheels_.phi_r - wheels_.phi_l));
    Vb.x = (wheel_radius_ / 2.0) * (wheels_.phi_l + wheels_.phi_r);

    odom_msg_.twist.twist.linear.x = Vb.x;
    odom_msg_.twist.twist.angular.z = Vb.omega;

    green_odom_pub->publish(odom_msg_);
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

  void log_arma_vector(arma::vec v)
  {
    for (arma::uword i = 0; i < v.n_elem; ++i) {
      RCLCPP_INFO(this->get_logger(), "%f", v.at(i));
    }
  }

  void log_arma_matrix(arma::mat m)
  {
    for (arma::uword i = 0; i < m.n_rows; ++i) {
      std::stringstream ss;
      ss << m.row(i);
      RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }
  }

// Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr green_odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr green_path_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr green_obstacles_pub;

// Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub;

// Broadcasters
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_body_broadcaster;
  std::unique_ptr<tf2_ros::TransformBroadcaster> green_odom_broadcast;

// Services
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_init;

// Variables
  std::string body_id_;
  std::string odom_id_;
  std::string wheel_left_;
  std::string wheel_right_;
  turtlelib::DiffDrive robot_; // starts at (0,0)
  turtlelib::Wheels wheels_;
  sensor_msgs::msg::JointState old_radian_;
  nav_msgs::msg::Odometry odom_msg_;
  tf2::Quaternion quat_;
  double wheel_radius_;
  double track_width_;
  nav_msgs::msg::Path path_msg_;
  std::vector<double> obstacles_x_;
  std::vector<double> obstacles_y_;
  double obstacles_r_;
  visualization_msgs::msg::MarkerArray obstacles_markers_array_;
  double collision_radius_;
  double x_detect = 0.0;
  double y_detect = 0.0;
  double theta_detect = 0.0;
  bool obstacle_sensed;

  // SLAM variables
  int n_obstacles;
  arma::mat covariance, R;
  arma::vec state_estimate, old_state_estimate, delta_state, init_obs;
  turtlelib::Transform2D T_ob, T_mo, T_mb;
  double offset_x = 0.0, offset_y = 0.0, offset_theta = 0.0;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}
