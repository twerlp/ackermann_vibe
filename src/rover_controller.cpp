#include "ackermann_rover/rover_controller.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

namespace ackermann_rover
{

RoverController::RoverController(const rclcpp::NodeOptions & options)
: LifecycleNode("rover_controller", "", options)
{
  RCLCPP_INFO(get_logger(), "RoverController created");
}

CallbackReturn RoverController::on_configure(const rclcpp_lifecycle::State &)
{
  declare_parameter<double>("wheelbase");
  declare_parameter<double>("track_width");
  declare_parameter<double>("wheel_radius");
  declare_parameter<double>("max_steering_angle");
  declare_parameter<double>("max_motor_speed");
  declare_parameter<double>("max_forward_speed");
  declare_parameter<double>("max_reverse_speed");

  wheelbase_ = get_parameter("wheelbase").as_double();
  track_width_ = get_parameter("track_width").as_double();
  wheel_radius_ = get_parameter("wheel_radius").as_double();
  max_steering_angle_ = get_parameter("max_steering_angle").as_double();
  max_motor_speed_ = get_parameter("max_motor_speed").as_double();
  max_forward_speed_ = get_parameter("max_forward_speed").as_double();
  max_reverse_speed_ = get_parameter("max_reverse_speed").as_double();

  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_theta_ = 0.0;

  rl_wheel_pos_ = 0.0;
  rr_wheel_pos_ = 0.0;
  fl_wheel_pos_ = 0.0;
  fr_wheel_pos_ = 0.0;

  curr_linear_vel_ = 0.0;
  curr_steering_angle_ = 0.0;

  motor_pub_ = create_publisher<std_msgs::msg::Float32>("cmd_motor", rclcpp::QoS(10).reliable());
  steering_pub_ = create_publisher<std_msgs::msg::Float32>("cmd_steering", rclcpp::QoS(10).reliable());
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10).reliable());
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(10).reliable());

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10).reliable(),
    std::bind(&RoverController::cmdVelCallback, this, std::placeholders::_1));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(get_logger(), "RoverController configured");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoverController::on_activate(const rclcpp_lifecycle::State &)
{
  motor_pub_->on_activate();
  steering_pub_->on_activate();
  odom_pub_->on_activate();
  joint_state_pub_->on_activate();

  last_time_ = now();
  last_joint_state_time_ = last_time_;
  odom_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&RoverController::publishOdometry, this));
  joint_state_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&RoverController::publishJointStates, this));

  publishJointStates();

  RCLCPP_INFO(get_logger(), "RoverController activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoverController::on_deactivate(const rclcpp_lifecycle::State &)
{
  odom_timer_->cancel();
  joint_state_timer_->cancel();
  motor_pub_->on_deactivate();
  steering_pub_->on_deactivate();
  odom_pub_->on_deactivate();
  joint_state_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "RoverController deactivated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoverController::on_cleanup(const rclcpp_lifecycle::State &)
{
  motor_pub_.reset();
  steering_pub_.reset();
  odom_pub_.reset();
  cmd_vel_sub_.reset();
  tf_broadcaster_.reset();

  RCLCPP_INFO(get_logger(), "RoverController cleaned up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoverController::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (odom_timer_) odom_timer_->cancel();
  if (joint_state_timer_) joint_state_timer_->cancel();
  RCLCPP_INFO(get_logger(), "RoverController shutdown");
  return CallbackReturn::SUCCESS;
}

void RoverController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double linear = msg->linear.x;
  double angular = msg->angular.z;

  if (linear > max_forward_speed_)
    linear = max_forward_speed_;
  else if (linear < -max_reverse_speed_)
    linear = -max_reverse_speed_;

  double steering_angle = 0.0;
  if (std::abs(linear) > 0.001) {
    double sign = (linear > 0) ? 1.0 : -1.0;
    steering_angle = sign * std::atan2(wheelbase_ * angular, std::abs(linear));
  } else if (std::abs(angular) > 0.001) {
    steering_angle = (angular > 0) ? max_steering_angle_ : -max_steering_angle_;
  }

  steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);

  if (std::abs(steering_angle) > 1e-6) {
    double R = wheelbase_ / std::tan(std::abs(steering_angle));
    double inner = std::atan(wheelbase_ / (R - track_width_ / 2.0));
    double outer = std::atan(wheelbase_ / (R + track_width_ / 2.0));
    if (steering_angle > 0) {
      fl_steering_angle_ = inner;
      fr_steering_angle_ = outer;
    } else {
      fl_steering_angle_ = -outer;
      fr_steering_angle_ = -inner;
    }
  } else {
    fl_steering_angle_ = 0.0;
    fr_steering_angle_ = 0.0;
  }

  double motor_rps = linear / (2.0 * M_PI * wheel_radius_);
  double motor_erpm = motor_rps * 60.0;
  float motor_cmd = static_cast<float>(motor_erpm / max_motor_speed_);

  motor_cmd = std::clamp(motor_cmd, -1.0f, 1.0f);

  auto motor_msg = std_msgs::msg::Float32();
  motor_msg.data = motor_cmd;
  motor_pub_->publish(motor_msg);

  auto steering_msg = std_msgs::msg::Float32();
  steering_msg.data = static_cast<float>(steering_angle);
  steering_pub_->publish(steering_msg);

  auto now_time = now();
  double dt = (now_time - last_time_).seconds();
  if (dt > 1.0) {
    dt = 0.02;
  }
  if (dt > 0.0) {
    updateOdometry(linear, steering_angle, dt);
  }
  last_time_ = now_time;

  curr_linear_vel_ = linear;
  curr_steering_angle_ = steering_angle;

  static int msg_count = 0;
  ++msg_count;
  if (msg_count <= 20 || msg_count % 5 == 0) {
    RCLCPP_INFO(get_logger(),
      "[rover_controller] msg#%d cmd_vel(%.3f, %.3f) -> steer=%.4f "
      "fl_steer=%.4f fr_steer=%.4f odom=(%.4f, %.4f) theta=%.4f dt=%.4f",
      msg_count, linear, angular, steering_angle,
      fl_steering_angle_, fr_steering_angle_,
      odom_x_, odom_y_, odom_theta_, dt);
  }
}

void RoverController::publishOdometry()
{
  auto now_time = now();
  auto odom = nav_msgs::msg::Odometry();
  odom.header.stamp = now_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";
  odom.pose.pose.position.x = odom_x_;
  odom.pose.pose.position.y = odom_y_;
  odom.pose.pose.orientation.z = std::sin(odom_theta_ * 0.5);
  odom.pose.pose.orientation.w = std::cos(odom_theta_ * 0.5);
  odom_pub_->publish(odom);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = now_time;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "base_footprint";
  tf.transform.translation.x = odom_x_;
  tf.transform.translation.y = odom_y_;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation.z = std::sin(odom_theta_ * 0.5);
  tf.transform.rotation.w = std::cos(odom_theta_ * 0.5);
  tf_broadcaster_->sendTransform(tf);
}

void RoverController::publishJointStates()
{
  auto now_time = now();
  double dt = (now_time - last_joint_state_time_).seconds();
  if (dt <= 0.0 || dt > 1.0) {
    dt = 0.02;
  }

  double v = curr_linear_vel_;
  double steer_c = curr_steering_angle_;

  double rl_omega = v / wheel_radius_;
  double rr_omega = v / wheel_radius_;
  double fl_omega, fr_omega;

  if (std::abs(steer_c) > 1e-6) {
    double R_center = wheelbase_ / std::tan(std::abs(steer_c));
    double v_front = v / std::cos(steer_c);
    double R_inner = R_center - track_width_ / 2.0;
    double R_outer = R_center + track_width_ / 2.0;
    fl_omega = v_front * R_inner / R_center / wheel_radius_;
    fr_omega = v_front * R_outer / R_center / wheel_radius_;
  } else {
    fl_omega = v / wheel_radius_;
    fr_omega = v / wheel_radius_;
  }

  rl_wheel_pos_ += rl_omega * dt;
  rr_wheel_pos_ += rr_omega * dt;
  fl_wheel_pos_ += fl_omega * dt;
  fr_wheel_pos_ += fr_omega * dt;

  rl_wheel_pos_ = std::fmod(rl_wheel_pos_, 2.0 * M_PI);
  rr_wheel_pos_ = std::fmod(rr_wheel_pos_, 2.0 * M_PI);
  fl_wheel_pos_ = std::fmod(fl_wheel_pos_, 2.0 * M_PI);
  fr_wheel_pos_ = std::fmod(fr_wheel_pos_, 2.0 * M_PI);

  last_joint_state_time_ = now_time;

  auto joint_state = sensor_msgs::msg::JointState();
  joint_state.header.stamp = now_time;
  joint_state.name = {
    "rl_wheel_joint", "rr_wheel_joint",
    "fl_wheel_joint", "fr_wheel_joint",
    "fl_steering_joint", "fr_steering_joint"
  };
  joint_state.position = {
    rl_wheel_pos_, rr_wheel_pos_,
    fl_wheel_pos_, fr_wheel_pos_,
    fl_steering_angle_, fr_steering_angle_
  };
  joint_state.velocity = {
    rl_omega, rr_omega,
    fl_omega, fr_omega,
    0.0, 0.0
  };
  joint_state_pub_->publish(joint_state);
}

void RoverController::updateOdometry(double linear_vel, double steering_angle, double dt)
{
  double omega = linear_vel * std::tan(steering_angle) / wheelbase_;
  double delta_theta = omega * dt;

  double delta_x, delta_y;
  if (std::abs(omega) > 0.001) {
    double R = linear_vel / omega;
    delta_x = R * std::sin(delta_theta);
    delta_y = R * (1.0 - std::cos(delta_theta));
  } else {
    delta_x = linear_vel * dt;
    delta_y = 0.0;
  }

  double cos_th = std::cos(odom_theta_);
  double sin_th = std::sin(odom_theta_);
  odom_x_ += delta_x * cos_th - delta_y * sin_th;
  odom_y_ += delta_x * sin_th + delta_y * cos_th;
  odom_theta_ += delta_theta;

  while (odom_theta_ > M_PI) odom_theta_ -= 2.0 * M_PI;
  while (odom_theta_ < -M_PI) odom_theta_ += 2.0 * M_PI;
}

}  // namespace ackermann_rover

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ackermann_rover::RoverController)
