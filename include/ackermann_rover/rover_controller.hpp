#ifndef ACKERMANN_ROVER__ROVER_CONTROLLER_HPP_
#define ACKERMANN_ROVER__ROVER_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ackermann_rover
{

class RoverController : public LifecycleNode
{
public:
  explicit RoverController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publishOdometry();
  void publishJointStates();
  void updateOdometry(double linear_vel, double steering_angle, double dt);

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>> motor_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>> steering_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>> odom_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>> joint_state_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  rclcpp::Time last_time_;
  rclcpp::Time last_joint_state_time_;

  double curr_linear_vel_;
  double curr_steering_angle_;
  double fl_steering_angle_;
  double fr_steering_angle_;

  double wheelbase_;
  double track_width_;
  double wheel_radius_;
  double max_steering_angle_;
  double max_motor_speed_;
  double max_forward_speed_;
  double max_reverse_speed_;

  double odom_x_;
  double odom_y_;
  double odom_theta_;

  double rl_wheel_pos_;
  double rr_wheel_pos_;
  double fl_wheel_pos_;
  double fr_wheel_pos_;
};

}  // namespace ackermann_rover

#endif  // ACKERMANN_ROVER__ROVER_CONTROLLER_HPP_
