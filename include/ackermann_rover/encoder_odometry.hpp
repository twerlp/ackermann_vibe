#ifndef ACKERMANN_ROVER__ENCODER_ODOMETRY_HPP_
#define ACKERMANN_ROVER__ENCODER_ODOMETRY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <nav_msgs/msg/odometry.hpp>

using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ackermann_rover
{

class EncoderOdometry : public LifecycleNode
{
public:
  explicit EncoderOdometry(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void odomRelayCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>> odom_pub_;
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> odom_sub_;
  bool simulated_;
};

}  // namespace ackermann_rover

#endif  // ACKERMANN_ROVER__ENCODER_ODOMETRY_HPP_
