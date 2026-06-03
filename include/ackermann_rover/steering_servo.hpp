#ifndef ACKERMANN_ROVER__STEERING_SERVO_HPP_
#define ACKERMANN_ROVER__STEERING_SERVO_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float32.hpp>

using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ackermann_rover
{

class SteeringServo : public LifecycleNode
{
public:
  explicit SteeringServo(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void steeringCmdCallback(const std_msgs::msg::Float32::SharedPtr msg);
  float angleToPulseWidth(float angle_rad);

  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float32>> steering_sub_;

  int pwm_pin_;
  int pwm_chip_;
  float center_pulse_us_;
  float range_pulse_us_;
  float max_angle_rad_;
  float min_angle_rad_;
  bool simulated_;
};

}  // namespace ackermann_rover

#endif  // ACKERMANN_ROVER__STEERING_SERVO_HPP_
