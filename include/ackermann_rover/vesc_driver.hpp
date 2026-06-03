#ifndef ACKERMANN_ROVER__VESC_DRIVER_HPP_
#define ACKERMANN_ROVER__VESC_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <string>

using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ackermann_rover
{

class VescDriver : public LifecycleNode
{
public:
  explicit VescDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void motorCmdCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void readVescState();
  void setMotorDuty(float duty);

  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float32>> motor_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32MultiArray>> vesc_state_pub_;

  rclcpp::TimerBase::SharedPtr read_timer_;

  std::string serial_port_;
  int baud_rate_;
  int serial_fd_;
  int vesc_id_;
  float max_duty_;
  float max_current_;
  bool simulated_;
};

}  // namespace ackermann_rover

#endif  // ACKERMANN_ROVER__VESC_DRIVER_HPP_
