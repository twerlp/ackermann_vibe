#ifndef ACKERMANN_ROVER__IMU_DRIVER_HPP_
#define ACKERMANN_ROVER__IMU_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>

using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ackermann_rover
{

class ImuDriver : public LifecycleNode
{
public:
  explicit ImuDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void readImuData();

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>> imu_pub_;

  rclcpp::TimerBase::SharedPtr read_timer_;

  std::string i2c_device_;
  uint8_t i2c_address_;
  int i2c_fd_;
  double imu_rate_;
  std::string frame_id_;
  bool simulated_;
};

}  // namespace ackermann_rover

#endif  // ACKERMANN_ROVER__IMU_DRIVER_HPP_
