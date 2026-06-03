#include "ackermann_rover/imu_driver.hpp"

namespace ackermann_rover
{

ImuDriver::ImuDriver(const rclcpp::NodeOptions & options)
: LifecycleNode("imu_driver", "", options),
  i2c_fd_(-1), simulated_(false)
{
  RCLCPP_INFO(get_logger(), "ImuDriver created");
}

CallbackReturn ImuDriver::on_configure(const rclcpp_lifecycle::State &)
{
  declare_parameter("i2c_device", "/dev/i2c-1");
  declare_parameter("i2c_address", 0x68);
  declare_parameter("imu_rate", 100.0);
  declare_parameter("frame_id", "imu_link");
  declare_parameter("accel_scale", 16384.0);
  declare_parameter("gyro_scale", 131.0);
  declare_parameter("simulated", true);

  i2c_device_ = get_parameter("i2c_device").as_string();
  i2c_address_ = static_cast<uint8_t>(get_parameter("i2c_address").as_int());
  imu_rate_ = get_parameter("imu_rate").as_double();
  frame_id_ = get_parameter("frame_id").as_string();
  simulated_ = get_parameter("simulated").as_bool();

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::QoS(10).best_effort());

  RCLCPP_INFO(get_logger(), "ImuDriver configured: I2C addr 0x%02X, %.0f Hz",
    i2c_address_, imu_rate_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImuDriver::on_activate(const rclcpp_lifecycle::State &)
{
  imu_pub_->on_activate();

  if (!simulated_) {
    // TODO: open I2C bus
    // i2c_fd_ = open(i2c_device_.c_str(), O_RDWR);
    // ioctl(i2c_fd_, I2C_SLAVE, i2c_address_);
    RCLCPP_ERROR(get_logger(),
      "Hardware IMU mode is not implemented. Set simulated=true for now.");
    return CallbackReturn::ERROR;
  }

  read_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / imu_rate_)),
    std::bind(&ImuDriver::readImuData, this));

  RCLCPP_INFO(get_logger(), "ImuDriver activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImuDriver::on_deactivate(const rclcpp_lifecycle::State &)
{
  read_timer_->cancel();
  imu_pub_->on_deactivate();

  if (!simulated_ && i2c_fd_ >= 0) {
    // close(i2c_fd_);
    i2c_fd_ = -1;
  }

  RCLCPP_INFO(get_logger(), "ImuDriver deactivated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImuDriver::on_cleanup(const rclcpp_lifecycle::State &)
{
  imu_pub_.reset();
  RCLCPP_INFO(get_logger(), "ImuDriver cleaned up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImuDriver::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (read_timer_) read_timer_->cancel();
  if (!simulated_ && i2c_fd_ >= 0) i2c_fd_ = -1;
  RCLCPP_INFO(get_logger(), "ImuDriver shutdown");
  return CallbackReturn::SUCCESS;
}

void ImuDriver::readImuData()
{
  auto imu_msg = sensor_msgs::msg::Imu();
  imu_msg.header.stamp = now();
  imu_msg.header.frame_id = frame_id_;

  if (simulated_) {
    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 9.81;
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;
    imu_msg.orientation.w = 1.0;
  } else {
    // TODO: Read from I2C (e.g., MPU6050 or ICM20948)
    // Read accelerometer registers (0x3B-0x40)
    // Read gyroscope registers (0x43-0x48)
    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 9.81;
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;
    imu_msg.orientation.w = 1.0;
  }

  imu_msg.linear_acceleration_covariance = {
    0.01, 0, 0,
    0, 0.01, 0,
    0, 0, 0.01
  };
  imu_msg.angular_velocity_covariance = {
    0.001, 0, 0,
    0, 0.001, 0,
    0, 0, 0.001
  };
  imu_msg.orientation_covariance = {
    0.01, 0, 0,
    0, 0.01, 0,
    0, 0, 0.01
  };

  imu_pub_->publish(imu_msg);
}

}  // namespace ackermann_rover

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ackermann_rover::ImuDriver)
