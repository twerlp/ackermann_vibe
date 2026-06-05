#include "ackermann_rover/steering_servo.hpp"

namespace ackermann_rover
{

SteeringServo::SteeringServo(const rclcpp::NodeOptions & options)
: LifecycleNode("steering_servo", "", options), simulated_(false)
{
  RCLCPP_INFO(get_logger(), "SteeringServo created");
}

CallbackReturn SteeringServo::on_configure(const rclcpp_lifecycle::State &)
{
  declare_parameter<int>("pwm_pin");
  declare_parameter<int>("pwm_chip");
  declare_parameter<double>("center_pulse_us");
  declare_parameter<double>("range_pulse_us");
  declare_parameter<double>("max_angle_rad");
  declare_parameter<double>("min_angle_rad");
  declare_parameter<bool>("simulated");

  pwm_pin_ = get_parameter("pwm_pin").as_int();
  pwm_chip_ = get_parameter("pwm_chip").as_int();
  center_pulse_us_ = static_cast<float>(get_parameter("center_pulse_us").as_double());
  range_pulse_us_ = static_cast<float>(get_parameter("range_pulse_us").as_double());
  max_angle_rad_ = static_cast<float>(get_parameter("max_angle_rad").as_double());
  min_angle_rad_ = static_cast<float>(get_parameter("min_angle_rad").as_double());
  simulated_ = get_parameter("simulated").as_bool();

  steering_sub_ = create_subscription<std_msgs::msg::Float32>(
    "cmd_steering", rclcpp::QoS(10).reliable(),
    std::bind(&SteeringServo::steeringCmdCallback, this, std::placeholders::_1));

  if (simulated_) {
    RCLCPP_WARN(get_logger(), "Steering servo running in simulated mode");
  }

  RCLCPP_INFO(get_logger(), "SteeringServo configured: pwm_pin=%d", pwm_pin_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn SteeringServo::on_activate(const rclcpp_lifecycle::State &)
{
  if (!simulated_) {
    // TODO: initialize PWM on Jetson Nano
    // Example: echo 0 > /sys/class/pwm/pwmchip0/export
    // Set period, duty cycle via sysfs or GPIO library
    RCLCPP_ERROR(get_logger(),
      "Hardware PWM mode is not implemented. Set simulated=true for now.");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "SteeringServo activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SteeringServo::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Center the servo before deactivating
  if (!simulated_) {
    // TODO: set PWM to center position
  }
  RCLCPP_INFO(get_logger(), "SteeringServo deactivated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SteeringServo::on_cleanup(const rclcpp_lifecycle::State &)
{
  steering_sub_.reset();
  RCLCPP_INFO(get_logger(), "SteeringServo cleaned up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SteeringServo::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "SteeringServo shutdown");
  return CallbackReturn::SUCCESS;
}

void SteeringServo::steeringCmdCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  float angle = std::clamp(msg->data, min_angle_rad_, max_angle_rad_);
  float pulse_us = angleToPulseWidth(angle);

  if (simulated_) {
    RCLCPP_DEBUG(get_logger(), "SIM: Steering angle=%.3f rad, pulse=%.0f us",
      angle, pulse_us);
  } else {
    // TODO: write pulse_us to PWM pin via sysfs or GPIO lib
  }
}

float SteeringServo::angleToPulseWidth(float angle_rad)
{
  float fraction = (angle_rad - min_angle_rad_) / (max_angle_rad_ - min_angle_rad_);
  fraction = std::clamp(fraction, 0.0f, 1.0f);
  float pulse = center_pulse_us_ - range_pulse_us_ + 2.0f * range_pulse_us_ * fraction;
  return pulse;
}

}  // namespace ackermann_rover

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ackermann_rover::SteeringServo)
