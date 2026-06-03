#include "ackermann_rover/vesc_driver.hpp"

namespace ackermann_rover
{

VescDriver::VescDriver(const rclcpp::NodeOptions & options)
: LifecycleNode("vesc_driver", "", options), serial_fd_(-1), simulated_(false)
{
  RCLCPP_INFO(get_logger(), "VescDriver created");
}

CallbackReturn VescDriver::on_configure(const rclcpp_lifecycle::State &)
{
  declare_parameter("serial_port", "/dev/vesc");
  declare_parameter("baud_rate", 115200);
  declare_parameter("vesc_id", 0);
  declare_parameter("max_duty", 0.95);
  declare_parameter("max_current", 40.0);
  declare_parameter("motor_poles", 14.0);
  declare_parameter("simulated", true);

  serial_port_ = get_parameter("serial_port").as_string();
  baud_rate_ = get_parameter("baud_rate").as_int();
  vesc_id_ = get_parameter("vesc_id").as_int();
  max_duty_ = static_cast<float>(get_parameter("max_duty").as_double());
  max_current_ = static_cast<float>(get_parameter("max_current").as_double());
  simulated_ = get_parameter("simulated").as_bool();

  vesc_state_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
    "vesc_state", rclcpp::QoS(10).best_effort());

  motor_sub_ = create_subscription<std_msgs::msg::Float32>(
    "cmd_motor", rclcpp::QoS(10).reliable(),
    std::bind(&VescDriver::motorCmdCallback, this, std::placeholders::_1));

  if (simulated_) {
    RCLCPP_WARN(get_logger(), "VESC running in simulated mode (no hardware)");
  }

  RCLCPP_INFO(get_logger(), "VescDriver configured on %s", serial_port_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn VescDriver::on_activate(const rclcpp_lifecycle::State &)
{
  vesc_state_pub_->on_activate();

  if (!simulated_) {
    // TODO: open serial port
    // serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
    RCLCPP_ERROR(get_logger(),
      "Hardware VESC mode is not implemented. Set simulated=true for now.");
    return CallbackReturn::ERROR;
  }

  read_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&VescDriver::readVescState, this));

  RCLCPP_INFO(get_logger(), "VescDriver activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn VescDriver::on_deactivate(const rclcpp_lifecycle::State &)
{
  read_timer_->cancel();
  setMotorDuty(0.0f);
  vesc_state_pub_->on_deactivate();

  if (!simulated_ && serial_fd_ >= 0) {
    // close(serial_fd_);
    serial_fd_ = -1;
  }

  RCLCPP_INFO(get_logger(), "VescDriver deactivated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn VescDriver::on_cleanup(const rclcpp_lifecycle::State &)
{
  vesc_state_pub_.reset();
  motor_sub_.reset();
  RCLCPP_INFO(get_logger(), "VescDriver cleaned up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn VescDriver::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (read_timer_) read_timer_->cancel();
  if (!simulated_ && serial_fd_ >= 0) serial_fd_ = -1;
  RCLCPP_INFO(get_logger(), "VescDriver shutdown");
  return CallbackReturn::SUCCESS;
}

void VescDriver::motorCmdCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  float duty = std::clamp(msg->data, -max_duty_, max_duty_);
  setMotorDuty(duty);
}

void VescDriver::setMotorDuty(float duty)
{
  // VESC UART command: COMM_SET_DUTY (id 5)
  // Packet: [2 (SOT)] [1 (len)] [vesc_id] [0 (pkt_id)] [5 (cmd)] [4 bytes duty*100000] [2 bytes CRC] [3 (EOT)]
  // TODO: implement actual serial write to VESC
  if (simulated_) {
    RCLCPP_DEBUG(get_logger(), "SIM: Setting motor duty cycle to %.3f", duty);
  }
}

void VescDriver::readVescState()
{
  auto state_msg = std_msgs::msg::Float32MultiArray();

  if (simulated_) {
    state_msg.data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  } else {
    // TODO: read actual VESC state via COMM_GET_VALUES
    // layout: [duty_cycle, erpm, current_in, voltage_in, amp_hours, temp_mos]
    state_msg.data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  }

  vesc_state_pub_->publish(state_msg);
}

}  // namespace ackermann_rover

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ackermann_rover::VescDriver)
