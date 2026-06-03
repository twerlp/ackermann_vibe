#include "ackermann_rover/encoder_odometry.hpp"
#include <cmath>

namespace ackermann_rover
{

EncoderOdometry::EncoderOdometry(const rclcpp::NodeOptions & options)
: LifecycleNode("encoder_odometry", "", options), simulated_(false)
{
  RCLCPP_INFO(get_logger(), "EncoderOdometry created");
}

CallbackReturn EncoderOdometry::on_configure(const rclcpp_lifecycle::State &)
{
  declare_parameter("simulated", true);
  simulated_ = get_parameter("simulated").as_bool();

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom_raw", rclcpp::QoS(10).reliable());

  if (simulated_) {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(10).reliable(),
      std::bind(&EncoderOdometry::odomRelayCallback, this, std::placeholders::_1));
    RCLCPP_WARN(get_logger(),
      "Encoder odometry in simulated mode -- relaying /odom -> /odom_raw");
  }

  RCLCPP_INFO(get_logger(), "EncoderOdometry configured");
  return CallbackReturn::SUCCESS;
}

CallbackReturn EncoderOdometry::on_activate(const rclcpp_lifecycle::State &)
{
  odom_pub_->on_activate();
  if (!simulated_) {
    RCLCPP_ERROR(get_logger(),
      "Hardware encoder mode is not implemented. Set simulated=true for now.");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "EncoderOdometry activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn EncoderOdometry::on_deactivate(const rclcpp_lifecycle::State &)
{
  odom_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "EncoderOdometry deactivated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn EncoderOdometry::on_cleanup(const rclcpp_lifecycle::State &)
{
  odom_pub_.reset();
  odom_sub_.reset();
  RCLCPP_INFO(get_logger(), "EncoderOdometry cleaned up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn EncoderOdometry::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "EncoderOdometry shutdown");
  return CallbackReturn::SUCCESS;
}

void EncoderOdometry::odomRelayCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto relay = nav_msgs::msg::Odometry(*msg);
  relay.header.frame_id = "odom";
  relay.child_frame_id = "base_footprint";
  odom_pub_->publish(relay);

  static int log_counter = 0;
  if (++log_counter % 50 == 0) {
    RCLCPP_INFO(get_logger(),
      "[encoder_odometry] relayed odom_raw=(%.4f, %.4f) theta=%.4f",
      msg->pose.pose.position.x, msg->pose.pose.position.y,
      2.0 * std::asin(msg->pose.pose.orientation.z));
  }
}

}  // namespace ackermann_rover

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ackermann_rover::EncoderOdometry)
