#ifndef ACKERMANN_ROVER__LIDAR_DRIVER_HPP_
#define ACKERMANN_ROVER__LIDAR_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <string>
#include <vector>
#include <fstream>

using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ackermann_rover
{

struct ObstacleSegment
{
  double x1, y1, x2, y2;
};

class LidarDriver : public LifecycleNode
{
public:
  explicit LidarDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void readLidarData();
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void loadSegmentsFromFile(const std::string & path);
  double rayCast(double lx, double ly, double angle) const;
  double raySegmentIntersect(double ox, double oy, double dx, double dy,
                              double x1, double y1, double x2, double y2) const;
  void publishObstacleMarkers();

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>> scan_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>> marker_pub_;
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> odom_sub_;

  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::TimerBase::SharedPtr marker_timer_;

  std::string serial_port_;
  int baud_rate_;
  int serial_fd_;
  float scan_rate_;
  float range_min_;
  float range_max_;
  float angle_min_;
  float angle_max_;
  float angle_increment_;
  int num_samples_;
  std::string frame_id_;
  std::string map_file_;
  bool simulated_;

  double odom_x_, odom_y_, odom_theta_;
  bool has_odom_;

  double lidar_offset_x_, lidar_offset_y_;
  std::vector<ObstacleSegment> obstacles_;
};

}  // namespace ackermann_rover

#endif  // ACKERMANN_ROVER__LIDAR_DRIVER_HPP_
