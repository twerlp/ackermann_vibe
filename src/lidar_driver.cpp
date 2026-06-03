#include "ackermann_rover/lidar_driver.hpp"
#include <cmath>
#include <sstream>

namespace ackermann_rover
{

LidarDriver::LidarDriver(const rclcpp::NodeOptions & options)
: LifecycleNode("lidar_driver", "", options), serial_fd_(-1), simulated_(false),
  odom_x_(0.0), odom_y_(0.0), odom_theta_(0.0), has_odom_(false),
  lidar_offset_x_(0.65), lidar_offset_y_(0.0)
{
  RCLCPP_INFO(get_logger(), "LidarDriver created");
}

CallbackReturn LidarDriver::on_configure(const rclcpp_lifecycle::State &)
{
  declare_parameter("serial_port", "/dev/lidar");
  declare_parameter("baud_rate", 230400);
  declare_parameter("scan_rate", 10.0);
  declare_parameter("range_min", 0.12);
  declare_parameter("range_max", 12.0);
  declare_parameter("angle_min", -M_PI);
  declare_parameter("angle_max", M_PI);
  declare_parameter("num_samples", 360);
  declare_parameter("frame_id", "lidar_link");
  declare_parameter("simulated", true);
  declare_parameter("map_file", "");
  declare_parameter("lidar_offset_x", 0.65);
  declare_parameter("lidar_offset_y", 0.0);

  serial_port_ = get_parameter("serial_port").as_string();
  baud_rate_ = get_parameter("baud_rate").as_int();
  scan_rate_ = get_parameter("scan_rate").as_double();
  range_min_ = get_parameter("range_min").as_double();
  range_max_ = get_parameter("range_max").as_double();
  angle_min_ = get_parameter("angle_min").as_double();
  angle_max_ = get_parameter("angle_max").as_double();
  num_samples_ = get_parameter("num_samples").as_int();
  frame_id_ = get_parameter("frame_id").as_string();
  simulated_ = get_parameter("simulated").as_bool();
  map_file_ = get_parameter("map_file").as_string();
  lidar_offset_x_ = get_parameter("lidar_offset_x").as_double();
  lidar_offset_y_ = get_parameter("lidar_offset_y").as_double();

  angle_increment_ = (angle_max_ - angle_min_) / num_samples_;

  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::QoS(10).best_effort());

  if (simulated_) {
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "gym_obstacles", rclcpp::QoS(10).reliable());

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(10).reliable(),
      std::bind(&LidarDriver::odomCallback, this, std::placeholders::_1));

    if (!map_file_.empty()) {
      loadSegmentsFromFile(map_file_);
    } else {
      buildDefaultGymMap();
    }
    RCLCPP_WARN(get_logger(), "LiDAR running in simulated mode with %zu obstacles",
      obstacles_.size());
  }

  RCLCPP_INFO(get_logger(), "LidarDriver configured on %s (%d samples, %.2f Hz)",
    serial_port_.c_str(), num_samples_, scan_rate_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn LidarDriver::on_activate(const rclcpp_lifecycle::State &)
{
  scan_pub_->on_activate();

  if (simulated_) {
    marker_pub_->on_activate();
    publishObstacleMarkers();
    marker_timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&LidarDriver::publishObstacleMarkers, this));
  }

  if (!simulated_) {
    RCLCPP_ERROR(get_logger(),
      "Hardware LiDAR mode is not implemented. Set simulated=true for now.");
    return CallbackReturn::ERROR;
  }

  read_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / scan_rate_)),
    std::bind(&LidarDriver::readLidarData, this));

  RCLCPP_INFO(get_logger(), "LidarDriver activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LidarDriver::on_deactivate(const rclcpp_lifecycle::State &)
{
  read_timer_->cancel();
  scan_pub_->on_deactivate();
  if (simulated_) marker_pub_->on_deactivate();

  if (!simulated_ && serial_fd_ >= 0) {
    serial_fd_ = -1;
  }

  RCLCPP_INFO(get_logger(), "LidarDriver deactivated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LidarDriver::on_cleanup(const rclcpp_lifecycle::State &)
{
  scan_pub_.reset();
  marker_pub_.reset();
  odom_sub_.reset();
  RCLCPP_INFO(get_logger(), "LidarDriver cleaned up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LidarDriver::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (read_timer_) read_timer_->cancel();
  if (!simulated_ && serial_fd_ >= 0) serial_fd_ = -1;
  RCLCPP_INFO(get_logger(), "LidarDriver shutdown");
  return CallbackReturn::SUCCESS;
}

void LidarDriver::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_x_ = msg->pose.pose.position.x;
  odom_y_ = msg->pose.pose.position.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  odom_theta_ = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);
  has_odom_ = true;
}

void LidarDriver::buildDefaultGymMap()
{
  const double X = 4.0, Y = 4.0;
  obstacles_.clear();

  // Outer walls — 8x8m room centered at origin
  obstacles_.push_back({-X, -Y,  X, -Y});
  obstacles_.push_back({ X, -Y,  X,  Y});
  obstacles_.push_back({ X,  Y, -X,  Y});
  obstacles_.push_back({-X,  Y, -X, -Y});

  // Box at upper-right (2, 2), 1m x 0.6m
  obstacles_.push_back({ 1.7,  1.7,  2.7,  1.7});
  obstacles_.push_back({ 2.7,  1.7,  2.7,  2.3});
  obstacles_.push_back({ 2.7,  2.3,  1.7,  2.3});
  obstacles_.push_back({ 1.7,  2.3,  1.7,  1.7});

  // Box at lower-left (-2, -2.5), 1.2m x 0.8m
  obstacles_.push_back({-2.6, -2.9, -1.4, -2.9});
  obstacles_.push_back({-1.4, -2.9, -1.4, -2.1});
  obstacles_.push_back({-1.4, -2.1, -2.6, -2.1});
  obstacles_.push_back({-2.6, -2.1, -2.6, -2.9});

  // Small box at (0.5, -1), 0.6m x 0.4m
  obstacles_.push_back({ 0.2, -1.2,  0.8, -1.2});
  obstacles_.push_back({ 0.8, -1.2,  0.8, -0.8});
  obstacles_.push_back({ 0.8, -0.8,  0.2, -0.8});
  obstacles_.push_back({ 0.2, -0.8,  0.2, -1.2});

  // Long wall segment in the middle (vertical barrier leaving gaps)
  obstacles_.push_back({-0.2, -1.5, -0.2,  0.5});
  obstacles_.push_back({-0.2,  2.0, -0.2,  3.5});
}

double LidarDriver::raySegmentIntersect(double ox, double oy, double dx, double dy,
                                         double x1, double y1, double x2, double y2) const
{
  double sx = x2 - x1;
  double sy = y2 - y1;

  double denom = dx * sy - dy * sx;
  if (std::abs(denom) < 1e-9)
    return -1.0;

  double t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom;
  double u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom;

  if (t > 0.0 && u >= 0.0 && u <= 1.0)
    return t;

  return -1.0;
}

double LidarDriver::rayCast(double lx, double ly, double angle) const
{
  double dx = std::cos(angle);
  double dy = std::sin(angle);
  double best_t = range_max_;

  for (const auto & seg : obstacles_) {
    double t = raySegmentIntersect(lx, ly, dx, dy, seg.x1, seg.y1, seg.x2, seg.y2);
    if (t > 0.0 && t < best_t)
      best_t = t;
  }

  if (best_t < range_min_)
    best_t = 0.0;

  return best_t;
}

void LidarDriver::readLidarData()
{
  auto scan = sensor_msgs::msg::LaserScan();
  scan.header.stamp = now();
  scan.header.frame_id = frame_id_;
  scan.angle_min = angle_min_;
  scan.angle_max = angle_max_;
  scan.angle_increment = angle_increment_;
  scan.time_increment = 0.0;
  scan.scan_time = 1.0 / scan_rate_;
  scan.range_min = range_min_;
  scan.range_max = range_max_;

  scan.ranges.resize(num_samples_);
  scan.intensities.resize(num_samples_);

  if (simulated_) {
    double lx = odom_x_ + lidar_offset_x_ * std::cos(odom_theta_) - lidar_offset_y_ * std::sin(odom_theta_);
    double ly = odom_y_ + lidar_offset_x_ * std::sin(odom_theta_) + lidar_offset_y_ * std::cos(odom_theta_);

    for (int i = 0; i < num_samples_; ++i) {
      double ray_angle = odom_theta_ + angle_min_ + i * angle_increment_;
      double dist = rayCast(lx, ly, ray_angle);
      scan.ranges[i] = static_cast<float>(dist);
      scan.intensities[i] = (dist < range_max_) ? 100.0f : 0.0f;
    }
  } else {
    for (int i = 0; i < num_samples_; ++i) {
      scan.ranges[i] = 0.0f;
      scan.intensities[i] = 0.0f;
    }
  }

  scan_pub_->publish(scan);
}

void LidarDriver::loadSegmentsFromFile(const std::string & path)
{
  obstacles_.clear();
  std::ifstream file(path);
  if (!file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Cannot open map file: %s", path.c_str());
    return;
  }
  std::string line;
  int count = 0;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream iss(line);
    double x1, y1, x2, y2;
    if (iss >> x1 >> y1 >> x2 >> y2) {
      obstacles_.push_back({x1, y1, x2, y2});
      ++count;
    }
  }
  RCLCPP_INFO(get_logger(), "Loaded %d segments from %s", count, path.c_str());
}

void LidarDriver::publishObstacleMarkers()
{
  auto markers = visualization_msgs::msg::MarkerArray();

  // Wall segments
  for (size_t i = 0; i < obstacles_.size(); ++i) {
    const auto & s = obstacles_[i];
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "odom";
    m.header.stamp = now();
    m.ns = "gym_walls";
    m.id = static_cast<int>(i);
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;

    double dx = s.x2 - s.x1;
    double dy = s.y2 - s.y1;
    double cx = (s.x1 + s.x2) / 2.0;
    double cy = (s.y1 + s.y2) / 2.0;
    double len = std::hypot(dx, dy);
    double yaw = std::atan2(dy, dx);

    m.pose.position.x = cx;
    m.pose.position.y = cy;
    m.pose.position.z = 0.25;
    m.pose.orientation.z = std::sin(yaw * 0.5);
    m.pose.orientation.w = std::cos(yaw * 0.5);
    m.scale.x = (len > 0.05) ? len : 0.05;
    m.scale.y = 0.05;
    m.scale.z = 0.5;
    m.color.r = 0.6f;
    m.color.g = 0.4f;
    m.color.b = 0.2f;
    m.color.a = 0.9f;
    m.lifetime.nanosec = 0;  // zero = permanent, never auto-delete

    markers.markers.push_back(m);
  }

  marker_pub_->publish(markers);
}

}  // namespace ackermann_rover

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ackermann_rover::LidarDriver)
