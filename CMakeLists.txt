cmake_minimum_required(VERSION 3.16)
project(ackermann_rover)

if(CMAKE_VERSION VERSION_LESS "3.16.0")
  cmake_policy(SET CMP0077 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# ── Rover Controller (LifecycleNode, composable) ─────────────────
add_library(rover_controller SHARED
  src/rover_controller.cpp
)
ament_target_dependencies(rover_controller
  rclcpp rclcpp_lifecycle rclcpp_components
  std_msgs geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros
)
rclcpp_components_register_node(rover_controller
  PLUGIN "ackermann_rover::RoverController"
  EXECUTABLE rover_controller_node
)
ament_export_dependencies(rclcpp)
ament_export_include_directories(include)

# ── VESC Driver (LifecycleNode, composable) ─────────────────────
add_library(vesc_driver SHARED
  src/vesc_driver.cpp
)
ament_target_dependencies(vesc_driver
  rclcpp rclcpp_lifecycle rclcpp_components std_msgs
)
rclcpp_components_register_node(vesc_driver
  PLUGIN "ackermann_rover::VescDriver"
  EXECUTABLE vesc_driver_node
)

# ── Steering Servo (LifecycleNode, composable) ──────────────────
add_library(steering_servo SHARED
  src/steering_servo.cpp
)
ament_target_dependencies(steering_servo
  rclcpp rclcpp_lifecycle rclcpp_components std_msgs
)
rclcpp_components_register_node(steering_servo
  PLUGIN "ackermann_rover::SteeringServo"
  EXECUTABLE steering_servo_node
)

# ── Encoder Odometry (LifecycleNode, composable) ─────────────────
add_library(encoder_odometry SHARED
  src/encoder_odometry.cpp
)
ament_target_dependencies(encoder_odometry
  rclcpp rclcpp_lifecycle rclcpp_components
  nav_msgs
)
rclcpp_components_register_node(encoder_odometry
  PLUGIN "ackermann_rover::EncoderOdometry"
  EXECUTABLE encoder_odometry_node
)

# ── LiDAR Driver (LifecycleNode, composable) ────────────────────
add_library(lidar_driver SHARED
  src/lidar_driver.cpp
)
ament_target_dependencies(lidar_driver
  rclcpp rclcpp_lifecycle rclcpp_components
  sensor_msgs nav_msgs visualization_msgs
)
rclcpp_components_register_node(lidar_driver
  PLUGIN "ackermann_rover::LidarDriver"
  EXECUTABLE lidar_driver_node
)

# ── IMU Driver (LifecycleNode, composable) ──────────────────────
add_library(imu_driver SHARED
  src/imu_driver.cpp
)
ament_target_dependencies(imu_driver
  rclcpp rclcpp_lifecycle rclcpp_components sensor_msgs
)
rclcpp_components_register_node(imu_driver
  PLUGIN "ackermann_rover::ImuDriver"
  EXECUTABLE imu_driver_node
)

# ── Export ───────────────────────────────────────────────────────
ament_export_dependencies(
  rclcpp rclcpp_lifecycle std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs tf2 tf2_ros
)
ament_export_include_directories(include)
ament_export_libraries(
  rover_controller vesc_driver steering_servo encoder_odometry lidar_driver imu_driver
)

install(TARGETS
  rover_controller vesc_driver steering_servo
  encoder_odometry lidar_driver imu_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)

# URDF model, RViz config, and map files
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(FILES config/rover.rviz DESTINATION share/${PROJECT_NAME}/config)

# 2D visualizer
install(PROGRAMS src/rover_visualizer.py DESTINATION lib/${PROJECT_NAME})

# Map editor
install(PROGRAMS src/map_editor.py DESTINATION lib/${PROJECT_NAME})

# Joint state bootstrap
install(PROGRAMS src/joint_state_bootstrap.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
