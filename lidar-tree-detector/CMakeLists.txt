cmake_minimum_required(VERSION 3.5)
project(lidar_tree_detector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(lidar_tree_detector_node src/lidar_tree_detector_node.cpp)
ament_target_dependencies(lidar_tree_detector_node rclcpp sensor_msgs)

install(TARGETS
  lidar_tree_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()