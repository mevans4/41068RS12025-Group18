cmake_minimum_required(VERSION 3.8)
project(tree_trunk_detector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_library(trunk_detector src/trunk_detector.cpp)
ament_target_dependencies(trunk_detector rclcpp sensor_msgs geometry_msgs)

add_executable(trunk_detector_node src/trunk_detector_node.cpp)
ament_target_dependencies(trunk_detector_node rclcpp sensor_msgs geometry_msgs)
target_link_libraries(trunk_detector_node trunk_detector)

install(TARGETS
  trunk_detector
  trunk_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
