cmake_minimum_required(VERSION 3.16)
project(ur3_write_m)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(write_m_node src/write_m_node.cpp)
ament_target_dependencies(write_m_node
  rclcpp geometry_msgs moveit_ros_planning_interface moveit_msgs visualization_msgs)
target_compile_features(write_m_node PUBLIC cxx_std_17)

add_executable(draw_circle_node src/draw_circle_node.cpp)
ament_target_dependencies(draw_circle_node
  rclcpp geometry_msgs moveit_ros_planning_interface moveit_msgs visualization_msgs)
target_compile_features(draw_circle_node PUBLIC cxx_std_17)

install(TARGETS write_m_node draw_circle_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
