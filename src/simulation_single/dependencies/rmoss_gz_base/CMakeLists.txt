cmake_minimum_required(VERSION 3.8)
project(rmoss_gz_base)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(rmua19_robot_base src/sim_chassis_bridge.cpp)
ament_target_dependencies(rmua19_robot_base rclcpp geometry_msgs)
target_link_libraries(rmua19_robot_base
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(TARGETS rmua19_robot_base DESTINATION lib/${PROJECT_NAME})
ament_package()
