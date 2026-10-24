cmake_minimum_required(VERSION 3.8)
project(navi2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF) # 禁用编译器扩展
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_BUILD_TYPE "Release")
  add_definitions(-Wall -Werror)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++17")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(rclcpp REQUIRED)

add_library(minco_goal_updater_bt_node SHARED src/goal_updater.cpp)
target_compile_definitions(minco_goal_updater_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(minco_goal_updater_bt_node
  behaviortree_cpp geometry_msgs nav2_behavior_tree rclcpp)
install(TARGETS minco_goal_updater_bt_node LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rcutils REQUIRED)
  ament_add_gtest(test_goal_updater test/test_goal_updater.cpp)
  ament_target_dependencies(test_goal_updater nav2_behavior_tree rclcpp rcutils)
  add_dependencies(test_goal_updater minco_goal_updater_bt_node)
  set_tests_properties(test_goal_updater PROPERTIES
    ENVIRONMENT "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:minco_goal_updater_bt_node>:$ENV{LD_LIBRARY_PATH}")
endif()

install(DIRECTORY launch maps config params behavior_tree
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
