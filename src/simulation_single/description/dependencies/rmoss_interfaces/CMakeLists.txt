cmake_minimum_required(VERSION 3.5)

project(rmoss_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(idl_files
  "msg/Gimbal.idl"
  "msg/GimbalCmd.idl"
  "msg/ShootCmd.idl"
  "msg/ChassisCmd.idl"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${idl_files}
  DEPENDENCIES geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
