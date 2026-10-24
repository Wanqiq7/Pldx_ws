cmake_minimum_required(VERSION 3.10)
project(minco_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

find_package(qpOASES QUIET)
if(NOT qpOASES_FOUND)
  find_path(QPOASES_INCLUDE_DIR qpOASES.hpp PATH_SUFFIXES qpOASES qpoases)
  find_library(QPOASES_LIBRARY qpOASES)
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/dynamic_mpc_solver.cpp
  src/disturbance.cpp
  src/wheel_odometry.cpp
  src/minco_mpc_controller.cpp
  src/performance/mpc_performance_monitor.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

set(_QPOASES_LINKED FALSE)

set(_QPOASES_VENDOR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/qpOASES)
if(EXISTS "${_QPOASES_VENDOR_DIR}/include/qpOASES.hpp" AND EXISTS "${_QPOASES_VENDOR_DIR}/src")
  file(GLOB _QPOASES_VENDOR_SRC "${_QPOASES_VENDOR_DIR}/src/*.cpp")
  add_library(qpOASES_vendor STATIC ${_QPOASES_VENDOR_SRC})
  set_target_properties(qpOASES_vendor PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(qpOASES_vendor PUBLIC "${_QPOASES_VENDOR_DIR}/include")
  target_compile_definitions(qpOASES_vendor PRIVATE __NO_COPYRIGHT__ LINUX)

  target_link_libraries(${PROJECT_NAME} qpOASES_vendor)
  set(_QPOASES_LINKED TRUE)
endif()

pluginlib_export_plugin_description_file(nav2_core minco_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(dynamic_mpc_solver_test test/dynamic_mpc_solver_test.cpp src/dynamic_mpc_solver.cpp)
  target_include_directories(dynamic_mpc_solver_test PRIVATE include)
  target_link_libraries(dynamic_mpc_solver_test Eigen3::Eigen qpOASES_vendor)
  ament_add_gtest(disturbance_test test/disturbance_test.cpp src/disturbance.cpp)
  target_include_directories(disturbance_test PRIVATE include)
  target_link_libraries(disturbance_test Eigen3::Eigen)
  ament_add_gtest(wheel_odometry_test test/wheel_odometry_test.cpp src/wheel_odometry.cpp)
  target_include_directories(wheel_odometry_test PRIVATE include)
  target_link_libraries(wheel_odometry_test Eigen3::Eigen)
  ament_add_gtest(controller_freshness_test test/controller_freshness_test.cpp)
  target_link_libraries(controller_freshness_test ${PROJECT_NAME})
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
