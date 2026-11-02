cmake_minimum_required(VERSION 3.10)
project(minco_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)

set(ACADOS_INSTALL_DIR "$ENV{ACADOS_SOURCE_DIR}" CACHE PATH "acados installation prefix")
find_path(ACADOS_INCLUDE_DIR acados_c/ocp_nlp_interface.h
  HINTS ${ACADOS_INSTALL_DIR} PATH_SUFFIXES include)
find_library(ACADOS_LIBRARY acados HINTS ${ACADOS_INSTALL_DIR} PATH_SUFFIXES lib)
find_library(HPIPM_LIBRARY hpipm HINTS ${ACADOS_INSTALL_DIR} PATH_SUFFIXES lib)
find_library(BLASFEO_LIBRARY blasfeo HINTS ${ACADOS_INSTALL_DIR} PATH_SUFFIXES lib)
if(NOT ACADOS_INCLUDE_DIR OR NOT ACADOS_LIBRARY OR NOT HPIPM_LIBRARY OR NOT BLASFEO_LIBRARY)
  message(FATAL_ERROR "acados, HPIPM and BLASFEO are required. Set ACADOS_INSTALL_DIR to their installation prefix.")
endif()

set(ACADOS_GENERATED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/model/report_codegen)
file(GLOB_RECURSE ACADOS_GENERATED_SOURCES CONFIGURE_DEPENDS ${ACADOS_GENERATED_DIR}/*.c)
list(FILTER ACADOS_GENERATED_SOURCES EXCLUDE REGEX "/main_[^/]*\\.c$")
if(NOT ACADOS_GENERATED_SOURCES)
  message(FATAL_ERROR "Generated report MPC sources are missing from ${ACADOS_GENERATED_DIR}")
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/report_mpc_solver.cpp
  src/minco_mpc_controller.cpp
  src/performance/mpc_performance_monitor.cpp
  ${ACADOS_GENERATED_SOURCES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE ${ACADOS_GENERATED_DIR} ${ACADOS_INCLUDE_DIR}
    ${ACADOS_INCLUDE_DIR}/acados ${ACADOS_INCLUDE_DIR}/blasfeo/include
    ${ACADOS_INCLUDE_DIR}/hpipm/include)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen ${ACADOS_LIBRARY} ${HPIPM_LIBRARY} ${BLASFEO_LIBRARY})
get_filename_component(ACADOS_LIBRARY_DIR ${ACADOS_LIBRARY} DIRECTORY)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "${ACADOS_LIBRARY_DIR}")

pluginlib_export_plugin_description_file(nav2_core minco_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(report_integration_test test/report_integration_test.cpp)
  target_include_directories(report_integration_test PRIVATE include)
  target_link_libraries(report_integration_test ${PROJECT_NAME} Eigen3::Eigen)
  ament_add_gtest(speed_limit_test test/speed_limit_test.cpp)
  target_include_directories(speed_limit_test PRIVATE include)
  target_link_libraries(speed_limit_test ${PROJECT_NAME} Eigen3::Eigen)
endif()

install(DIRECTORY model/ DESTINATION share/${PROJECT_NAME}/model
  FILES_MATCHING PATTERN "*.py" PATTERN "README.md")
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
