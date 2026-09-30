cmake_minimum_required(VERSION 3.8)
project(sim_nav_adapter)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  sim_nav_adapter/ground_truth_odom.py
  sim_nav_adapter/point_lio_odom.py
  sim_nav_adapter/navigation_cloud_adapter.py
  sim_nav_adapter/cmd_vel_adapter.py
  scripts/check_sim_tf.py
  scripts/check_sim_costmap.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_transform_math test/test_transform_math.py)
  ament_add_pytest_test(test_navigation_cloud_adapter test/test_navigation_cloud_adapter.py)
  ament_add_pytest_test(test_velocity_command test/test_velocity_command.py)
  ament_add_pytest_test(test_cmd_vel_adapter test/test_cmd_vel_adapter.py)
  ament_add_pytest_test(test_costmap_check test/test_costmap_check.py)
endif()

ament_package()
