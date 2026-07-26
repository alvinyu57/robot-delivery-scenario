cmake_minimum_required(VERSION 3.10)
project(delivery_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch models rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_launch_test(
    test/test_gazebo_delivery_mission.py
    TARGET test_gazebo_delivery_mission
    TIMEOUT 600
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
  set_tests_properties(
    test_gazebo_delivery_mission
    PROPERTIES RUN_SERIAL TRUE
  )
endif()

ament_package()
