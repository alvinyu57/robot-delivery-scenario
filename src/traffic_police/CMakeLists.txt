cmake_minimum_required(VERSION 3.8)
project(traffic_police)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(delivery_robot_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS calib3d imgproc rgbd video)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(traffic_police_interfaces REQUIRED)

add_library(traffic_police_motion_estimator src/motion_estimator.cpp)
target_include_directories(traffic_police_motion_estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(traffic_police_motion_estimator SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS})
target_compile_features(traffic_police_motion_estimator PUBLIC cxx_std_17)
target_link_libraries(traffic_police_motion_estimator PUBLIC ${OpenCV_LIBRARIES})

add_executable(traffic_police_node src/traffic_police_node.cpp)
target_include_directories(traffic_police_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(traffic_police_node PUBLIC cxx_std_17)
ament_target_dependencies(
  traffic_police_node
  "cv_bridge"
  "delivery_robot_interfaces"
  "message_filters"
  "rcl_interfaces"
  "rclcpp"
  "sensor_msgs"
  "traffic_police_interfaces"
)
target_link_libraries(
  traffic_police_node
  traffic_police_motion_estimator
  ${OpenCV_LIBRARIES})

install(TARGETS traffic_police_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS traffic_police_motion_estimator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(
    test_motion_estimator
    test/test_motion_estimator.cpp)
  target_compile_features(test_motion_estimator PUBLIC cxx_std_17)
  target_link_libraries(
    test_motion_estimator
    traffic_police_motion_estimator)
endif()

ament_package()
