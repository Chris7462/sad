cmake_minimum_required(VERSION 3.20)

project(ch4_ros
  VERSION 0.1.0
  DESCRIPTION "SAD chapter 4: IMU preintegration and g2o GINS in ROS 2"
  LANGUAGES C CXX)

# Default to a Release build if the user doesn't specify one
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type specified, defaulting to 'Release'.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(g2o REQUIRED)
find_package(sad_msgs REQUIRED)
find_package(sad_common REQUIRED)

# GINS with IMU preintegration node
add_executable(gins_ros_node
  src/gins_ros_node.cpp
  src/gins_ros.cpp
  src/gins_pre_integ.cpp
  src/g2o_types.cpp)

target_include_directories(gins_ros_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(gins_ros_node
  PRIVATE
    rclcpp::rclcpp
    ${tf2_ros_TARGETS}
    ${tf2_eigen_TARGETS}
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    Eigen3::Eigen
    Sophus::Sophus
    g2o::core
    g2o::stuff
    g2o::solver_eigen
    ${sad_msgs_TARGETS}
    sad_common::sad_common)

target_compile_features(gins_ros_node PRIVATE cxx_std_20)

target_compile_options(gins_ros_node PRIVATE
  -Wall
  -Wextra
  -Wpedantic)

install(TARGETS gins_ros_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
