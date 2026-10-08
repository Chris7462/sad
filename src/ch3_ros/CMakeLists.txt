cmake_minimum_required(VERSION 3.20)

project(ch3_ros
  VERSION 0.1.0
  DESCRIPTION "SAD chapter 3: IMU integration and GNSS in ROS 2"
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
find_package(manif REQUIRED)
#find_package(Sophus REQUIRED)
#find_package(glog REQUIRED)
find_package(sad_msgs REQUIRED)

list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)
find_package(GeographicLib REQUIRED)

# IMU integration node
add_executable(imu_ros_node
  src/imu_ros_node.cpp
  src/imu_ros.cpp
  src/imu_integration.cpp)

target_include_directories(imu_ros_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(imu_ros_node
  PRIVATE
    rclcpp::rclcpp
    ${tf2_ros_TARGETS}
    ${tf2_eigen_TARGETS}
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    Eigen3::Eigen
    MANIF::manif)

target_compile_features(imu_ros_node PRIVATE cxx_std_20)

target_compile_options(imu_ros_node PRIVATE
  -Wall
  -Wextra
  -Wpedantic)

# GNSS node
add_executable(gnss_ros_node
  src/gnss_ros_node.cpp
  src/gnss_ros.cpp
  src/utm_convert.cpp)

target_include_directories(gnss_ros_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GeographicLib_INCLUDE_DIRS})

target_link_libraries(gnss_ros_node
  PRIVATE
    rclcpp::rclcpp
    ${tf2_ros_TARGETS}
    ${tf2_eigen_TARGETS}
    ${sensor_msgs_TARGETS}
    Eigen3::Eigen
    #Sophus::Sophus
    MANIF::manif
    #glog::glog
    ${sad_msgs_TARGETS}
    ${GeographicLib_LIBRARIES})

target_compile_features(gnss_ros_node PRIVATE cxx_std_20)

target_compile_options(gnss_ros_node PRIVATE
  -Wall
  -Wextra
  -Wpedantic)

# ESKF GNSS + IMU fusion node
add_executable(eskf_ros_node
  src/eskf_ros_node.cpp
  src/eskf_ros.cpp
  src/eskf.cpp
  src/static_init.cpp
  src/utm_convert.cpp)

target_include_directories(eskf_ros_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GeographicLib_INCLUDE_DIRS})

target_link_libraries(eskf_ros_node
  PRIVATE
    rclcpp::rclcpp
    ${tf2_ros_TARGETS}
    ${tf2_eigen_TARGETS}
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    Eigen3::Eigen
    MANIF::manif
    ${sad_msgs_TARGETS}
    ${GeographicLib_LIBRARIES})

target_compile_features(eskf_ros_node PRIVATE cxx_std_20)

target_compile_options(eskf_ros_node PRIVATE
  -Wall
  -Wextra
  -Wpedantic)

install(TARGETS imu_ros_node gnss_ros_node eskf_ros_node
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
