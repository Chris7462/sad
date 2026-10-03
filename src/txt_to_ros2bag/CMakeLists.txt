cmake_minimum_required(VERSION 3.20)

project(txt_to_ros2bag
  VERSION 0.1.0
  DESCRIPTION "SAD txt sensor log to ROS 2 bag converter"
  LANGUAGES C CXX)

# Default to a Release build if the user doesn't specify one
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type specified, defaulting to 'Release'.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sad_msgs REQUIRED)

add_executable(${PROJECT_NAME}_node
  src/txt_to_ros2bag_node.cpp
  src/txt_to_ros2bag.cpp
  src/bag_writer.cpp)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME}_node PRIVATE
  ${rclcpp_TARGETS}
  ${rosbag2_cpp_TARGETS}
  ${rosbag2_storage_TARGETS}
  ${rosidl_runtime_cpp_TARGETS}
  ${sensor_msgs_TARGETS}
  ${sad_msgs_TARGETS})

target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_20)

target_compile_options(${PROJECT_NAME}_node PRIVATE
  -Wall
  -Wextra
  -Wpedantic)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
