cmake_minimum_required(VERSION 3.20)

project(sad_common
  VERSION 0.1.0
  DESCRIPTION "Code shared by the SAD book chapters: sensor types, UTM conversion, static IMU init, ESKF"
  LANGUAGES C CXX)

# Default to a Release build if the user doesn't specify one
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type specified, defaulting to 'Release'.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sad_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)
find_package(GeographicLib REQUIRED)

add_library(${PROJECT_NAME} STATIC
  src/eskf.cpp
  src/static_imu_init.cpp
  src/utm_convert.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Public: these types appear in the installed headers
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    rclcpp::rclcpp
    ${sad_msgs_TARGETS}
    Eigen3::Eigen
    Sophus::Sophus
  PRIVATE
    ${GeographicLib_LIBRARIES})

# GeographicLib is only used inside utm_convert.cpp
target_include_directories(${PROJECT_NAME} PRIVATE ${GeographicLib_INCLUDE_DIRS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall
  -Wextra
  -Wpedantic)

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sad_msgs Eigen3 Sophus)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
