#pragma once

// C++ standard library
#include <cstddef>
#include <memory>
#include <sstream>
#include <string>

// ROS 2
#include <rclcpp/rclcpp.hpp>

// local
#include "txt_to_ros2bag/bag_writer.hpp"


class Txt2BagNode : public rclcpp::Node
{
public:
  Txt2BagNode();
  ~Txt2BagNode() = default;

  // Read the whole txt file and write it to the bag. Returns false on failure.
  bool convert();

private:
  // Each returns false if the line could not be parsed.
  bool write_imu(std::istringstream & ss);
  bool write_gnss(std::istringstream & ss);
  bool write_odom(std::istringstream & ss);

  // Seconds (as stored in the txt file) -> ROS time. Also tracks ordering.
  rclcpp::Time to_ros_time(double seconds);

  std::unique_ptr<BagWriter> bag_writer_;

  std::string txt_path_;
  std::string output_bag_;

  std::string imu_topic_;
  std::string gnss_topic_;
  std::string odom_topic_;

  std::string imu_frame_id_;
  std::string gnss_frame_id_;
  std::string odom_frame_id_;

  size_t imu_count_{0};
  size_t gnss_count_{0};
  size_t odom_count_{0};
  size_t skipped_count_{0};
  size_t out_of_order_count_{0};

  bool has_first_time_{false};
  double first_time_{0.0};
  double last_time_{0.0};
};
