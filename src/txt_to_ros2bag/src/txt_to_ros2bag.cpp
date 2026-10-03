// C++ standard library
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>

// ROS 2
#include <sensor_msgs/msg/imu.hpp>

// local
#include "sad_msgs/msg/gnss.hpp"
#include "sad_msgs/msg/wheel_pulse.hpp"
#include "txt_to_ros2bag/txt_to_ros2bag.hpp"

namespace fs = std::filesystem;


Txt2BagNode::Txt2BagNode()
: Node("txt_to_ros2bag_node")
{
  txt_path_ = declare_parameter("txt_path", std::string());
  output_bag_ = declare_parameter("output_bag", std::string("sad_ch3_bag"));

  imu_topic_ = declare_parameter("imu_topic", std::string("/sad/imu"));
  gnss_topic_ = declare_parameter("gnss_topic", std::string("/sad/gnss"));
  odom_topic_ = declare_parameter("odom_topic", std::string("/sad/odom"));

  imu_frame_id_ = declare_parameter("imu_frame_id", std::string("imu_link"));
  gnss_frame_id_ = declare_parameter("gnss_frame_id", std::string("gnss_link"));
  odom_frame_id_ = declare_parameter("odom_frame_id", std::string("wheel_link"));
}

bool Txt2BagNode::convert()
{
  if (txt_path_.empty() || !fs::is_regular_file(txt_path_)) {
    RCLCPP_ERROR(get_logger(), "txt_path is not a file: '%s'", txt_path_.c_str());
    return false;
  }

  // rosbag2 refuses to open an existing bag directory; fail with a clear message instead.
  if (fs::exists(output_bag_)) {
    RCLCPP_ERROR(
      get_logger(), "Output bag '%s' already exists. Remove it or change output_bag.",
      output_bag_.c_str());
    return false;
  }

  std::ifstream fin(txt_path_);
  if (!fin) {
    RCLCPP_ERROR(get_logger(), "Cannot open '%s'", txt_path_.c_str());
    return false;
  }

  bag_writer_ = std::make_unique<BagWriter>(output_bag_);

  // Reliable on every topic: the ESKF must not lose samples.
  // IMU is 100 Hz, so give it and the wheel pulses a deeper queue.
  const auto high_rate_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile();
  const auto low_rate_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  bag_writer_->create_topic(imu_topic_, "sensor_msgs/msg/Imu", high_rate_qos);
  bag_writer_->create_topic(odom_topic_, "sad_msgs/msg/WheelPulse", high_rate_qos);
  bag_writer_->create_topic(gnss_topic_, "sad_msgs/msg/Gnss", low_rate_qos);

  RCLCPP_INFO(get_logger(), "Converting '%s' -> '%s'", txt_path_.c_str(), output_bag_.c_str());

  std::string line;
  size_t line_no = 0;
  while (std::getline(fin, line)) {
    ++line_no;
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::istringstream ss(line);
    std::string data_type;
    ss >> data_type;

    bool ok = true;
    if (data_type == "IMU") {
      ok = write_imu(ss);
    } else if (data_type == "GNSS") {
      ok = write_gnss(ss);
    } else if (data_type == "ODOM") {
      ok = write_odom(ss);
    } else {
      ok = false;
    }

    if (!ok) {
      ++skipped_count_;
      RCLCPP_WARN(get_logger(), "Skipping line %zu: '%s'", line_no, line.c_str());
    }
  }

  // Destroying the writer closes the bag and flushes metadata.yaml.
  bag_writer_.reset();

  RCLCPP_INFO(
    get_logger(), "Done. IMU: %zu, GNSS: %zu, ODOM: %zu, skipped: %zu",
    imu_count_, gnss_count_, odom_count_, skipped_count_);
  if (has_first_time_) {
    RCLCPP_INFO(
      get_logger(), "Time range: %.6f -> %.6f (%.3f s)",
      first_time_, last_time_, last_time_ - first_time_);
  }
  if (out_of_order_count_ > 0) {
    RCLCPP_WARN(
      get_logger(), "%zu line(s) had a timestamp older than the previous line.",
      out_of_order_count_);
  }

  return (imu_count_ + gnss_count_ + odom_count_) > 0;
}

rclcpp::Time Txt2BagNode::to_ros_time(double seconds)
{
  if (!has_first_time_) {
    has_first_time_ = true;
    first_time_ = seconds;
  } else if (seconds < last_time_) {
    ++out_of_order_count_;
  }
  last_time_ = seconds;

  return rclcpp::Time(static_cast<int64_t>(std::llround(seconds * 1e9)), RCL_ROS_TIME);
}

// IMU time gx gy gz ax ay az
bool Txt2BagNode::write_imu(std::istringstream & ss)
{
  double time, gx, gy, gz, ax, ay, az;
  if (!(ss >> time >> gx >> gy >> gz >> ax >> ay >> az)) {
    return false;
  }

  const rclcpp::Time stamp = to_ros_time(time);

  sensor_msgs::msg::Imu msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = imu_frame_id_;
  msg.angular_velocity.x = gx;
  msg.angular_velocity.y = gy;
  msg.angular_velocity.z = gz;
  msg.linear_acceleration.x = ax;
  msg.linear_acceleration.y = ay;
  msg.linear_acceleration.z = az;
  // No orientation estimate in this data: REP 145 says mark it with -1.
  msg.orientation.w = 1.0;
  msg.orientation_covariance[0] = -1.0;

  bag_writer_->write_message(msg, imu_topic_, stamp);
  ++imu_count_;
  return true;
}

// GNSS time lat lon alt heading heading_valid
bool Txt2BagNode::write_gnss(std::istringstream & ss)
{
  double time, lat, lon, alt, heading;
  int heading_valid;
  if (!(ss >> time >> lat >> lon >> alt >> heading >> heading_valid)) {
    return false;
  }

  const rclcpp::Time stamp = to_ros_time(time);

  sad_msgs::msg::Gnss msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = gnss_frame_id_;
  // The txt file carries no status column; the book code hardcodes fixed (4).
  msg.status = sad_msgs::msg::Gnss::STATUS_FIXED;
  msg.latitude = lat;
  msg.longitude = lon;
  msg.altitude = alt;
  msg.heading = heading;
  msg.heading_valid = (heading_valid != 0);

  bag_writer_->write_message(msg, gnss_topic_, stamp);
  ++gnss_count_;
  return true;
}

// ODOM time left_pulse right_pulse
bool Txt2BagNode::write_odom(std::istringstream & ss)
{
  double time, left, right;
  if (!(ss >> time >> left >> right)) {
    return false;
  }

  const rclcpp::Time stamp = to_ros_time(time);

  sad_msgs::msg::WheelPulse msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = odom_frame_id_;
  msg.left_pulse = left;
  msg.right_pulse = right;

  bag_writer_->write_message(msg, odom_topic_, stamp);
  ++odom_count_;
  return true;
}
