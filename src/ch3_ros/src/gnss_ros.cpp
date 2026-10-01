#include <string>
#include <fstream>
#include <sstream>

#include <tf2_eigen/tf2_eigen.hpp>

#include "ch3_ros/gnss_ros.hpp"
#include "ch3_ros/utm_convert.hpp"


using namespace std::chrono_literals;

GnssReplay::GnssReplay()
: Node("gnss_replay_node")
{
  const std::string path = declare_parameter<std::string>(
    "path", "/home/yi-chen/Research/slam_in_autonomous_driving/data/ch3/10.txt");

  std::ifstream file(path);
  if (!file.is_open()) {
    RCLCPP_FATAL(get_logger(), "Cannot open file: %s", path.c_str());
    throw std::runtime_error("Cannot open file: " + path);
  }

  std::string line;
  size_t line_no = 0;
  while (std::getline(file, line)) {
    ++line_no;
    std::istringstream iss(line);
    std::string type;
    if (!(iss >> type)) {
      continue; // Skip empty lines
    }
    if (type != "GNSS") {
      continue; // Skip IMU / ODOM
    }

    double time, lat, lon, alt, heading;
    bool heading_valid;
    if (!(iss >> time >> lat >> lon >> alt >> heading >> heading_valid)) {
      RCLCPP_WARN(get_logger(), "Malformed GNSS line %zu, skipped", line_no);
      continue;
    }
    gnss_data_.emplace_back(time, 4, Eigen::Vector3d(lat, lon, alt), heading, heading_valid);
  }

  RCLCPP_INFO(get_logger(), "Loaded %zu GNSS samples from %s",
    gnss_data_.size(), path.c_str());

  timer_ = create_wall_timer(
    1ms, std::bind(&GnssReplay::timer_callback, this));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

}

void GnssReplay::timer_callback()
{
  if (index_ >= gnss_data_.size()) {
    RCLCPP_INFO(get_logger(), "All GNSS data has been published.");
    timer_->cancel();
    return;
  }

  GNSS gnss_out = gnss_data_[index_++];
  if (ConvertGps2UTM(gnss_out, antenna_pos_, antenna_angle_)) {
    if (!first_gnss_set_) {
        origin = gnss_out.utm_pose_.translation();
        first_gnss_set_ = true;
    }

    //gnss_out.utm_pose_.translation() -= origin;
    //auto & trans = gnss_out.utm_pose_.translation();
    //auto & quat = gnss_out.utm_pose_.unit_quaternion();

    gnss_out.utm_pose_.translation(gnss_out.utm_pose_.translation() - origin);

    const Eigen::Vector3d trans = gnss_out.utm_pose_.translation();
    const Eigen::Quaterniond quat = gnss_out.utm_pose_.quat();

    RCLCPP_INFO(get_logger(),
      "Publishing GNSS data at time %.3f: Quat: [%.3f, %.3f, %.3f, %.3f], tran: [%.3f, %.3f, %.3f]",
      gnss_out.unix_time_, quat.w(), quat.x(), quat.y(), quat.z(),
      trans.x(), trans.y(), trans.z());

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.child_frame_id = "gnss_link";
    msg.transform.translation = tf2::toMsg2(trans);
    msg.transform.rotation = tf2::toMsg(quat);
    tf_broadcaster_->sendTransform(msg);
  }
}
