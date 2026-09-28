#include <string>
#include <fstream>
#include <sstream>

#include <tf2_eigen/tf2_eigen.hpp>

#include "ch3_ros/imu_ros.hpp"


using namespace std::chrono_literals;

ImuReplay::ImuReplay()
: Node("imu_replay_node"), imu_integ_(gravity_, init_bg_, init_ba_)
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
    if (type != "IMU") {
      continue; // Skip ODOM / GNSS
    }

    double t, gx, gy, gz, ax, ay, az;
    if (!(iss >> t >> gx >> gy >> gz >> ax >> ay >> az)) {
      RCLCPP_WARN(get_logger(), "Malformed IMU line %zu, skipped", line_no);
      continue;
    }
    imu_data_.emplace_back(t, Eigen::Vector3d(gx, gy, gz), Eigen::Vector3d(ax, ay, az));
  }

  RCLCPP_INFO(get_logger(), "Loaded %zu IMU samples from %s",
    imu_data_.size(), path.c_str());

  timer_ = create_wall_timer(
    10ms, std::bind(&ImuReplay::timer_callback, this));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

}

void ImuReplay::timer_callback()
{
  if (index_ >= imu_data_.size()) {
    RCLCPP_INFO(get_logger(), "All IMU data has been published.");
    timer_->cancel();
    return;
  }

  const IMU & imu = imu_data_[index_++];
  imu_integ_.AddIMU(imu);

  RCLCPP_INFO(get_logger(),
    "Publishing IMU data at time %.3f: Gyro: [%.3f, %.3f, %.3f], Accel: [%.3f, %.3f, %.3f]",
    imu.timestamp_, imu.gyro_.x(), imu.gyro_.y(), imu.gyro_.z(),
    imu.acce_.x(), imu.acce_.y(), imu.acce_.z());

  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.child_frame_id = "imu_link";
  msg.transform.translation = tf2::toMsg2(imu_integ_.GetP());
  msg.transform.rotation = tf2::toMsg(imu_integ_.GetR().quat());
  tf_broadcaster_->sendTransform(msg);
}
