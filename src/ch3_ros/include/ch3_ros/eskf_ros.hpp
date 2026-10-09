#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sad_msgs/msg/gnss.hpp>
#include <sad_msgs/msg/wheel_pulse.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "ch3_ros/eskf.hpp"
#include "ch3_ros/static_imu_init.hpp"


/**
 * RTK GNSS + IMU (+ optional wheel speed) fusion with the ESKF, the ROS 2 version of
 * run_eskf_gins in the book.
 */
class EskfRos : public rclcpp::Node
{
public:
  EskfRos();

private:
  void imu_callback(const sensor_msgs::msg::Imu & msg);
  void gnss_callback(const sad_msgs::msg::Gnss & msg);
  void odom_callback(const sad_msgs::msg::WheelPulse & msg);

  /// Fuse a GNSS reading with valid heading (6-DoF). The first one starts the filter.
  void fuse_gnss_pose(GNSS & gnss, const builtin_interfaces::msg::Time & stamp);

  /// Fuse only the position of a GNSS reading whose heading is invalid (3-DoF)
  void fuse_gnss_position(GNSS & gnss, const builtin_interfaces::msg::Time & stamp);

  /// Publish the nominal state as tf and nav_msgs/Odometry, stamped with the data time
  void publish_state(const builtin_interfaces::msg::Time & stamp);

  /// Publish the antenna extrinsics as the static tf base_link -> gnss_link, so that
  /// map -> gnss_link is the antenna pose predicted by the filter
  void publish_antenna_extrinsics();

  /// Publish the measured antenna position as tf map -> gnss_raw, for comparison with
  /// map -> gnss_link. Uses only the GNSS reading and the local origin, no filter state.
  void publish_raw_gnss(GNSS gnss, const builtin_interfaces::msg::Time & stamp);

  /// Publish the pure IMU dead reckoning pose as tf map -> imu_dr
  void publish_dead_reckoning(const builtin_interfaces::msg::Time & stamp);

  StaticIMUInit imu_init_;
  ESKF eskf_;

  // Pure IMU dead reckoning for comparison: a second filter that starts from the same initial
  // state as eskf_ and only ever runs Predict(). Without observations its biases and gravity
  // stay at their initial values, so it is plain IMU integration.
  ESKF dr_eskf_;
  bool publish_dead_reckoning_ = true;
  ESKF::Options eskf_options_;

  bool imu_inited_ = false;
  bool gnss_inited_ = false;

  // RTK antenna extrinsics (set from parameters)
  double antenna_angle_ = 12.06;  // [deg]
  Eigen::Vector2d antenna_pos_ = Eigen::Vector2d(-0.17, -0.20);  // [m]

  bool first_gnss_set_ = false;
  Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();

  bool with_odom_ = false;
  std::string map_frame_ = "map";
  std::string base_frame_ = "base_link";
  std::string gnss_frame_ = "gnss_link";      // antenna, rigidly attached to base_link
  std::string gnss_raw_frame_ = "gnss_raw";   // antenna as measured by GNSS
  std::string dr_frame_ = "imu_dr";           // body pose from pure IMU dead reckoning

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sad_msgs::msg::Gnss>::SharedPtr gnss_sub_;
  rclcpp::Subscription<sad_msgs::msg::WheelPulse>::SharedPtr odom_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};
