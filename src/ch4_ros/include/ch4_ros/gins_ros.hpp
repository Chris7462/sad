#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sad_msgs/msg/gnss.hpp>
#include <sad_msgs/msg/wheel_pulse.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <sad_common/static_imu_init.hpp>

#include "ch4_ros/gins_pre_integ.hpp"


/**
 * RTK GNSS + IMU + wheel speed fusion with IMU preintegration and g2o, the ROS 2 version of
 * run_gins_pre_integ in the book. The Pangolin UI is replaced by tf and nav_msgs/Odometry.
 */
class GinsRos : public rclcpp::Node
{
public:
  GinsRos();

private:
  void imu_callback(const sensor_msgs::msg::Imu & msg);
  void gnss_callback(const sad_msgs::msg::Gnss & msg);
  void odom_callback(const sad_msgs::msg::WheelPulse & msg);

  /// Publish the current state as tf and nav_msgs/Odometry, stamped with the data time
  void publish_state(const builtin_interfaces::msg::Time & stamp);

  StaticIMUInit imu_init_;
  std::unique_ptr<GinsPreInteg> gins_;

  bool imu_inited_ = false;
  bool gnss_inited_ = false;

  // RTK antenna extrinsics (set from parameters)
  double antenna_angle_ = 12.06;  // [deg]
  Eigen::Vector2d antenna_pos_ = Eigen::Vector2d(-0.17, -0.20);  // [m]

  bool first_gnss_set_ = false;
  Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();

  std::string map_frame_ = "map";
  std::string base_frame_ = "base_link";

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sad_msgs::msg::Gnss>::SharedPtr gnss_sub_;
  rclcpp::Subscription<sad_msgs::msg::WheelPulse>::SharedPtr odom_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
