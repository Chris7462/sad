#pragma once

#include <memory>
#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sad_msgs/msg/gnss.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "ch3_ros/gnss.hpp"


class GnssRos : public rclcpp::Node
{
public:
  GnssRos();

private:
  void gnss_callback(const sad_msgs::msg::Gnss & msg);

  // RTK antenna extrinsics (set from parameters)
  double antenna_angle_ = 12.06;  // [deg]
  Eigen::Vector2d antenna_pos_ = Eigen::Vector2d(-0.17, -0.20);  // [m]

  bool first_gnss_set_ = false;
  Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();

  rclcpp::Subscription<sad_msgs::msg::Gnss>::SharedPtr gnss_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
