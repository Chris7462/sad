#pragma once

#include <vector>
#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "ch3_ros/gnss.hpp"


class GnssReplay: public rclcpp::Node
{
public:
  GnssReplay();

private:
  void timer_callback();

  double antenna_angle_ = 12.06;
  double antenna_pox_x_ = -0.17;
  double antenna_pox_y_ = -0.20;

  Eigen::Vector2d antenna_pos_ = Eigen::Vector2d(antenna_pox_x_, antenna_pox_y_);

  std::vector<GNSS> gnss_data_;

  bool first_gnss_set_ = false;
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();

  size_t index_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
