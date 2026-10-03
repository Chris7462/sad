#pragma once

#include <memory>
#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "ch3_ros/imu_integration.hpp"


class ImuRos : public rclcpp::Node
{
public:
  ImuRos();

private:
  void imu_callback(const sensor_msgs::msg::Imu & msg);

  Eigen::Vector3d gravity_ = Eigen::Vector3d(0, 0, -9.81);
  Eigen::Vector3d init_bg_ = Eigen::Vector3d(0.000224886, -7.61038e-05, -0.000742259);
  Eigen::Vector3d init_ba_ = Eigen::Vector3d(-0.165205, 0.0926887, 0.0058049);

  IMUIntegration imu_integ_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
