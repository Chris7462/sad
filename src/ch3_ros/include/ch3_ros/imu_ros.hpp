#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "ch3_ros/imu_integration.hpp"


class ImuReplay: public rclcpp::Node
{
public:
  ImuReplay();

private:
  void timer_callback();

  Eigen::Vector3d gravity_ = Eigen::Vector3d(0, 0, -9.81);
  Eigen::Vector3d init_bg_ = Eigen::Vector3d(0.000224886, -7.61038e-05, -0.000742259);
  Eigen::Vector3d init_ba_ = Eigen::Vector3d(-0.165205, 0.0926887, 0.0058049);

  std::vector<IMU> imu_data_;
  IMUIntegration imu_integ_;

  size_t index_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
