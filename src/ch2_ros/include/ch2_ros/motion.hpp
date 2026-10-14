#include <memory>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>


class TimerPublisher : public rclcpp::Node
{
public:
  TimerPublisher();

private:
  void timer_callback();
  double angular_velocity_ = 10.0 * M_PI / 180.0;
  double linear_velocity_ = 5.0;
  double dt_ = 0.05;

  Sophus::SE3d pose_ = Sophus::SE3d();

  Sophus::SO3d rotation_ = Sophus::SO3d();
  Eigen::Vector3d translation_ = Eigen::Vector3d::Zero();

  Eigen::Vector3d v_body_ = Eigen::Vector3d(linear_velocity_, 0.0, 0.0);
  Eigen::Vector3d omega_ = Eigen::Vector3d(0.0, 0.0, angular_velocity_);

  // SE(3) tangent vector / twist, ordered (v, w)
  Sophus::SE3d::Tangent xi_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
