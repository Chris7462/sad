#include <chrono>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <manif/SE3.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::chrono_literals;

class TimerPublisher : public rclcpp::Node
{
public:
  TimerPublisher()
  : Node("timer_publisher")
  {
    xi_ << v_body_, omega_;

    timer_ = this->create_wall_timer(
      50ms, std::bind(&TimerPublisher::timer_callback, this));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  void timer_callback()
  {
    pose_ = pose_.rplus(xi_ * dt_);
    RCLCPP_INFO_STREAM(this->get_logger(), "SE3 Pose: " << pose_.translation().transpose());

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.child_frame_id = "se3_link";
    msg.transform.translation = tf2::toMsg2(pose_.translation());
    msg.transform.rotation = tf2::toMsg(pose_.quat());
    tf_broadcaster_->sendTransform(msg);

    Eigen::Vector3d v_world = rotation_.rotation() * v_body_;
    translation_ += v_world * dt_;
    rotation_ = rotation_.rplus(manif::SO3d::Tangent(omega_ * dt_));
    RCLCPP_INFO_STREAM(this->get_logger(), "Rp+t Pose: " << translation_.transpose());

    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.child_frame_id = "rpt_link";
    msg.transform.translation = tf2::toMsg2(translation_);
    msg.transform.rotation = tf2::toMsg(rotation_.quat());
    tf_broadcaster_->sendTransform(msg);
  }

  double angular_velocity_ = 10.0 * M_PI / 180.0;
  double linear_velocity_ = 5.0;
  double dt_ = 0.05;

  manif::SE3d pose_ = manif::SE3d::Identity();

  manif::SO3d rotation_ = manif::SO3d::Identity();
  Eigen::Vector3d translation_ = Eigen::Vector3d::Zero();

  Eigen::Vector3d v_body_ = Eigen::Vector3d(linear_velocity_, 0.0, 0.0);
  Eigen::Vector3d omega_ = Eigen::Vector3d(0.0, 0.0, angular_velocity_);

  // SE(3) tangent vector / twist: manif uses the same (v, w) ordering as Sophus
  manif::SE3d::Tangent xi_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimerPublisher>());
  rclcpp::shutdown();
  return 0;
}
