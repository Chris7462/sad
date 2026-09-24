#include <chrono>

#include <std_msgs/msg/string.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "ch2_ros/motion.hpp"


using namespace std::chrono_literals;

TimerPublisher::TimerPublisher()
: Node("timer_publisher")
{
  xi_ << v_body_, omega_;

  timer_ = this->create_wall_timer(
    50ms, std::bind(&TimerPublisher::timer_callback, this));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void TimerPublisher::timer_callback()
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
