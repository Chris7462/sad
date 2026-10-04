#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "ch3_ros/gnss_ros.hpp"
#include "ch3_ros/utm_convert.hpp"


GnssRos::GnssRos()
: Node("gnss_ros_node")
{
  const std::string gnss_topic = declare_parameter<std::string>("gnss_topic", "/sad/gnss");
  antenna_angle_ = declare_parameter<double>("antenna_angle", antenna_angle_);
  antenna_pos_.x() = declare_parameter<double>("antenna_pos_x", antenna_pos_.x());
  antenna_pos_.y() = declare_parameter<double>("antenna_pos_y", antenna_pos_.y());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Match the QoS recorded in the bag: reliable.
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
  gnss_sub_ = create_subscription<sad_msgs::msg::Gnss>(
    gnss_topic, qos, std::bind(&GnssRos::gnss_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Waiting for GNSS data on %s", gnss_topic.c_str());
}

void GnssRos::gnss_callback(const sad_msgs::msg::Gnss & msg)
{
  GNSS gnss_out(msg);
  if (!ConvertGps2UTM(gnss_out, antenna_pos_, antenna_angle_)) {
    return;
  }

  // Use the first reading as the local origin
  if (!first_gnss_set_) {
    origin_ = gnss_out.utm_pose_.translation();
    first_gnss_set_ = true;
  }

  //gnss_out.utm_pose_.translation() -= origin;
  //auto & trans = gnss_out.utm_pose_.translation();
  //auto & quat = gnss_out.utm_pose_.unit_quaternion();

  gnss_out.utm_pose_.translation(gnss_out.utm_pose_.translation() - origin_);

  const Eigen::Vector3d trans = gnss_out.utm_pose_.translation();
  const Eigen::Quaterniond quat = gnss_out.utm_pose_.quat();

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "GNSS at time %.3f: Quat: [%.3f, %.3f, %.3f, %.3f], tran: [%.3f, %.3f, %.3f]",
    gnss_out.unix_time_, quat.w(), quat.x(), quat.y(), quat.z(),
    trans.x(), trans.y(), trans.z());

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = msg.header.stamp;  // data time, not now()
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "gnss_link";
  tf_msg.transform.translation = tf2::toMsg2(trans);
  tf_msg.transform.rotation = tf2::toMsg(quat);
  tf_broadcaster_->sendTransform(tf_msg);
}
