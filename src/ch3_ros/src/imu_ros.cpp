#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "ch3_ros/imu_ros.hpp"


ImuRos::ImuRos()
: Node("imu_ros_node"), imu_integ_(gravity_, init_bg_, init_ba_)
{
  const std::string imu_topic = declare_parameter<std::string>("imu_topic", "/sad/imu");

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Match the QoS recorded in the bag: reliable, deep enough for 100 Hz IMU.
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, qos, std::bind(&ImuRos::imu_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Waiting for IMU data on %s", imu_topic.c_str());
}

void ImuRos::imu_callback(const sensor_msgs::msg::Imu & msg)
{
  const IMU imu(
    rclcpp::Time(msg.header.stamp).seconds(),
    Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
    Eigen::Vector3d(
      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z));

  imu_integ_.AddIMU(imu);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "IMU at time %.3f: Gyro: [%.3f, %.3f, %.3f], Accel: [%.3f, %.3f, %.3f]",
    imu.timestamp_, imu.gyro_.x(), imu.gyro_.y(), imu.gyro_.z(),
    imu.acce_.x(), imu.acce_.y(), imu.acce_.z());

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = msg.header.stamp;  // data time, not now()
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "imu_link";
  tf_msg.transform.translation = tf2::toMsg2(imu_integ_.GetP());
  tf_msg.transform.rotation = tf2::toMsg(imu_integ_.GetR().quat());
  tf_broadcaster_->sendTransform(tf_msg);
}
