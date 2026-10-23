#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <sad_common/utm_convert.hpp>

#include "ch4_ros/gins_ros.hpp"


GinsRos::GinsRos()
: Node("gins_ros_node")
{
  const std::string imu_topic = declare_parameter<std::string>("imu_topic", "/sad/imu");
  const std::string gnss_topic = declare_parameter<std::string>("gnss_topic", "/sad/gnss");
  const std::string odom_topic = declare_parameter<std::string>("odom_topic", "/sad/odom");

  map_frame_ = declare_parameter<std::string>("map_frame", map_frame_);
  base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);

  antenna_angle_ = declare_parameter<double>("antenna_angle", antenna_angle_);
  antenna_pos_.x() = declare_parameter<double>("antenna_pos_x", antenna_pos_.x());
  antenna_pos_.y() = declare_parameter<double>("antenna_pos_y", antenna_pos_.y());

  // Print the g2o optimization and the edge errors after every GNSS reading (book --debug)
  GinsPreInteg::Options gins_options;
  gins_options.verbose_ = declare_parameter<bool>("verbose", false);
  gins_ = std::make_unique<GinsPreInteg>(gins_options);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("gins/odometry", 10);

  // Reliable, matching the bag. Deep queues so nothing is dropped at high playback rates.
  const auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(1000)).reliable();
  const auto low_rate_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, imu_qos, std::bind(&GinsRos::imu_callback, this, std::placeholders::_1));
  gnss_sub_ = create_subscription<sad_msgs::msg::Gnss>(
    gnss_topic, low_rate_qos, std::bind(&GinsRos::gnss_callback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<sad_msgs::msg::WheelPulse>(
    odom_topic, low_rate_qos, std::bind(&GinsRos::odom_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "Waiting for data on %s, %s, %s",
    imu_topic.c_str(), gnss_topic.c_str(), odom_topic.c_str());
}

void GinsRos::imu_callback(const sensor_msgs::msg::Imu & msg)
{
  const IMU imu(
    rclcpp::Time(msg.header.stamp).seconds(),
    Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
    Eigen::Vector3d(
      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z));

  // 1. Static initialization: needs the vehicle standing still
  if (!imu_init_.InitSuccess()) {
    imu_init_.AddIMU(imu);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for the static IMU initialization ...");
    return;
  }

  // 2. First reading after the initialization: configure the GINS
  if (!imu_inited_) {
    GinsPreInteg::Options options;
    options.preinteg_options_.init_bg_ = imu_init_.GetInitBg();
    options.preinteg_options_.init_ba_ = imu_init_.GetInitBa();
    options.gravity_ = imu_init_.GetGravity();
    // NOTE: as in the book, SetOptions() does not use its argument (to be fixed later)
    gins_->SetOptions(options);
    imu_inited_ = true;

    const Eigen::Vector3d & bg = options.preinteg_options_.init_bg_;
    const Eigen::Vector3d & ba = options.preinteg_options_.init_ba_;
    const Eigen::Vector3d & gravity = options.gravity_;
    RCLCPP_INFO(
      get_logger(),
      "IMU initialized: bg = [%.6f, %.6f, %.6f], ba = [%.6f, %.6f, %.6f], "
      "gravity = [%.4f, %.4f, %.4f]",
      bg.x(), bg.y(), bg.z(), ba.x(), ba.y(), ba.z(), gravity.x(), gravity.y(), gravity.z());
    return;
  }

  // 3. Wait for the first valid RTK reading
  if (!gnss_inited_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for a GNSS reading with valid heading ...");
    return;
  }

  // 4. Accumulate into the preintegration, then publish the predicted state
  gins_->AddImu(imu);
  publish_state(msg.header.stamp);
}

void GinsRos::gnss_callback(const sad_msgs::msg::Gnss & msg)
{
  if (!imu_inited_) {
    return;
  }

  // Only readings with a valid heading are used (as in the book)
  GNSS gnss(msg);
  if (!ConvertGps2UTM(gnss, antenna_pos_, antenna_angle_) || !gnss.heading_valid_) {
    return;
  }

  // The first valid reading is the local origin
  if (!first_gnss_set_) {
    origin_ = gnss.utm_pose_.translation();
    first_gnss_set_ = true;
  }
  gnss.utm_pose_.translation() -= origin_;

  gins_->AddGnss(gnss);

  if (!gnss_inited_) {
    gnss_inited_ = true;
    RCLCPP_INFO(get_logger(), "First valid GNSS reading received, GINS started.");
  }

  publish_state(msg.header.stamp);
}

void GinsRos::odom_callback(const sad_msgs::msg::WheelPulse & msg)
{
  const Odom odom(rclcpp::Time(msg.header.stamp).seconds(), msg.left_pulse, msg.right_pulse);

  // The static initializer needs the wheel speed to know the vehicle stands still
  imu_init_.AddOdom(odom);

  if (imu_inited_ && gnss_inited_) {
    gins_->AddOdom(odom);
  }
}

void GinsRos::publish_state(const builtin_interfaces::msg::Time & stamp)
{
  const NavState state = gins_->GetState();
  const Eigen::Quaterniond quat = state.R_.unit_quaternion();

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = map_frame_;
  tf_msg.child_frame_id = base_frame_;
  tf_msg.transform.translation = tf2::toMsg2(state.p_);
  tf_msg.transform.rotation = tf2::toMsg(quat);
  tf_broadcaster_->sendTransform(tf_msg);

  // nav_msgs/Odometry: pose in header.frame_id, twist in child_frame_id (body frame)
  const Eigen::Vector3d v_body = state.R_.inverse() * state.v_;

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = map_frame_;
  odom_msg.child_frame_id = base_frame_;
  odom_msg.pose.pose.position = tf2::toMsg(state.p_);
  odom_msg.pose.pose.orientation = tf2::toMsg(quat);
  odom_msg.twist.twist.linear = tf2::toMsg2(v_body);
  odom_pub_->publish(odom_msg);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "t = %.3f, p = [%.3f, %.3f, %.3f], v = [%.3f, %.3f, %.3f], "
    "bg = [%.6f, %.6f, %.6f], ba = [%.6f, %.6f, %.6f]",
    state.timestamp_, state.p_.x(), state.p_.y(), state.p_.z(),
    state.v_.x(), state.v_.y(), state.v_.z(),
    state.bg_.x(), state.bg_.y(), state.bg_.z(),
    state.ba_.x(), state.ba_.y(), state.ba_.z());
}
