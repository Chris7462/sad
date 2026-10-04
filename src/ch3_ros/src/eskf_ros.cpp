#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "ch3_ros/eskf_ros.hpp"
#include "ch3_ros/utm_convert.hpp"


EskfRos::EskfRos()
: Node("eskf_ros_node")
{
  const std::string imu_topic = declare_parameter<std::string>("imu_topic", "/sad/imu");
  const std::string gnss_topic = declare_parameter<std::string>("gnss_topic", "/sad/gnss");
  const std::string odom_topic = declare_parameter<std::string>("odom_topic", "/sad/odom");

  with_odom_ = declare_parameter<bool>("with_odom", with_odom_);
  map_frame_ = declare_parameter<std::string>("map_frame", map_frame_);
  base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);

  antenna_angle_ = declare_parameter<double>("antenna_angle", antenna_angle_);
  antenna_pos_.x() = declare_parameter<double>("antenna_pos_x", antenna_pos_.x());
  antenna_pos_.y() = declare_parameter<double>("antenna_pos_y", antenna_pos_.y());

  // ESKF noise options (standard deviations). The gyro / accelerometer measurement noise is
  // not a parameter: it is estimated by the static IMU initializer.
  eskf_options_.bias_gyro_std_ =
    declare_parameter<double>("bias_gyro_std", eskf_options_.bias_gyro_std_);
  eskf_options_.bias_acce_std_ =
    declare_parameter<double>("bias_acce_std", eskf_options_.bias_acce_std_);
  eskf_options_.gnss_pos_std_ =
    declare_parameter<double>("gnss_pos_std", eskf_options_.gnss_pos_std_);
  eskf_options_.gnss_height_std_ =
    declare_parameter<double>("gnss_height_std", eskf_options_.gnss_height_std_);
  eskf_options_.gnss_ang_std_ =
    declare_parameter<double>("gnss_ang_std_deg", 1.0) * M_PI / 180.0;
  eskf_options_.odom_std_ = declare_parameter<double>("odom_std", eskf_options_.odom_std_);
  eskf_options_.odom_span_ = declare_parameter<double>("odom_span", eskf_options_.odom_span_);
  eskf_options_.wheel_radius_ =
    declare_parameter<double>("wheel_radius", eskf_options_.wheel_radius_);
  eskf_options_.circle_pulse_ =
    declare_parameter<double>("circle_pulse", eskf_options_.circle_pulse_);
  eskf_options_.update_bias_gyro_ =
    declare_parameter<bool>("update_bias_gyro", eskf_options_.update_bias_gyro_);
  eskf_options_.update_bias_acce_ =
    declare_parameter<bool>("update_bias_acce", eskf_options_.update_bias_acce_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("eskf/odometry", 10);

  // Reliable, matching the bag. Deep queues so nothing is dropped at high playback rates.
  const auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(1000)).reliable();
  const auto low_rate_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, imu_qos, std::bind(&EskfRos::imu_callback, this, std::placeholders::_1));
  gnss_sub_ = create_subscription<sad_msgs::msg::Gnss>(
    gnss_topic, low_rate_qos, std::bind(&EskfRos::gnss_callback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<sad_msgs::msg::WheelPulse>(
    odom_topic, low_rate_qos, std::bind(&EskfRos::odom_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "Waiting for data on %s, %s, %s (wheel speed update: %s)",
    imu_topic.c_str(), gnss_topic.c_str(), odom_topic.c_str(), with_odom_ ? "on" : "off");
}

void EskfRos::imu_callback(const sensor_msgs::msg::Imu & msg)
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

  // 2. First reading after the initialization: configure the ESKF
  if (!imu_inited_) {
    // Measurement noise estimated by the initializer (variance -> standard deviation)
    eskf_options_.gyro_std_ = std::sqrt(imu_init_.GetCovGyro()[0]);
    eskf_options_.acce_std_ = std::sqrt(imu_init_.GetCovAcce()[0]);

    const Eigen::Vector3d bg = imu_init_.GetInitBg();
    const Eigen::Vector3d ba = imu_init_.GetInitBa();
    const Eigen::Vector3d gravity = imu_init_.GetGravity();
    eskf_.SetInitialConditions(eskf_options_, bg, ba, gravity);
    imu_inited_ = true;

    RCLCPP_INFO(
      get_logger(),
      "IMU initialized: bg = [%.6f, %.6f, %.6f], ba = [%.6f, %.6f, %.6f], "
      "gravity = [%.4f, %.4f, %.4f], gyro std = %.6f, acce std = %.6f",
      bg.x(), bg.y(), bg.z(), ba.x(), ba.y(), ba.z(),
      gravity.x(), gravity.y(), gravity.z(), eskf_options_.gyro_std_, eskf_options_.acce_std_);
    return;
  }

  // 3. Wait for the first valid RTK reading, which sets the initial pose
  if (!gnss_inited_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for a GNSS reading with valid heading ...");
    return;
  }

  // 4. Propagate
  const double dt = imu.timestamp_ - eskf_.GetCurrentTime();
  if (!eskf_.Predict(imu)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "IMU reading skipped, dt = %.4f s", dt);
    return;
  }

  publish_state(msg.header.stamp);
}

void EskfRos::gnss_callback(const sad_msgs::msg::Gnss & msg)
{
  if (!imu_inited_) {
    return;
  }

  // Only RTK readings with valid heading are fused
  GNSS gnss(msg);
  if (!ConvertGps2UTM(gnss, antenna_pos_, antenna_angle_) || !gnss.heading_valid_) {
    return;
  }

  // The first valid reading is the local origin
  if (!first_gnss_set_) {
    origin_ = gnss.utm_pose_.translation();
    first_gnss_set_ = true;
  }
  gnss.utm_pose_.translation(gnss.utm_pose_.translation() - origin_);

  if (!eskf_.ObserveGps(gnss)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "GNSS reading skipped: %.4f s older than the filter time",
      eskf_.GetCurrentTime() - gnss.unix_time_);
    return;
  }

  if (!gnss_inited_) {
    gnss_inited_ = true;
    RCLCPP_INFO(get_logger(), "First valid GNSS reading received, ESKF started.");
  }

  publish_state(msg.header.stamp);
}

void EskfRos::odom_callback(const sad_msgs::msg::WheelPulse & msg)
{
  const Odom odom(rclcpp::Time(msg.header.stamp).seconds(), msg.left_pulse, msg.right_pulse);

  // The static initializer always needs the wheel speed to know the vehicle stands still
  imu_init_.AddOdom(odom);

  if (with_odom_ && imu_inited_ && gnss_inited_) {
    if (!eskf_.ObserveWheelSpeed(odom)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Wheel speed reading skipped: %.4f s older than the filter time",
        eskf_.GetCurrentTime() - odom.timestamp_);
    }
  }
}

void EskfRos::publish_state(const builtin_interfaces::msg::Time & stamp)
{
  const NavState state = eskf_.GetNominalState();
  const Eigen::Quaterniond quat = state.R_.quat();

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = map_frame_;
  tf_msg.child_frame_id = base_frame_;
  tf_msg.transform.translation = tf2::toMsg2(state.p_);
  tf_msg.transform.rotation = tf2::toMsg(quat);
  tf_broadcaster_->sendTransform(tf_msg);

  // nav_msgs/Odometry: pose in header.frame_id, twist in child_frame_id (body frame)
  const Eigen::Vector3d v_body = state.R_.inverse().act(state.v_);

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
