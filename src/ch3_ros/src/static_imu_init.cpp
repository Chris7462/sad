#include <cmath>

#include <rclcpp/logging.hpp>

#include "ch3_ros/static_imu_init.hpp"


namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("static_imu_init");

// Mean and per-axis (diagonal) variance of getter(item) over a container
template<typename Container, typename Getter>
void ComputeMeanAndCovDiag(
  const Container & data, Eigen::Vector3d & mean, Eigen::Vector3d & cov_diag, Getter && getter)
{
  const double len = static_cast<double>(data.size());

  mean.setZero();
  for (const auto & item : data) {
    mean += getter(item);
  }
  mean /= len;

  cov_diag.setZero();
  for (const auto & item : data) {
    cov_diag += (getter(item) - mean).cwiseAbs2();
  }
  cov_diag /= (len - 1.0);
}
}  // namespace

bool StaticIMUInit::AddIMU(const IMU & imu)
{
  if (init_success_) {
    return true;
  }

  if (options_.use_speed_for_static_checking_ && !is_static_) {
    // Vehicle is moving (or no odom yet): start over
    init_imu_deque_.clear();
    return false;
  }

  if (init_imu_deque_.empty()) {
    init_start_time_ = imu.timestamp_;
  }

  init_imu_deque_.push_back(imu);
  current_time_ = imu.timestamp_;

  const double init_time = imu.timestamp_ - init_start_time_;
  if (init_time > options_.init_time_seconds_) {
    TryInit();
  }

  while (init_imu_deque_.size() > options_.init_imu_queue_max_size_) {
    init_imu_deque_.pop_front();
  }

  return init_success_;
}

bool StaticIMUInit::AddOdom(const Odom & odom)
{
  if (init_success_) {
    return true;
  }

  // abs(): reversing gives negative pulses, which must not count as standing still
  is_static_ =
    std::abs(odom.left_pulse_) < options_.static_odom_pulse_ &&
    std::abs(odom.right_pulse_) < options_.static_odom_pulse_;

  current_time_ = odom.timestamp_;
  return true;
}

bool StaticIMUInit::TryInit()
{
  if (init_imu_deque_.size() < 10) {
    return false;
  }

  Eigen::Vector3d mean_gyro, mean_acce;
  ComputeMeanAndCovDiag(
    init_imu_deque_, mean_gyro, cov_gyro_, [](const IMU & imu) {return imu.gyro_;});
  ComputeMeanAndCovDiag(
    init_imu_deque_, mean_acce, cov_acce_, [](const IMU & imu) {return imu.acce_;});

  // Gravity points opposite to the mean specific force, with the configured norm
  gravity_ = -mean_acce / mean_acce.norm() * options_.gravity_norm_;

  // Recompute the accelerometer statistics with gravity removed
  ComputeMeanAndCovDiag(
    init_imu_deque_, mean_acce, cov_acce_,
    [this](const IMU & imu) -> Eigen::Vector3d {return imu.acce_ + gravity_;});

  if (cov_gyro_.norm() > options_.max_static_gyro_var_) {
    RCLCPP_ERROR(
      kLogger, "Gyro noise too large: %f > %f", cov_gyro_.norm(), options_.max_static_gyro_var_);
    return false;
  }

  if (cov_acce_.norm() > options_.max_static_acce_var_) {
    RCLCPP_ERROR(
      kLogger, "Accelerometer noise too large: %f > %f",
      cov_acce_.norm(), options_.max_static_acce_var_);
    return false;
  }

  init_bg_ = mean_gyro;
  init_ba_ = mean_acce;

  RCLCPP_INFO(
    kLogger, "IMU initialized from %zu samples over %.2f s",
    init_imu_deque_.size(), current_time_ - init_start_time_);

  init_success_ = true;
  return true;
}
