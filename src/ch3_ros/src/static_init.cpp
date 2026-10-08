#include <cmath>
#include <iterator>
#include <vector>

#include <rclcpp/logging.hpp>

#include "ch3_ros/static_init.hpp"
#include "ch3_ros/utm_convert.hpp"


namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("static_init");

// Mean and per-component (diagonal) variance of getter(item) over a container.
// Vec is a fixed-size Eigen vector, e.g. Eigen::Vector3d.
template<typename Container, typename Vec, typename Getter>
void ComputeMeanAndCovDiag(const Container & data, Vec & mean, Vec & cov_diag, Getter && getter)
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

// Per-component lag-1 autocorrelation of getter(item) over a container, in its stored order.
// `mean` is the one from ComputeMeanAndCovDiag. A component that never changes gets 0.
template<typename Container, typename Vec, typename Getter>
void ComputeLag1Autocorr(const Container & data, const Vec & mean, Vec & lag1, Getter && getter)
{
  Vec sum_sq = Vec::Zero();
  Vec sum_lag = Vec::Zero();

  for (auto it = data.begin(); it != data.end(); ++it) {
    const Vec diff = getter(*it) - mean;
    sum_sq += diff.cwiseAbs2();

    const auto next = std::next(it);
    if (next != data.end()) {
      const Vec next_diff = getter(*next) - mean;
      sum_lag += diff.cwiseProduct(next_diff);
    }
  }

  lag1.setZero();
  for (Eigen::Index i = 0; i < lag1.size(); ++i) {
    if (sum_sq[i] > 0.0) {
      lag1[i] = sum_lag[i] / sum_sq[i];
    }
  }
}
}  // namespace

bool StaticInit::AddIMU(const IMU & imu)
{
  if (init_success_) {
    return true;
  }

  if (options_.use_speed_for_static_checking_ && !is_static_) {
    // Vehicle is moving (or no odom yet): start over
    init_imu_deque_.clear();
    init_gnss_deque_.clear();
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

bool StaticInit::AddOdom(const Odom & odom)
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

bool StaticInit::AddGNSS(const GNSS & gnss)
{
  if (init_success_) {
    return true;
  }

  if (options_.use_speed_for_static_checking_ && !is_static_) {
    // Vehicle is moving (or no odom yet): start over
    init_gnss_deque_.clear();
    return false;
  }

  // Fill utm_ with the raw antenna position: no antenna extrinsics, no local origin
  GNSS reading = gnss;
  if (!ConvertGps2UTMOnlyTrans(reading)) {
    return false;
  }
  init_gnss_deque_.push_back(reading);

  while (init_gnss_deque_.size() > options_.init_gnss_queue_max_size_) {
    init_gnss_deque_.pop_front();
  }

  return false;
}

bool StaticInit::TryInit()
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

  // Optional: never blocks the initialization
  EstimateGnssNoise();

  init_success_ = true;
  return true;
}

void StaticInit::EstimateGnssNoise()
{
  gnss_noise_valid_ = false;

  if (init_gnss_deque_.size() < 10) {
    return;
  }

  // Position (UTM east, north, altitude), relative to the first reading to keep the numbers small
  const auto to_position = [](const GNSS & gnss) -> Eigen::Vector3d {
      return Eigen::Vector3d(gnss.utm_.xy_[0], gnss.utm_.xy_[1], gnss.utm_.z_);
    };
  const Eigen::Vector3d origin = to_position(init_gnss_deque_.front());
  const auto position_getter =
    [&origin, &to_position](const GNSS & gnss) -> Eigen::Vector3d {
      return to_position(gnss) - origin;
    };

  Eigen::Vector3d mean_position, cov_position;
  ComputeMeanAndCovDiag(init_gnss_deque_, mean_position, cov_position, position_getter);
  ComputeLag1Autocorr(init_gnss_deque_, mean_position, gnss_position_lag1_, position_getter);
  gnss_position_std_ = cov_position.cwiseSqrt();

  // Heading, valid readings only. Differences to the first one are wrapped into [-180, 180)
  // so a reading near north does not jump by 360 deg.
  using Vec1d = Eigen::Matrix<double, 1, 1>;
  std::vector<Vec1d> heading;
  double heading_reference = 0.0;
  for (const GNSS & gnss : init_gnss_deque_) {
    if (!gnss.heading_valid_) {
      continue;
    }
    if (heading.empty()) {
      heading_reference = gnss.heading_;
    }

    double diff = std::fmod(gnss.heading_ - heading_reference + 180.0, 360.0);
    if (diff < 0.0) {
      diff += 360.0;
    }
    heading.push_back(Vec1d(diff - 180.0));
  }

  gnss_heading_std_ = 0.0;
  gnss_heading_lag1_ = 0.0;
  if (heading.size() >= 2) {
    Vec1d mean_heading, cov_heading, lag1_heading;
    const auto heading_getter = [](const Vec1d & value) -> Vec1d {return value;};
    ComputeMeanAndCovDiag(heading, mean_heading, cov_heading, heading_getter);
    ComputeLag1Autocorr(heading, mean_heading, lag1_heading, heading_getter);
    gnss_heading_std_ = std::sqrt(cov_heading[0]);
    gnss_heading_lag1_ = lag1_heading[0];
  }

  gnss_num_samples_ = init_gnss_deque_.size();
  gnss_num_heading_samples_ = heading.size();
  gnss_duration_ = init_gnss_deque_.back().unix_time_ - init_gnss_deque_.front().unix_time_;
  gnss_noise_valid_ = true;
}

double StaticInit::GetGnssHorizontalStd() const
{
  return std::sqrt(
    0.5 * (gnss_position_std_[0] * gnss_position_std_[0] +
    gnss_position_std_[1] * gnss_position_std_[1]));
}
