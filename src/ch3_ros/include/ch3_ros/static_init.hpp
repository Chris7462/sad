#pragma once

#include <cstddef>
#include <deque>

#include <Eigen/Core>

#include "ch3_ros/gnss.hpp"
#include "ch3_ros/imu.hpp"
#include "ch3_ros/odom.hpp"

/**
 * Static initializer (book section 3.5.4), extended with a GNSS noise estimate.
 *
 * Feed it with AddIMU / AddOdom / AddGNSS. While the wheel odometry says the vehicle is standing
 * still, it collects IMU and GNSS readings; after init_time_seconds_ it estimates the gyro /
 * accelerometer biases, the IMU measurement noise and the gravity vector. Check InitSuccess(),
 * then use the getters.
 *
 * The GNSS readings of the same standstill give the noise of the receiver: the standard
 * deviation of the raw antenna position (no antenna extrinsics, no local origin) and of the
 * dual-antenna heading, plus the lag-1 autocorrelation of each (about 0 for white noise, about 1
 * for an error that drifts slowly, in which case the standard deviation is only a lower bound).
 * This part is optional: the initialization succeeds without GNSS, check GnssNoiseValid().
 */
class StaticInit
{
public:
  struct Options
  {
    double init_time_seconds_ = 10.0;         // how long the vehicle must stand still
    size_t init_imu_queue_max_size_ = 2000;   // max IMU samples kept for initialization
    size_t init_gnss_queue_max_size_ = 2000;  // max GNSS samples kept for the noise estimate
    double static_odom_pulse_ = 5.0;          // |pulse| below this counts as standing still
    double max_static_gyro_var_ = 0.5;        // max gyro variance accepted as static
    double max_static_acce_var_ = 0.05;       // max accelerometer variance accepted as static
    double gravity_norm_ = 9.81;
    bool use_speed_for_static_checking_ = true;   // false: assume static at start (no odom)
  };

  StaticInit() = default;
  explicit StaticInit(const Options & options)
  : options_(options) {}

  bool AddIMU(const IMU & imu);
  bool AddOdom(const Odom & odom);
  bool AddGNSS(const GNSS & gnss);

  bool InitSuccess() const {return init_success_;}

  Eigen::Vector3d GetCovGyro() const {return cov_gyro_;}
  Eigen::Vector3d GetCovAcce() const {return cov_acce_;}
  Eigen::Vector3d GetInitBg() const {return init_bg_;}
  Eigen::Vector3d GetInitBa() const {return init_ba_;}
  Eigen::Vector3d GetGravity() const {return gravity_;}

  /// True if enough GNSS readings were collected at standstill for the getters below
  bool GnssNoiseValid() const {return gnss_noise_valid_;}

  size_t GetGnssNumSamples() const {return gnss_num_samples_;}                  // positions used
  size_t GetGnssNumHeadingSamples() const {return gnss_num_heading_samples_;}   // valid headings
  double GetGnssDuration() const {return gnss_duration_;}                       // [s]

  /// Standard deviation / lag-1 autocorrelation of the antenna position: east, north, height [m]
  Eigen::Vector3d GetGnssPositionStd() const {return gnss_position_std_;}
  Eigen::Vector3d GetGnssPositionLag1() const {return gnss_position_lag1_;}

  /// RMS of the east and north standard deviations [m]
  double GetGnssHorizontalStd() const;

  /// Standard deviation [deg] / lag-1 autocorrelation of the heading
  double GetGnssHeadingStd() const {return gnss_heading_std_;}
  double GetGnssHeadingLag1() const {return gnss_heading_lag1_;}

private:
  bool TryInit();
  void EstimateGnssNoise();

  Options options_;
  bool init_success_ = false;
  Eigen::Vector3d cov_gyro_ = Eigen::Vector3d::Zero();   // gyro noise variance (per axis)
  Eigen::Vector3d cov_acce_ = Eigen::Vector3d::Zero();   // accelerometer noise variance (per axis)
  Eigen::Vector3d init_bg_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d init_ba_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity_ = Eigen::Vector3d::Zero();
  bool is_static_ = false;
  std::deque<IMU> init_imu_deque_;
  double current_time_ = 0.0;
  double init_start_time_ = 0.0;

  std::deque<GNSS> init_gnss_deque_;   // raw antenna position in utm_ (no extrinsics, no origin)
  bool gnss_noise_valid_ = false;
  size_t gnss_num_samples_ = 0;
  size_t gnss_num_heading_samples_ = 0;
  double gnss_duration_ = 0.0;
  Eigen::Vector3d gnss_position_std_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gnss_position_lag1_ = Eigen::Vector3d::Zero();
  double gnss_heading_std_ = 0.0;
  double gnss_heading_lag1_ = 0.0;
};
