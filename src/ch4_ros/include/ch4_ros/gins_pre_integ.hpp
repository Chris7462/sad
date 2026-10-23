#pragma once

#include <cmath>
#include <memory>

#include <Eigen/Core>

#include <sad_common/gnss.hpp>
#include <sad_common/imu.hpp>
#include <sad_common/imu_preintegration.hpp>
#include <sad_common/nav_state.hpp>
#include <sad_common/odom.hpp>

#include "ch4_ros/g2o_types.hpp"

/**
 * GINS optimized with IMU preintegration (book chapter 4).
 *
 * This chapter still uses a two-frame model and does not run a full batch optimization
 * (although it could). The two-frame model is closer to the ESKF.
 * IMU readings are accumulated in the preintegrator.
 * Every RTK reading triggers one optimization and a marginalization; the marginalization result
 * is written into the prior of the next frame.
 * For the wheel odometry, the latest reading is used as the velocity observation.
 *
 * Ported as-is from the book; only the changes needed to build were made:
 *  - std::cout instead of glog
 *  - std::make_unique instead of g2o::make_unique (removed from g2o)
 *  - NavState / Sophus::SE3d instead of the book's NavStated / GetSE3()
 *  - M_PI / 180 instead of math::kDEG2RAD
 */
class GinsPreInteg
{
public:
  using Vec3d = Eigen::Vector3d;
  using Mat3d = Eigen::Matrix3d;
  using Mat6d = Eigen::Matrix<double, 6, 6>;

  /// GINS options
  struct Options
  {
    Options() {}

    Vec3d gravity_ = Vec3d(0, 0, -9.8);   // gravity

    /// The IMU noise parameters are set inside the preintegration options
    IMUPreintegration::Options preinteg_options_;

    // Noise
    double bias_gyro_var_ = 1e-6;           // gyro bias random walk standard deviation
    double bias_acce_var_ = 1e-4;           // accelerometer bias random walk standard deviation
    Mat3d bg_rw_info_ = Mat3d::Identity();  // gyro bias random walk information matrix
    Mat3d ba_rw_info_ = Mat3d::Identity();  // accelerometer bias random walk information matrix

    double gnss_pos_noise_ = 0.1;                    // GNSS position noise
    double gnss_height_noise_ = 0.1;                 // GNSS height noise
    double gnss_ang_noise_ = 1.0 * M_PI / 180.0;     // GNSS angle noise
    Mat6d gnss_info_ = Mat6d::Identity();            // 6D GNSS information matrix

    /// Wheel odometry
    double odom_var_ = 0.05;
    Mat3d odom_info_ = Mat3d::Identity();
    double odom_span_ = 0.1;        // odometry sampling period
    double wheel_radius_ = 0.155;   // wheel radius
    double circle_pulse_ = 1024.0;  // encoder pulses per revolution

    bool verbose_ = true;  // print debug information
  };

  /// Options can be given at construction or set later
  explicit GinsPreInteg(Options options = Options())
  : options_(options) {SetOptions(options_);}

  /**
   * IMU processing. The initial biases must have been set before calling this.
   * @param imu IMU reading
   */
  void AddImu(const IMU & imu);

  /**
   * GNSS processing
   * @param gnss
   */
  void AddGnss(const GNSS & gnss);

  /**
   * Wheel odometry processing
   * @param odom
   */
  void AddOdom(const Odom & odom);

  /// Set the options. Can be called at construction or after the static initialization.
  void SetOptions(Options options);

  /**
   * Current state.
   * Without IMU integration, the last optimized state is returned;
   * otherwise the state is predicted with the IMU integration.
   */
  NavState GetState() const;

private:
  // Optimization
  void Optimize();

  Options options_;
  double current_time_ = 0.0;  // current time

  std::shared_ptr<IMUPreintegration> pre_integ_ = nullptr;
  std::shared_ptr<NavState> last_frame_ = nullptr;  // previous state
  std::shared_ptr<NavState> this_frame_ = nullptr;  // current state
  Mat15d prior_info_ = Mat15d::Identity() * 1e2;    // prior of the current state

  /// GNSS observations of the two frames
  GNSS last_gnss_;
  GNSS this_gnss_;

  IMU last_imu_;    // previous IMU reading
  Odom last_odom_;  // previous odometry reading
  bool last_odom_set_ = false;

  /// Flags
  bool first_gnss_received_ = false;  // first GNSS reading received
  bool first_imu_received_ = false;   // first IMU reading received
};
