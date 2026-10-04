#pragma once

#include <cmath>

#include <Eigen/Core>
#include <manif/SE3.h>
#include <manif/SO3.h>

#include "ch3_ros/gnss.hpp"
#include "ch3_ros/imu.hpp"
#include "ch3_ros/nav_state.hpp"
#include "ch3_ros/odom.hpp"

/**
 * Error-state Kalman filter of book chapter 3.
 *
 * 18-dimensional error state, ordered as in the book: p, v, theta, bg, ba, g.
 * GNSS readings must already be converted to the body pose in the local frame (utm_pose_).
 *
 * Differences from the book code:
 *  - every noise option is a standard deviation and is squared when the matrices are built
 *  - the GNSS update uses the 6x6 GNSS noise matrix, so the height noise takes effect
 *  - data older than the filter time is rejected (returns false) instead of asserting
 *  - ObservePosition() lets GNSS readings without a valid heading still correct the position
 */
class ESKF
{
public:
  using Vec3d = Eigen::Vector3d;
  using Mat3d = Eigen::Matrix3d;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat6d = Eigen::Matrix<double, 6, 6>;
  using Vec18d = Eigen::Matrix<double, 18, 1>;
  using Mat18d = Eigen::Matrix<double, 18, 18>;

  struct Options
  {
    /// IMU. Noise values are discrete-time standard deviations (per IMU step, not multiplied by dt)
    double imu_dt_ = 0.01;           // nominal IMU period [s]
    double gyro_std_ = 1e-5;         // gyro measurement noise
    double acce_std_ = 1e-2;         // accelerometer measurement noise
    double bias_gyro_std_ = 1e-6;    // gyro bias random walk
    double bias_acce_std_ = 1e-4;    // accelerometer bias random walk

    /// Wheel odometry
    double odom_std_ = 0.5;          // velocity observation noise [m/s]
    double odom_span_ = 0.1;         // sampling period of the pulse count [s]
    double wheel_radius_ = 0.155;    // [m]
    double circle_pulse_ = 1024.0;   // pulses per wheel revolution

    /// RTK GNSS
    double gnss_pos_std_ = 0.1;                    // horizontal position noise [m]
    double gnss_height_std_ = 0.1;                 // height noise [m]
    double gnss_ang_std_ = 1.0 * M_PI / 180.0;     // rotation noise [rad]

    /// Misc
    bool update_bias_gyro_ = true;
    bool update_bias_acce_ = true;
  };

  ESKF() {BuildNoise(options_);}
  explicit ESKF(const Options & options)
  : options_(options) {BuildNoise(options_);}

  /// Set noise options, initial biases and gravity; resets the covariance
  void SetInitialConditions(
    const Options & options, const Vec3d & init_bg, const Vec3d & init_ba,
    const Vec3d & gravity = Vec3d(0.0, 0.0, -9.8));

  /// Propagate with one IMU reading. False if the reading was skipped (bad dt).
  bool Predict(const IMU & imu);

  /// Wheel speed observation. False if the reading is older than the filter time.
  bool ObserveWheelSpeed(const Odom & odom);

  /// GNSS observation. The first one initializes R and p. False if older than the filter time.
  bool ObserveGps(const GNSS & gnss);

  /// Position-only observation (body position in the local frame), using the GNSS position
  /// noise. For GNSS readings without a valid heading. False if the filter has no initial pose
  /// yet or the reading is older than the filter time.
  bool ObservePosition(double timestamp, const Vec3d & position);

  /// True once the first GNSS reading has set the initial pose
  bool PoseInitialized() const {return !first_gnss_;}

  /// Generic pose observation with isotropic noise (standard deviations)
  bool ObserveSE3(
    const manif::SE3d & pose, double trans_std = 0.1, double ang_std = 1.0 * M_PI / 180.0);

  NavState GetNominalState() const {return NavState(current_time_, R_, p_, v_, bg_, ba_);}
  manif::SE3d GetNominalSE3() const {return manif::SE3d(p_, R_);}
  Vec3d GetGravity() const {return g_;}
  Mat18d GetCov() const {return cov_;}
  double GetCurrentTime() const {return current_time_;}

  void SetX(const NavState & x, const Vec3d & gravity);
  void SetCov(const Mat18d & cov) {cov_ = cov;}

private:
  void BuildNoise(const Options & options);

  /// Pose observation with a full 6x6 noise covariance (translation first, then rotation)
  bool ObserveSE3(const manif::SE3d & pose, const Mat6d & noise);

  /// Fold the error state into the nominal state, then reset it
  void UpdateAndReset();

  /// Project the covariance after the reset, eq. (3.63)
  void ProjectCov();

  double current_time_ = 0.0;

  /// Nominal state
  Vec3d p_ = Vec3d::Zero();
  Vec3d v_ = Vec3d::Zero();
  manif::SO3d R_ = manif::SO3d::Identity();
  Vec3d bg_ = Vec3d::Zero();
  Vec3d ba_ = Vec3d::Zero();
  Vec3d g_ = Vec3d(0.0, 0.0, -9.8);

  /// Error state
  Vec18d dx_ = Vec18d::Zero();

  /// Covariance
  Mat18d cov_ = Mat18d::Identity();

  /// Noise
  Mat18d Q_ = Mat18d::Zero();
  Mat3d odom_noise_ = Mat3d::Zero();
  Mat6d gnss_noise_ = Mat6d::Zero();

  bool first_gnss_ = true;

  Options options_;
};
