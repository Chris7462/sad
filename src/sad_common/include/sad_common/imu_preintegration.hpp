#pragma once

#include <Eigen/Core>
#include <sophus/so3.hpp>

#include "sad_common/imu.hpp"
#include "sad_common/nav_state.hpp"

/**
 * IMU preintegration (book chapter 4).
 *
 * Call Integrate() for every new IMU reading, then read the preintegrated values with the getters.
 * The Jacobians with respect to the biases are also kept here, for building the g2o edges.
 *
 * Differences from the book code:
 *  - noise options are named *_std_ (standard deviations), like the ESKF options
 *  - all computations are in double (the book squares the noise into float and uses 0.5f)
 *  - the GetDelta*() getters are const
 *  - the right Jacobian of SO(3) is computed as Jr(w) = Jl(-w) with upstream Sophus, since
 *    upstream Sophus has no jr() (the book ships a modified Sophus)
 */
class IMUPreintegration
{
public:
  using Vec3d = Eigen::Vector3d;
  using Mat3d = Eigen::Matrix3d;
  using Mat6d = Eigen::Matrix<double, 6, 6>;
  using Mat9d = Eigen::Matrix<double, 9, 9>;

  /// The initial biases must be set; the rest can stay at the defaults
  struct Options
  {
    Vec3d init_bg_ = Vec3d::Zero();   // initial gyro bias
    Vec3d init_ba_ = Vec3d::Zero();   // initial accelerometer bias
    double gyro_std_ = 1e-2;          // gyro measurement noise (standard deviation)
    double acce_std_ = 1e-1;          // accelerometer measurement noise (standard deviation)
  };

  IMUPreintegration()
  : IMUPreintegration(Options()) {}
  explicit IMUPreintegration(const Options & options);

  /**
   * Add one IMU reading
   * @param imu   IMU reading
   * @param dt    time since the previous reading [s]
   */
  void Integrate(const IMU & imu, double dt);

  /**
   * Predict the state at the end of the preintegration interval
   * @param start  state at the start of the interval
   * @param grav   gravity vector
   * @return       predicted state
   */
  NavState Predict(const NavState & start, const Vec3d & grav = Vec3d(0.0, 0.0, -9.81)) const;

  /// Preintegrated measurements corrected to first order for a bias different from the one
  /// used during the integration
  Sophus::SO3d GetDeltaRotation(const Vec3d & bg) const;
  Vec3d GetDeltaVelocity(const Vec3d & bg, const Vec3d & ba) const;
  Vec3d GetDeltaPosition(const Vec3d & bg, const Vec3d & ba) const;

  double dt_ = 0.0;                         // total preintegration time
  Mat9d cov_ = Mat9d::Zero();               // accumulated noise covariance, order: R, v, p
  Mat6d noise_gyro_acce_ = Mat6d::Zero();   // measurement noise (variances)

  /// Biases used during the integration
  Vec3d bg_ = Vec3d::Zero();
  Vec3d ba_ = Vec3d::Zero();

  /// Preintegrated measurements
  Sophus::SO3d dR_;
  Vec3d dv_ = Vec3d::Zero();
  Vec3d dp_ = Vec3d::Zero();

  /// Jacobians with respect to the biases
  Mat3d dR_dbg_ = Mat3d::Zero();
  Mat3d dV_dbg_ = Mat3d::Zero();
  Mat3d dV_dba_ = Mat3d::Zero();
  Mat3d dP_dbg_ = Mat3d::Zero();
  Mat3d dP_dba_ = Mat3d::Zero();
};
