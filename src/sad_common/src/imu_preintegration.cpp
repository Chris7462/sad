#include "sad_common/imu_preintegration.hpp"


IMUPreintegration::IMUPreintegration(const Options & options)
{
  bg_ = options.init_bg_;
  ba_ = options.init_ba_;

  // Standard deviations -> variances
  const double ng2 = options.gyro_std_ * options.gyro_std_;
  const double na2 = options.acce_std_ * options.acce_std_;
  noise_gyro_acce_.diagonal() << ng2, ng2, ng2, na2, na2, na2;
}

void IMUPreintegration::Integrate(const IMU & imu, double dt)
{
  // Measurements with the biases removed
  const Vec3d gyr = imu.gyro_ - bg_;
  const Vec3d acc = imu.acce_ - ba_;

  // Update dv, dp, eq. (4.13), (4.16)
  dp_ = dp_ + dv_ * dt + 0.5 * dR_.matrix() * acc * dt * dt;
  dv_ = dv_ + dR_ * acc * dt;

  // dR is not updated yet: A and B below still need the current dR

  // Coefficients of the noise propagation, A and B, eq. (4.29).
  // The rotation blocks are filled in further down.
  Mat9d A = Mat9d::Identity();
  Eigen::Matrix<double, 9, 6> B = Eigen::Matrix<double, 9, 6>::Zero();

  const Mat3d acc_hat = Sophus::SO3d::hat(acc);
  const double dt2 = dt * dt;

  // NOTE: the top-left blocks of A and B differ slightly from the book's equations
  A.block<3, 3>(3, 0) = -dR_.matrix() * dt * acc_hat;
  A.block<3, 3>(6, 0) = -0.5 * dR_.matrix() * acc_hat * dt2;
  A.block<3, 3>(6, 3) = dt * Mat3d::Identity();

  B.block<3, 3>(3, 3) = dR_.matrix() * dt;
  B.block<3, 3>(6, 3) = 0.5 * dR_.matrix() * dt2;

  // Update the bias Jacobians, eq. (4.39)
  dP_dba_ = dP_dba_ + dV_dba_ * dt - 0.5 * dR_.matrix() * dt2;                       // (4.39d)
  dP_dbg_ = dP_dbg_ + dV_dbg_ * dt - 0.5 * dR_.matrix() * dt2 * acc_hat * dR_dbg_;   // (4.39e)
  dV_dba_ = dV_dba_ - dR_.matrix() * dt;                                              // (4.39b)
  dV_dbg_ = dV_dbg_ - dR_.matrix() * dt * acc_hat * dR_dbg_;                          // (4.39c)

  // Rotation part
  const Vec3d omega = gyr * dt;
  // Right Jacobian of SO(3): Jr(w) = Jl(-w)
  const Mat3d right_j = Sophus::SO3d::leftJacobian(-omega);
  const Sophus::SO3d delta_R = Sophus::SO3d::exp(omega);
  dR_ = dR_ * delta_R;                                                                // (4.9)

  A.block<3, 3>(0, 0) = delta_R.matrix().transpose();
  B.block<3, 3>(0, 0) = right_j * dt;

  // Noise propagation
  cov_ = A * cov_ * A.transpose() + B * noise_gyro_acce_ * B.transpose();

  // Update dR_dbg
  dR_dbg_ = delta_R.matrix().transpose() * dR_dbg_ - right_j * dt;                   // (4.39a)

  dt_ += dt;
}

Sophus::SO3d IMUPreintegration::GetDeltaRotation(const Vec3d & bg) const
{
  return dR_ * Sophus::SO3d::exp(dR_dbg_ * (bg - bg_));
}

IMUPreintegration::Vec3d IMUPreintegration::GetDeltaVelocity(
  const Vec3d & bg, const Vec3d & ba) const
{
  return dv_ + dV_dbg_ * (bg - bg_) + dV_dba_ * (ba - ba_);
}

IMUPreintegration::Vec3d IMUPreintegration::GetDeltaPosition(
  const Vec3d & bg, const Vec3d & ba) const
{
  return dp_ + dP_dbg_ * (bg - bg_) + dP_dba_ * (ba - ba_);
}

NavState IMUPreintegration::Predict(const NavState & start, const Vec3d & grav) const
{
  const Sophus::SO3d Rj = start.R_ * dR_;
  const Vec3d vj = start.R_ * dv_ + start.v_ + grav * dt_;
  const Vec3d pj = start.R_ * dp_ + start.p_ + start.v_ * dt_ + 0.5 * grav * dt_ * dt_;

  return NavState(start.timestamp_ + dt_, Rj, pj, vj, bg_, ba_);
}
