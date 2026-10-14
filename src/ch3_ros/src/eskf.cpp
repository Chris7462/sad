#include <cmath>

#include "ch3_ros/eskf.hpp"


void ESKF::SetInitialConditions(
  const Options & options, const Vec3d & init_bg, const Vec3d & init_ba, const Vec3d & gravity)
{
  options_ = options;
  BuildNoise(options_);
  bg_ = init_bg;
  ba_ = init_ba;
  g_ = gravity;
  cov_ = Mat18d::Identity() * 1e-4;
}

void ESKF::SetX(const NavState & x, const Vec3d & gravity)
{
  current_time_ = x.timestamp_;
  R_ = x.R_;
  p_ = x.p_;
  v_ = x.v_;
  bg_ = x.bg_;
  ba_ = x.ba_;
  g_ = gravity;
}

void ESKF::BuildNoise(const Options & options)
{
  // All options are standard deviations: square them to get variances
  const double ev2 = options.acce_std_ * options.acce_std_;
  const double et2 = options.gyro_std_ * options.gyro_std_;
  const double eg2 = options.bias_gyro_std_ * options.bias_gyro_std_;
  const double ea2 = options.bias_acce_std_ * options.bias_acce_std_;

  // Process noise, order: p, v, theta, bg, ba, g
  Q_.setZero();
  Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, eg2, eg2, eg2, ea2, ea2, ea2, 0, 0, 0;

  // Wheel odometry noise
  const double o2 = options.odom_std_ * options.odom_std_;
  odom_noise_.setZero();
  odom_noise_.diagonal() << o2, o2, o2;

  // GNSS noise: x, y, height, then rotation
  const double gp2 = options.gnss_pos_std_ * options.gnss_pos_std_;
  const double gh2 = options.gnss_height_std_ * options.gnss_height_std_;
  const double ga2 = options.gnss_ang_std_ * options.gnss_ang_std_;
  gnss_noise_.setZero();
  gnss_noise_.diagonal() << gp2, gp2, gh2, ga2, ga2, ga2;
}

bool ESKF::Predict(const IMU & imu)
{
  const double dt = imu.timestamp_ - current_time_;

  if (dt < 0.0) {
    // Older than the filter time (arrived out of order): ignore, keep the filter time
    return false;
  }

  if (dt > 5.0 * options_.imu_dt_) {
    // Gap too large, e.g. the very first IMU reading: only move the filter time
    current_time_ = imu.timestamp_;
    return false;
  }

  // Nominal state propagation
  const Vec3d new_p = p_ + v_ * dt + 0.5 * (R_ * (imu.acce_ - ba_)) * dt * dt + 0.5 * g_ * dt * dt;
  const Vec3d new_v = v_ + R_ * (imu.acce_ - ba_) * dt + g_ * dt;
  const Sophus::SO3d new_R = R_ * Sophus::SO3d::exp((imu.gyro_ - bg_) * dt);

  // Error-state propagation: Jacobian F of the motion model, eq. (3.47).
  // Unlike the book, F is built before the nominal state is committed, so R_ here is R_k,
  // the same rotation the propagation of p and v above used.
  Mat18d F = Mat18d::Identity();                                                  // diagonal
  F.block<3, 3>(0, 3) = Mat3d::Identity() * dt;                                   // p wrt v
  F.block<3, 3>(3, 6) = -R_.matrix() * Sophus::SO3d::hat(imu.acce_ - ba_) * dt;   // v wrt theta
  F.block<3, 3>(3, 12) = -R_.matrix() * dt;                                       // v wrt ba
  F.block<3, 3>(3, 15) = Mat3d::Identity() * dt;                                  // v wrt g
  F.block<3, 3>(6, 6) = Sophus::SO3d::exp(-(imu.gyro_ - bg_) * dt).matrix();      // theta wrt theta
  F.block<3, 3>(6, 9) = -Mat3d::Identity() * dt;                                  // theta wrt bg

  // Mean and covariance prediction. dx_ is zero after every reset, so F * dx_ is skipped.
  cov_ = F * cov_.eval() * F.transpose() + Q_;

  // Commit the nominal state only now. The other states do not change.
  R_ = new_R;
  v_ = new_v;
  p_ = new_p;
  current_time_ = imu.timestamp_;

  return true;
}

bool ESKF::ObserveWheelSpeed(const Odom & odom)
{
  if (odom.timestamp_ < current_time_) {
    return false;
  }

  // 3D velocity observation, H is 3x18 with identity on the v block
  Eigen::Matrix<double, 3, 18> H = Eigen::Matrix<double, 3, 18>::Zero();
  H.block<3, 3>(0, 3) = Mat3d::Identity();

  const Eigen::Matrix<double, 18, 3> K =
    cov_ * H.transpose() * (H * cov_ * H.transpose() + odom_noise_).inverse();

  // Pulses -> wheel speed -> body velocity along x
  const double pulse_to_speed =
    options_.wheel_radius_ * 2.0 * M_PI / options_.circle_pulse_ / options_.odom_span_;
  const double average_vel = 0.5 * (odom.left_pulse_ + odom.right_pulse_) * pulse_to_speed;

  const Vec3d vel_world = R_ * Vec3d(average_vel, 0.0, 0.0);

  dx_ = K * (vel_world - v_);
  cov_ = (Mat18d::Identity() - K * H) * cov_;

  UpdateAndReset();
  return true;
}

bool ESKF::ObserveGps(const GNSS & gnss)
{
  if (first_gnss_) {
    // The first reading sets the initial pose
    R_ = gnss.utm_pose_.so3();
    p_ = gnss.utm_pose_.translation();
    first_gnss_ = false;
    current_time_ = gnss.unix_time_;
    return true;
  }

  if (gnss.unix_time_ < current_time_) {
    return false;
  }

  ObserveSE3(gnss.utm_pose_, gnss_noise_);
  current_time_ = gnss.unix_time_;
  return true;
}

bool ESKF::ObservePosition(double timestamp, const Vec3d & position)
{
  if (first_gnss_ || timestamp < current_time_) {
    return false;
  }

  // Observes p only: H is 3x18 with identity on the p block
  Eigen::Matrix<double, 3, 18> H = Eigen::Matrix<double, 3, 18>::Zero();
  H.block<3, 3>(0, 0) = Mat3d::Identity();

  // Position part of the GNSS noise (x, y, height)
  const Mat3d noise = gnss_noise_.topLeftCorner<3, 3>();

  const Eigen::Matrix<double, 18, 3> K =
    cov_ * H.transpose() * (H * cov_ * H.transpose() + noise).inverse();

  dx_ = K * (position - p_);
  cov_ = (Mat18d::Identity() - K * H) * cov_;

  UpdateAndReset();
  current_time_ = timestamp;
  return true;
}

bool ESKF::ObserveSE3(const Sophus::SE3d & pose, double trans_std, double ang_std)
{
  const double t2 = trans_std * trans_std;
  const double a2 = ang_std * ang_std;

  Mat6d noise = Mat6d::Zero();
  noise.diagonal() << t2, t2, t2, a2, a2, a2;
  return ObserveSE3(pose, noise);
}

bool ESKF::ObserveSE3(const Sophus::SE3d & pose, const Mat6d & noise)
{
  // Observes p and R: H is 6x18
  Eigen::Matrix<double, 6, 18> H = Eigen::Matrix<double, 6, 18>::Zero();
  H.block<3, 3>(0, 0) = Mat3d::Identity();   // p part
  H.block<3, 3>(3, 6) = Mat3d::Identity();   // R part, eq. (3.66)

  const Eigen::Matrix<double, 18, 6> K =
    cov_ * H.transpose() * (H * cov_ * H.transpose() + noise).inverse();

  // Innovation
  Vec6d innov = Vec6d::Zero();
  innov.head<3>() = pose.translation() - p_;
  innov.tail<3>() = (R_.inverse() * pose.so3()).log();   // eq. (3.67)

  dx_ = K * innov;
  cov_ = (Mat18d::Identity() - K * H) * cov_;

  UpdateAndReset();
  return true;
}

void ESKF::UpdateAndReset()
{
  p_ += dx_.segment<3>(0);
  v_ += dx_.segment<3>(3);
  R_ = R_ * Sophus::SO3d::exp(dx_.segment<3>(6));

  if (options_.update_bias_gyro_) {
    bg_ += dx_.segment<3>(9);
  }

  if (options_.update_bias_acce_) {
    ba_ += dx_.segment<3>(12);
  }

  g_ += dx_.segment<3>(15);

  ProjectCov();
  dx_.setZero();
}

void ESKF::ProjectCov()
{
  Mat18d J = Mat18d::Identity();
  J.block<3, 3>(6, 6) = Mat3d::Identity() - 0.5 * Sophus::SO3d::hat(dx_.segment<3>(6));
  cov_ = J * cov_ * J.transpose();
}
