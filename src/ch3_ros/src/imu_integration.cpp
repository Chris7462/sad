#include "ch3_ros/imu_integration.hpp"

IMUIntegration::IMUIntegration(const Eigen::Vector3d & gravity,
  const Eigen::Vector3d & init_bg, const Eigen::Vector3d & init_ba)
: gravity_(gravity), bg_(init_bg), ba_(init_ba)
{
}

void IMUIntegration::AddIMU(const IMU & imu)
{
  double dt = imu.timestamp_ - timestamp_;
  if (dt > 0.0 && dt < 0.1) {
    p_ = p_ + v_ * dt + 0.5 * (R_ * (imu.acce_ - ba_) + gravity_) * dt * dt;
    v_ = v_ + (R_ * (imu.acce_ - ba_) + gravity_) * dt;
    R_ = R_ * Sophus::SO3d::exp((imu.gyro_ - bg_) * dt);
  }

  timestamp_ = imu.timestamp_;
}
