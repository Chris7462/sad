#pragma once

#include <Eigen/Core>
#include <manif/SO3.h>

#include "ch3_ros/imu.hpp"

class IMUIntegration
{
public:
  IMUIntegration(const Eigen::Vector3d & gravity,
    const Eigen::Vector3d & init_bg, const Eigen::Vector3d & init_ba);

  void AddIMU(const IMU & imu);

  manif::SO3d GetR() const { return R_; }
  Eigen::Vector3d GetV() const { return v_; }
  Eigen::Vector3d GetP() const { return p_; }

private:
  Eigen::Vector3d gravity_ = Eigen::Vector3d(0.0, 0.0, -9.81);
  Eigen::Vector3d bg_;
  Eigen::Vector3d ba_;

  double timestamp_ = 0.0;

  manif::SO3d R_ = manif::SO3d::Identity();
  Eigen::Vector3d v_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_ = Eigen::Vector3d::Zero();
};
