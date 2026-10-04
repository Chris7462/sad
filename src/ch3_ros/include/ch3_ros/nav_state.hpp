#pragma once

#include <Eigen/Core>
#include <manif/SO3.h>

/**
 * Navigation state: time, R, p, v, bg, ba
 */
struct NavState
{
  NavState() = default;

  explicit NavState(
    double time,
    const manif::SO3d & R = manif::SO3d::Identity(),
    const Eigen::Vector3d & p = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d & v = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d & bg = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d & ba = Eigen::Vector3d::Zero())
  : timestamp_(time), R_(R), p_(p), v_(v), bg_(bg), ba_(ba) {}

  double timestamp_ = 0.0;
  manif::SO3d R_ = manif::SO3d::Identity();           // rotation, body -> world
  Eigen::Vector3d p_ = Eigen::Vector3d::Zero();       // position in world
  Eigen::Vector3d v_ = Eigen::Vector3d::Zero();       // velocity in world
  Eigen::Vector3d bg_ = Eigen::Vector3d::Zero();      // gyro bias
  Eigen::Vector3d ba_ = Eigen::Vector3d::Zero();      // accelerometer bias
};
