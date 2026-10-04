#pragma once

#include <cstddef>
#include <deque>

#include <Eigen/Core>

#include "ch3_ros/imu.hpp"
#include "ch3_ros/odom.hpp"

/**
 * Static IMU initializer (book section 3.5.4).
 *
 * Feed it with AddIMU / AddOdom. While the wheel odometry says the vehicle is standing still,
 * it collects IMU readings; after init_time_seconds_ it estimates the gyro / accelerometer
 * biases, the measurement noise and the gravity vector. Check InitSuccess(), then use the getters.
 */
class StaticIMUInit
{
public:
  struct Options
  {
    double init_time_seconds_ = 10.0;         // how long the vehicle must stand still
    size_t init_imu_queue_max_size_ = 2000;   // max IMU samples kept for initialization
    double static_odom_pulse_ = 5.0;          // |pulse| below this counts as standing still
    double max_static_gyro_var_ = 0.5;        // max gyro variance accepted as static
    double max_static_acce_var_ = 0.05;       // max accelerometer variance accepted as static
    double gravity_norm_ = 9.81;
    bool use_speed_for_static_checking_ = true;   // false: assume static at start (no odom)
  };

  StaticIMUInit() = default;
  explicit StaticIMUInit(const Options & options)
  : options_(options) {}

  bool AddIMU(const IMU & imu);
  bool AddOdom(const Odom & odom);

  bool InitSuccess() const {return init_success_;}

  Eigen::Vector3d GetCovGyro() const {return cov_gyro_;}
  Eigen::Vector3d GetCovAcce() const {return cov_acce_;}
  Eigen::Vector3d GetInitBg() const {return init_bg_;}
  Eigen::Vector3d GetInitBa() const {return init_ba_;}
  Eigen::Vector3d GetGravity() const {return gravity_;}

private:
  bool TryInit();

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
};
