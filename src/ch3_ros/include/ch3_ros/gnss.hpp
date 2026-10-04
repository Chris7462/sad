#pragma once

#include <Eigen/Core>
//#include <sophus/se3.hpp>
#include <manif/SE3.h>


#include <rclcpp/time.hpp>
#include <sad_msgs/msg/gnss.hpp>

enum class GpsStatusType
{
  GNSS_FLOAT_SOLUTION = 5,
  GNSS_FIXED_SOLUTION = 4,
  GNSS_PSEUDO_SOLUTION = 2,
  GNSS_SINGLE_POINT_SOLUTION = 1,
  GNSS_NOT_EXIST = 0,
  GNSS_OTHER = -1,
};

struct UTMCoordinate {
  UTMCoordinate() = default;
  explicit UTMCoordinate(int zone, const Eigen::Vector2d & xy = Eigen::Vector2d::Zero(), bool north = true)
  : zone_(zone), xy_(xy), north_(north) {}

  int zone_ = 0;
  Eigen::Vector2d xy_ = Eigen::Vector2d::Zero();
  double z_ = 0;
  bool north_ = true;
};

struct GNSS {
  GNSS() = default;
  GNSS(double unix_time, int status, const Eigen::Vector3d & lat_lon_alt, double heading, bool heading_valid)
    : unix_time_(unix_time),
      status_(GpsStatusType(status)),
      lat_lon_alt_(lat_lon_alt),
      heading_(heading),
      heading_valid_(heading_valid) {}

  explicit GNSS(const sad_msgs::msg::Gnss & msg)
  : unix_time_(rclcpp::Time(msg.header.stamp).seconds()),
    status_(GpsStatusType(msg.status)),
    lat_lon_alt_(msg.latitude, msg.longitude, msg.altitude),
    heading_(msg.heading),
    heading_valid_(msg.heading_valid) {}

  double unix_time_ = 0;
  GpsStatusType status_ = GpsStatusType::GNSS_NOT_EXIST;
  Eigen::Vector3d lat_lon_alt_ = Eigen::Vector3d::Zero();
  double heading_ = 0.0;
  bool heading_valid_ = false;

  UTMCoordinate utm_;
  bool utm_valid_ = false;

  //Sophus::SE3d utm_pose_;
  manif::SE3d utm_pose_;
};
