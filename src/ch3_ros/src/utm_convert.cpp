#include <cmath>
#include <GeographicLib/UTMUPS.hpp>

#include <rclcpp/logging.hpp>

#include "ch3_ros/gnss.hpp"
#include "ch3_ros/utm_convert.hpp"


constexpr double kDEG2RAD = M_PI / 180.0;  // deg->rad

namespace {
  const rclcpp::Logger kLogger = rclcpp::get_logger("utm_convert");
}

bool LatLon2UTM(const Eigen::Vector2d & latlon, UTMCoordinate & utm_coor)
{
  // GeographicLib works in degrees; throws GeographicErr on invalid input
  try {
    int zone = 0;
    bool northp = true;
    GeographicLib::UTMUPS::Forward(latlon[0], latlon[1], zone, northp, utm_coor.xy_[0], utm_coor.xy_[1]);
    utm_coor.zone_ = zone;  // 0 means UPS (polar regions), 1-60 means UTM
    utm_coor.north_ = northp;
  } catch (const GeographicLib::GeographicErr& e) {
    RCLCPP_ERROR(kLogger, "LatLon2UTM failed: %s", e.what());
    return false;
  }
  return true;
}

bool UTM2LatLon(const UTMCoordinate & utm_coor, Eigen::Vector2d & latlon)
{
  try {
    GeographicLib::UTMUPS::Reverse(
      utm_coor.zone_, utm_coor.north_, utm_coor.xy_[0], utm_coor.xy_[1], latlon[0], latlon[1]);
  } catch (const GeographicLib::GeographicErr& e) {
    RCLCPP_ERROR(kLogger, "UTM2LatLon failed: %s", e.what());
    return false;
  }
  return true;
}

bool ConvertGps2UTM(GNSS & gps_msg, const Eigen::Vector2d & antenna_pos,
  const double & antenna_angle, const Eigen::Vector3d & map_origin)
{
  UTMCoordinate utm_rtk;
  if (!LatLon2UTM(gps_msg.lat_lon_alt_.head<2>(), utm_rtk)) {
    return false;
  }
  utm_rtk.z_ = gps_msg.lat_lon_alt_[2];

  double heading = 0;
  if (gps_msg.heading_valid_) {
    heading = (90 - gps_msg.heading_) * kDEG2RAD;
  }

  /// TWG 转到 TWB
  //Sophus::SE3d TBG(Sophus::SO3d::rotZ(antenna_angle * kDEG2RAD),
  //                 Eigen::Vector3d(antenna_pos[0], antenna_pos[1], 0));
  //Sophus::SE3d TGB = TBG.inverse();
  manif::SE3d TBG(Eigen::Vector3d(antenna_pos[0], antenna_pos[1], 0),
                  manif::SO3d(0.0, 0.0, antenna_angle * kDEG2RAD));   // (roll, pitch, yaw)
  manif::SE3d TGB = TBG.inverse();

  /// 若指明地图原点，则减去地图原点
  double x = utm_rtk.xy_[0] - map_origin[0];
  double y = utm_rtk.xy_[1] - map_origin[1];
  double z = utm_rtk.z_ - map_origin[2];
  //Sophus::SE3d TWG(Sophus::SO3d::rotZ(heading), Eigen::Vector3d(x, y, z));
  //Sophus::SE3d TWB = TWG * TGB;
  manif::SE3d TWG(Eigen::Vector3d(x, y, z), manif::SO3d(0.0, 0.0, heading));
  manif::SE3d TWB = TWG * TGB;

  gps_msg.utm_valid_ = true;
  gps_msg.utm_.xy_[0] = TWB.translation().x();
  gps_msg.utm_.xy_[1] = TWB.translation().y();
  gps_msg.utm_.z_ = TWB.translation().z();

  if (gps_msg.heading_valid_) {
    gps_msg.utm_pose_ = TWB;
  } else {
    //gps_msg.utm_pose_ = Sophus::SE3d(Sophus::SO3d(), TWB.translation());
    gps_msg.utm_pose_ = manif::SE3d(TWB.translation(), Eigen::Quaterniond::Identity());
  }

  return true;
}

bool ConvertGps2UTMOnlyTrans(GNSS & gps_msg) {
  UTMCoordinate utm_rtk;
  if (!LatLon2UTM(gps_msg.lat_lon_alt_.head<2>(), utm_rtk)) {
    return false;
  }
  gps_msg.utm_valid_ = true;
  gps_msg.utm_.xy_ = utm_rtk.xy_;
  gps_msg.utm_.z_ = gps_msg.lat_lon_alt_[2];
  //gps_msg.utm_pose_ = Sophus::SE3d(
  //  Sophus::SO3d(), Eigen::Vector3d(gps_msg.utm_.xy_[0], gps_msg.utm_.xy_[1], gps_msg.utm_.z_));
  gps_msg.utm_pose_ = manif::SE3d(
    Eigen::Vector3d(gps_msg.utm_.xy_[0], gps_msg.utm_.xy_[1], gps_msg.utm_.z_),
    Eigen::Quaterniond::Identity());

  return true;
}
