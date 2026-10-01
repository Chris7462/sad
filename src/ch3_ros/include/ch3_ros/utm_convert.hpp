#pragma once

#include "ch3_ros/gnss.hpp"


/**
 * 计算本书的GNSS读数对应的UTM pose和六自由度Pose
 * @param gnss_reading  输入gnss读数
 * @param antenna_pos   安装位置
 * @param antenna_angle 安装偏角
 * @param map_origin    地图原点，指定时，将从UTM位置中减掉坐标原点
 * @return
 */
bool ConvertGps2UTM(GNSS & gnss_reading, const Eigen::Vector2d & antenna_pos, const double & antenna_angle,
    const Eigen::Vector3d & map_origin = Eigen::Vector3d::Zero());

/**
 * 仅转换平移部分的经纬度，不作外参和角度处理
 * @param gnss_reading
 * @return
 */
bool ConvertGps2UTMOnlyTrans(GNSS & gnss_reading);

/**
 * 经纬度转UTM (implemented with GeographicLib::UTMUPS)
 * NOTE 经纬度单位为度数
 * NOTE zone_ is set to 0 (UPS) for latitudes outside the UTM range (north of 84°N / south of 80°S)
 * @param latlon
 * @param utm_coor
 * @return false if GeographicLib rejects the input
 */
bool LatLon2UTM(const Eigen::Vector2d & latlon, UTMCoordinate & utm_coor);

/**
 * UTM转经纬度 (implemented with GeographicLib::UTMUPS)
 * NOTE 输出经纬度单位为度数
 * @param utm_coor
 * @param latlon
 * @return false if GeographicLib rejects the input
 */
bool UTM2LatLon(const UTMCoordinate & utm_coor, Eigen::Vector2d & latlon);
