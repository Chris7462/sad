#include "ch3_ros/imu_ros.hpp"



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuRos>());
  rclcpp::shutdown();

  return 0;
}
