#include "ch3_ros/gnss_ros.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssRos>());
  rclcpp::shutdown();

  return 0;
}
