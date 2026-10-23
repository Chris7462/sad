#include "ch4_ros/gins_ros.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GinsRos>());
  rclcpp::shutdown();

  return 0;
}
