#include "ch3_ros/eskf_ros.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EskfRos>());
  rclcpp::shutdown();

  return 0;
}
