#include "ch2_ros/motion.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimerPublisher>());
  rclcpp::shutdown();

  return 0;
}
