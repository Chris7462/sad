// C++ standard library
#include <exception>
#include <memory>

// ROS 2
#include <rclcpp/rclcpp.hpp>

// local
#include "txt_to_ros2bag/txt_to_ros2bag.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int ret = 0;
  try {
    // One-shot conversion: no spinning needed.
    auto node = std::make_shared<Txt2BagNode>();
    ret = node->convert() ? 0 : 1;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("txt_to_ros2bag_node"), "Conversion failed: %s", e.what());
    ret = 1;
  }

  rclcpp::shutdown();
  return ret;
}
