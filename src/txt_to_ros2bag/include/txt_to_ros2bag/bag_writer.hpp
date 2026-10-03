#pragma once

// C++ standard library
#include <memory>
#include <string>
#include <vector>

// ROS 2
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosidl_runtime_cpp/traits.hpp>


class BagWriter
{
public:
  explicit BagWriter(const std::string & output_bag_name);

  // Register one topic. Must be called before the first write on that topic.
  void create_topic(
    const std::string & topic_name,
    const std::string & topic_type,
    const rclcpp::QoS & qos);

  template<typename T>
  void write_message(const T & msg, const std::string & topic_name, const rclcpp::Time & timestamp)
  {
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
    rclcpp::Serialization<T> serializer;
    serializer.serialize_message(&msg, serialized_msg.get());

    writer_->write(
      serialized_msg,
      topic_name,
      rosidl_generator_traits::name<T>(),
      timestamp);
  }

private:
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};
