// ROS 2
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

// local
#include "txt_to_ros2bag/bag_writer.hpp"


BagWriter::BagWriter(const std::string & output_bag_name)
{
  writer_ = std::make_unique<rosbag2_cpp::Writer>();

  // Configure storage options for MCAP format
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = output_bag_name;
  storage_options.storage_id = "mcap";

  // Configure converter options
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  writer_->open(storage_options, converter_options);
}

void BagWriter::create_topic(
  const std::string & topic_name,
  const std::string & topic_type,
  const rclcpp::QoS & qos)
{
  rosbag2_storage::TopicMetadata topic_metadata;
  topic_metadata.name = topic_name;
  topic_metadata.type = topic_type;
  topic_metadata.serialization_format = "cdr";
  // `ros2 bag play` publishes with the QoS recorded here.
  topic_metadata.offered_qos_profiles = {qos};
  // Empty string is valid when not recording live type introspection data.
  topic_metadata.type_description_hash = "";
  writer_->create_topic(topic_metadata);
}
