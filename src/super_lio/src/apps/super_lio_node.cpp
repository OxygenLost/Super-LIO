
#include <memory>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "lio/params.h"
#include "lio/super_lio_reloc.h"
#include "ros/ROSWrapper.h"


using namespace LI2Sup;

namespace
{

constexpr char kImuType[] = "sensor_msgs/msg/Imu";
constexpr char kLidarType[] = "livox_ros_driver2/msg/CustomMsg";

std::size_t get_topic_message_count(
  const rosbag2_storage::BagMetadata & metadata,
  const std::string & topic_name)
{
  for (const auto & topic : metadata.topics_with_message_count) {
    if (topic.topic_metadata.name == topic_name) {
      return topic.message_count;
    }
  }
  return 0;
}

void print_progress(std::size_t current, std::size_t total)
{
  if (total == 0) {
    return;
  }

  constexpr int kBarWidth = 40;
  const double ratio = static_cast<double>(current) / static_cast<double>(total);
  const int filled = static_cast<int>(ratio * kBarWidth);

  std::ostringstream oss;
  oss << "\r[offline] lidar " << current << "/" << total << " [";
  for (int i = 0; i < kBarWidth; ++i) {
    oss << (i < filled ? '=' : ' ');
  }
  oss << "] " << std::setw(3) << static_cast<int>(ratio * 100.0) << "%";
  std::cout << oss.str() << std::flush;

  if (current >= total) {
    std::cout << std::endl;
  }
}

std::string format_duration(double seconds)
{
  const auto total_seconds = static_cast<long long>(std::max(0.0, seconds));
  const long long hours = total_seconds / 3600;
  const long long minutes = (total_seconds % 3600) / 60;
  const long long secs = total_seconds % 60;

  std::ostringstream oss;
  if (hours > 0) {
    oss << hours << "h";
  }
  if (hours > 0 || minutes > 0) {
    oss << minutes << "m";
  }
  oss << secs << "s";
  return oss.str();
}

void print_progress(std::size_t current, std::size_t total, double elapsed_seconds)
{
  if (total == 0) {
    return;
  }

  constexpr int kBarWidth = 40;
  const double ratio = static_cast<double>(current) / static_cast<double>(total);
  const int filled = static_cast<int>(ratio * kBarWidth);
  const double remaining_seconds =
    (current > 0) ? (elapsed_seconds / static_cast<double>(current)) *
      static_cast<double>(total - current) : 0.0;

  std::ostringstream oss;
  oss << "\r[offline] lidar " << current << "/" << total << " [";
  for (int i = 0; i < kBarWidth; ++i) {
    oss << (i < filled ? '=' : ' ');
  }
  oss << "] " << std::setw(3) << static_cast<int>(ratio * 100.0) << "% "
      << "elapsed " << format_duration(elapsed_seconds)
      << " eta " << format_duration(remaining_seconds);
  std::cout << oss.str() << std::flush;

  if (current >= total) {
    std::cout << std::endl;
  }
}

bool validate_topics(
  const std::vector<rosbag2_storage::TopicMetadata> & topics,
  rclcpp::Logger logger)
{
  std::unordered_map<std::string, std::string> topic_types;
  for (const auto & topic : topics) {
    topic_types.emplace(topic.name, topic.type);
  }

  const auto imu_it = topic_types.find(g_imu_topic);
  if (imu_it == topic_types.end()) {
    RCLCPP_ERROR(logger, "Bag is missing required topic %s", g_imu_topic.c_str());
    return false;
  }
  if (imu_it->second != kImuType) {
    RCLCPP_ERROR(
      logger,
      "Topic %s has type %s, expected %s",
      g_imu_topic.c_str(),
      imu_it->second.c_str(),
      kImuType);
    return false;
  }

  const auto lidar_it = topic_types.find(g_lidar_topic);
  if (lidar_it == topic_types.end()) {
    RCLCPP_ERROR(logger, "Bag is missing required topic %s", g_lidar_topic.c_str());
    return false;
  }
  if (lidar_it->second != kLidarType) {
    RCLCPP_ERROR(
      logger,
      "Topic %s has type %s, expected %s",
      g_lidar_topic.c_str(),
      lidar_it->second.c_str(),
      kLidarType);
    return false;
  }

  return true;
}

void run_offline(const ROSWrapper::Ptr & data_wrapper, const std::shared_ptr<SuperLIO> & lio)
{
  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = g_bagfile;
  storage_options.storage_id = "sqlite3";

  try {
    reader.open(storage_options);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      data_wrapper->get_logger(),
      "Failed to open bag %s: %s",
      g_bagfile.c_str(),
      ex.what());
    return;
  }

  if (!validate_topics(reader.get_all_topics_and_types(), data_wrapper->get_logger())) {
    return;
  }

  const std::size_t total_lidar_frames =
    get_topic_message_count(reader.get_metadata(), g_lidar_topic);
  std::size_t lidar_frames_processed = 0;
  int last_percent = -1;
  const auto start_time = std::chrono::steady_clock::now();

  rosbag2_storage::StorageFilter filter;
  filter.topics = {g_imu_topic, g_lidar_topic};
  reader.set_filter(filter);

  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
  rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> lidar_serialization;

  while (rclcpp::ok() && reader.has_next()) {
    auto bag_message = reader.read_next();
    rclcpp::SerializedMessage serialized_message(*bag_message->serialized_data);

    if (bag_message->topic_name == g_imu_topic) {
      sensor_msgs::msg::Imu imu_msg;
      imu_serialization.deserialize_message(&serialized_message, &imu_msg);
      data_wrapper->feedImu(imu_msg);
      continue;
    }

    if (bag_message->topic_name == g_lidar_topic) {
      livox_ros_driver2::msg::CustomMsg lidar_msg;
      lidar_serialization.deserialize_message(&serialized_message, &lidar_msg);
      data_wrapper->feedLivox(lidar_msg);
      while (rclcpp::ok() && data_wrapper->hasSynchronizedMeasure()) {
        lio->process();
      }

      ++lidar_frames_processed;
      if (total_lidar_frames > 0) {
        const double elapsed_seconds = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - start_time).count();
        const int percent = static_cast<int>(
          (100.0 * static_cast<double>(lidar_frames_processed)) /
          static_cast<double>(total_lidar_frames));
        if (percent != last_percent || lidar_frames_processed == total_lidar_frames) {
          print_progress(lidar_frames_processed, total_lidar_frames, elapsed_seconds);
          last_percent = percent;
        }
      }
    }
  }

  while (rclcpp::ok() && data_wrapper->hasSynchronizedMeasure()) {
    lio->process();
  }

  if (total_lidar_frames > 0 && lidar_frames_processed < total_lidar_frames) {
    const double elapsed_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time).count();
    print_progress(lidar_frames_processed, total_lidar_frames, elapsed_seconds);
    std::cout << std::endl;
  }
}

}  // namespace

int main(int argc, char** argv){
  rclcpp::init(argc, argv);

  ROSWrapper::Ptr data_wrapper = std::make_shared<ROSWrapper>();
  
  auto lio = std::make_shared<SuperLIO>();
  lio->setROSWrapper(data_wrapper);
  lio->init();

  if (!g_bagfile.empty()) {
    run_offline(data_wrapper, lio);
    if (rclcpp::ok()) {
      lio->saveMap();
      lio->printTimeRecord();
    }
    rclcpp::shutdown();
    return 0;
  }

  auto timer = data_wrapper->create_wall_timer(
    std::chrono::milliseconds(2),
    [lio]() { lio->process(); },
    data_wrapper->getSensorCallbackGroup()
  );

  rclcpp::spin(data_wrapper);

  lio->saveMap();
  lio->printTimeRecord();

  rclcpp::shutdown();
  return 0;
}
