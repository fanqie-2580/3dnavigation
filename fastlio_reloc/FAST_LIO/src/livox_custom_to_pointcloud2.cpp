#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>

struct EIGEN_ALIGN16 LivoxPointXYZIRTLT
{
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  std::uint16_t ring;
  std::uint8_t tag;
  std::uint8_t line;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
  LivoxPointXYZIRTLT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (float, time, time)
  (std::uint16_t, ring, ring)
  (std::uint8_t, tag, tag)
  (std::uint8_t, line, line))

class LivoxCustomToPointCloud2Node : public rclcpp::Node
{
public:
  LivoxCustomToPointCloud2Node()
  : Node("livox_custom_to_pointcloud2")
  {
    declare_parameter<std::string>("input_topic", "/livox/lidar");
    declare_parameter<std::string>("output_topic", "/livox/lidar_pointcloud2");
    declare_parameter<std::string>("frame_id_override", "");

    get_parameter("input_topic", input_topic_);
    get_parameter("output_topic", output_topic_);
    get_parameter("frame_id_override", frame_id_override_);

    // DDDMR's lego_loam input subscribes with the default reliable QoS.
    // Publish reliable here so the converted cloud can be consumed by both
    // reliable and best-effort subscribers.
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LivoxCustomToPointCloud2Node::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Converting Livox CustomMsg to PointCloud2: input=%s output=%s frame_id_override=%s",
      input_topic_.c_str(),
      output_topic_.c_str(),
      frame_id_override_.empty() ? "<message header>" : frame_id_override_.c_str());
  }

private:
  void cloudCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    if (msg->points.empty()) {
      return;
    }

    const std::string frame_id = frame_id_override_.empty() ? msg->header.frame_id : frame_id_override_;
    if (frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Livox CustomMsg frame_id is empty and frame_id_override is not set.");
      return;
    }

    pcl::PointCloud<LivoxPointXYZIRTLT> cloud;
    cloud.points.reserve(msg->points.size());

    for (const auto & src_point : msg->points) {
      if (!std::isfinite(src_point.x) || !std::isfinite(src_point.y) || !std::isfinite(src_point.z)) {
        continue;
      }

      LivoxPointXYZIRTLT dst_point;
      dst_point.x = src_point.x;
      dst_point.y = src_point.y;
      dst_point.z = src_point.z;
      dst_point.intensity = static_cast<float>(src_point.reflectivity);
      dst_point.time = static_cast<float>(static_cast<double>(src_point.offset_time) * 1e-9);
      dst_point.ring = static_cast<std::uint16_t>(src_point.line);
      dst_point.tag = src_point.tag;
      dst_point.line = src_point.line;
      cloud.points.push_back(dst_point);
    }

    if (cloud.points.empty()) {
      return;
    }

    cloud.width = static_cast<std::uint32_t>(cloud.points.size());
    cloud.height = 1;
    cloud.is_dense = false;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header = msg->header;
    output.header.frame_id = frame_id;
    publisher_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_override_;

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LivoxCustomToPointCloud2Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
