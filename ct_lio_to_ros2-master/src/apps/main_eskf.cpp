// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <functional>
#include <memory>

#include <Eigen/Geometry>

// ros2 lib
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <random>

#include "common/utility.h"
#include "common/math_utils.h"
#include "preprocess/cloud_convert/cloud_convert.h"
#include "liw/lio/lidarodom.h"

using std::placeholders::_1;

nav_msgs::msg::Path laserOdoPath;

zjloc::lidarodom *lio;
zjloc::CloudConvert *convert;

#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "log/" + name))

namespace
{
SE3 buildPoseFromTranslationAndRpyDeg(
    const std::vector<double> &translation,
    const std::vector<double> &rpy_deg)
{
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < 3 && i < translation.size(); ++i)
  {
    t[static_cast<int>(i)] = translation[i];
  }

  const double roll = (rpy_deg.size() > 0 ? rpy_deg[0] : 0.0) * kDegToRad;
  const double pitch = (rpy_deg.size() > 1 ? rpy_deg[1] : 0.0) * kDegToRad;
  const double yaw = (rpy_deg.size() > 2 ? rpy_deg[2] : 0.0) * kDegToRad;

  const Eigen::Quaterniond q =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  return SE3(q.normalized(), t);
}

geometry_msgs::msg::TransformStamped toTransformStamped(
    const rclcpp::Time &stamp,
    const std::string &parent_frame,
    const std::string &child_frame,
    const SE3 &pose)
{
  geometry_msgs::msg::TransformStamped transform;
  const Eigen::Quaterniond q_current(pose.so3().matrix());

  transform.header.stamp = stamp;
  transform.header.frame_id = parent_frame;
  transform.child_frame_id = child_frame;
  transform.transform.translation.x = pose.translation().x();
  transform.transform.translation.y = pose.translation().y();
  transform.transform.translation.z = pose.translation().z();
  transform.transform.rotation.x = q_current.x();
  transform.transform.rotation.y = q_current.y();
  transform.transform.rotation.z = q_current.z();
  transform.transform.rotation.w = q_current.w();

  return transform;
}
} // namespace

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
  std::vector<point3D> cloud_out;
  zjloc::common::Timer::Evaluate([&]()
                                 { convert->Process(msg, cloud_out); },
                                 "laser convert");

  zjloc::common::Timer::Evaluate([&]()
                                 { 
        double sample_size = lio->getIndex() < 20 ? 0.01 : 0.01;
        // double sample_size = 0.01;
        std::mt19937_64 g;
        std::shuffle(cloud_out.begin(), cloud_out.end(), g);
        subSampleFrame(cloud_out, sample_size);
        std::shuffle(cloud_out.begin(), cloud_out.end(), g); },
                                 "laser ds");

  lio->pushData(cloud_out, std::make_pair(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9, convert->getTimeSpan()));
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  static int c = 0;
  // if (c % 2 == 0 && use_velodyne)
  {
    std::vector<point3D> cloud_out;
    zjloc::common::Timer::Evaluate([&]()
                                   { convert->Process(msg, cloud_out); },
                                   "laser convert");

    zjloc::common::Timer::Evaluate([&]() { // boost::mt19937_64 g;
      double sample_size = lio->getIndex() < 20 ? 0.01 : 0.05;
      // double sample_size = 0.05;
      std::mt19937_64 g;
      std::shuffle(cloud_out.begin(), cloud_out.end(), g);
      subSampleFrame(cloud_out, sample_size);
      std::shuffle(cloud_out.begin(), cloud_out.end(), g);
    },
                                   "laser ds");

    // lio->pushData(cloud_out, std::make_pair(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9 - convert->getTimeSpan(), convert->getTimeSpan())); //  FIXME: for staircase dataset(header timestamp is the frame end)
    lio->pushData(cloud_out, std::make_pair(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9, convert->getTimeSpan())); //  normal
  }
  c++;
}

void imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Mid-360 / livox_ros_driver2 typically publishes linear_acceleration in g.
  // CT-LIO ESKF and gravity init expect m/s^2. Auto-detect by magnitude.
  Vec3d acce(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  const double acc_norm = acce.norm();
  if (acc_norm > 0.5 && acc_norm < 3.0)
  {
    acce *= zjloc::math::G_m_s2;
  }
  IMUPtr imu = std::make_shared<zjloc::IMU>(
      msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
      Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
      acce);
  lio->pushData(imu);
}

void updateStatus(const std_msgs::msg::Int32::SharedPtr msg)
{
  int type = msg->data;
  if (type == 1)
  {
  }
  else if (type == 2)
  {
  }
  else if (type == 3)
    ;
  else if (type == 4)
    ;
  else
    ;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  google::InitGoogleLogging("ct_lio");

  auto node = std::make_shared<rclcpp::Node>("ct_lio");

  lio = new zjloc::lidarodom();
  if (!lio->init(node))
  {
    return -1;
  }

  node->declare_parameter<std::string>("map_frame", "map");
  node->declare_parameter<std::string>("base_frame", "base_link");
  node->declare_parameter<std::string>("imu_frame", "mid360_imu");
  node->declare_parameter<std::string>("lidar_frame", "mid360_lidar");
  node->declare_parameter<bool>("publish_static_sensor_tf", true);
  node->declare_parameter<std::vector<double>>("base_to_lidar_translation", std::vector<double>{0.0, 0.0, 0.0});
  node->declare_parameter<std::vector<double>>("base_to_lidar_rpy_deg", std::vector<double>{0.0, 0.0, 0.0});

  std::string map_frame;
  std::string base_frame;
  std::string imu_frame;
  std::string lidar_frame;
  bool publish_static_sensor_tf = true;
  std::vector<double> base_to_lidar_translation;
  std::vector<double> base_to_lidar_rpy_deg;

  node->get_parameter("map_frame", map_frame);
  node->get_parameter("base_frame", base_frame);
  node->get_parameter("imu_frame", imu_frame);
  node->get_parameter("lidar_frame", lidar_frame);
  node->get_parameter("publish_static_sensor_tf", publish_static_sensor_tf);
  node->get_parameter("base_to_lidar_translation", base_to_lidar_translation);
  node->get_parameter("base_to_lidar_rpy_deg", base_to_lidar_rpy_deg);

  const SE3 T_base_lidar = buildPoseFromTranslationAndRpyDeg(
      base_to_lidar_translation, base_to_lidar_rpy_deg);
  // TIL_ / getLidarToImuExtrinsic() is T_imu_lidar: p_imu = TIL_ * p_lidar.
  const SE3 T_imu_lidar = lio->getLidarToImuExtrinsic();
  const SE3 T_base_imu = T_base_lidar * T_imu_lidar.inverse();
  const SE3 T_imu_base = T_base_imu.inverse();

  auto pub_scan = node->create_publisher<sensor_msgs::msg::PointCloud2>("scan", 10);
  auto pub_cloud_registered = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 10);
  auto laser_map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  auto pub_laser_map = node->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", laser_map_qos);
  auto cloud_pub_func = std::function<bool(std::string & topic_name, zjloc::CloudPtr & cloud, double time)>(
      [&](std::string &topic_name, zjloc::CloudPtr &cloud, double time)
      {
        auto cloud_ptr_output = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*cloud, *cloud_ptr_output);

        cloud_ptr_output->header.stamp = rclcpp::Time(static_cast<uint64_t>(time * 1e9));
        cloud_ptr_output->header.frame_id = map_frame;
        if (topic_name == "laser")
        {
          pub_scan->publish(*cloud_ptr_output);
        }
        else if (topic_name == "cloud_registered")
        {
          pub_cloud_registered->publish(*cloud_ptr_output);
        }
        else if (topic_name == "laser_map")
        {
          pub_laser_map->publish(*cloud_ptr_output);
        }
        else
          ; // publisher_->publish(*cloud_ptr_output);
        return true;
      }

  );

  auto pubLaserOdometry = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 100);
  auto pubLaserOdometryPath = node->create_publisher<nav_msgs::msg::Path>("/odometry_path", 5);

  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  auto static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  if (publish_static_sensor_tf)
  {
    const auto static_stamp = node->now();
    static_tf_broadcaster->sendTransform(
        toTransformStamped(static_stamp, base_frame, lidar_frame, T_base_lidar));
    static_tf_broadcaster->sendTransform(
        toTransformStamped(static_stamp, base_frame, imu_frame, T_base_imu));
  }

  auto pose_pub_func = std::function<bool(std::string & topic_name, SE3 & pose, double stamp)>(
      [&](std::string &topic_name, SE3 &pose, double stamp)
      {
        if (topic_name == "laser")
        {
          const auto ros_stamp = rclcpp::Time(static_cast<uint64_t>(stamp * 1e9));
          const SE3 base_pose = pose * T_imu_base;
          const Eigen::Quaterniond q_current(base_pose.so3().matrix());
          const auto transform = toTransformStamped(
              ros_stamp, map_frame, base_frame, base_pose);

          tf_broadcaster->sendTransform(transform);

          // publish odometry
          auto laserOdometry = std::make_shared<nav_msgs::msg::Odometry>();
          laserOdometry->header.frame_id = map_frame;
          laserOdometry->child_frame_id = base_frame;
          laserOdometry->header.stamp = ros_stamp;

          laserOdometry->pose.pose.orientation.x = q_current.x();
          laserOdometry->pose.pose.orientation.y = q_current.y();
          laserOdometry->pose.pose.orientation.z = q_current.z();
          laserOdometry->pose.pose.orientation.w = q_current.w();
          laserOdometry->pose.pose.position.x = base_pose.translation().x();
          laserOdometry->pose.pose.position.y = base_pose.translation().y();
          laserOdometry->pose.pose.position.z = base_pose.translation().z();
          pubLaserOdometry->publish(*laserOdometry);

          //  publish path
          auto laserPose = std::make_shared<geometry_msgs::msg::PoseStamped>();
          laserPose->header = laserOdometry->header;
          laserPose->pose = laserOdometry->pose.pose;
          laserOdoPath.header.stamp = laserOdometry->header.stamp;
          laserOdoPath.poses.push_back(*laserPose);
          laserOdoPath.header.frame_id = map_frame;
          pubLaserOdometryPath->publish(laserOdoPath);
        }

        return true;
      }

  );

  auto vel_pub = node->create_publisher<std_msgs::msg::Float32>("/velocity", 1);
  auto dist_pub = node->create_publisher<std_msgs::msg::Float32>("/move_dist", 1);

  auto data_pub_func = std::function<bool(std::string & topic_name, double time1, double time2)>(
      [&](std::string &topic_name, double time1, double time2)
      {
        auto time_rviz = std::make_shared<std_msgs::msg::Float32>();

        time_rviz->data = time1;
        if (topic_name == "velocity")
          vel_pub->publish(*time_rviz);
        else
          dist_pub->publish(*time_rviz);

        return true;
      }

  );

  lio->setFunc(cloud_pub_func);
  lio->setFunc(pose_pub_func);
  lio->setFunc(data_pub_func);

  convert = new zjloc::CloudConvert;
  convert->LoadFromYAML(node);
  std::cout << ANSI_COLOR_GREEN_BOLD << "init successful" << ANSI_COLOR_RESET << std::endl;

  node->declare_parameter<std::string>("save_map_path", "ct_lio_map.pcd");

  auto save_map_service = node->create_service<std_srvs::srv::Trigger>(
      "/save_map",
      [node](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        (void)request;
        std::string save_map_path;
        node->get_parameter("save_map_path", save_map_path);
        if (save_map_path.empty())
        {
          save_map_path = "ct_lio_map.pcd";
        }

        const bool saved = lio->saveMapToPcd(save_map_path);
        response->success = saved;
        response->message = saved ? ("Saved map to " + save_map_path)
                                  : ("Failed to save map to " + save_map_path);
      });

  std::string laser_topic;
  std::string imu_topic;

  node->declare_parameter<std::string>("lid_topic", "livox/lidar");
  node->get_parameter("lid_topic", laser_topic);

  node->declare_parameter<std::string>("imu_topic", "livox/imu");
  node->get_parameter("imu_topic", imu_topic);

  // 声明两个订阅者指针，但只使用其中一个
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subLaserCloudLivox;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudStandard;

  // 在 main 函数中替换原来的订阅者创建代码
  if (convert->lidar_type_ == zjloc::CloudConvert::LidarType::AVIA)
  {
    subLaserCloudLivox = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        laser_topic, 100, livox_pcl_cbk);
  }
  else
  {
    subLaserCloudStandard = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        laser_topic, 100, standard_pcl_cbk);
  }

  auto sub_imu_ori = node->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, 500, imuHandler);

  auto sub_type = node->create_subscription<std_msgs::msg::Int32>(
      "/change_status", 2, updateStatus);

  std::thread measurement_process(&zjloc::lidarodom::run, lio);
  measurement_process.detach();

  rclcpp::spin(node);

  zjloc::common::Timer::PrintAll();
  zjloc::common::Timer::DumpIntoFile(DEBUG_FILE_DIR("log_time.txt"));

  std::cout << ANSI_COLOR_GREEN_BOLD << " out done. " << ANSI_COLOR_RESET << std::endl;

  sleep(3);
  rclcpp::shutdown();
  return 0;
}
