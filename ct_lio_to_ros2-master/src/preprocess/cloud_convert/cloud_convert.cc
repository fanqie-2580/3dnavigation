#include "cloud_convert.h"

#include <cmath>
#include <cstring>
#include <glog/logging.h>
// #include <execution>

namespace zjloc
{

  void CloudConvert::Process(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg, std::vector<point3D> &pcl_out)
  {
    AviaHandler(msg);
    pcl_out = cloud_out_;
  }

  void CloudConvert::Process(const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
                             std::vector<point3D> &pcl_out)
  {
    switch (lidar_type_)
    {
    case LidarType::OUST64:
      Oust64Handler(msg);
      break;

    case LidarType::VELO32:
      VelodyneHandler(msg);
      break;

    case LidarType::ROBOSENSE16:
      RobosenseHandler(msg);
      break;

    case LidarType::PANDAR:
      PandarHandler(msg);
      break;

    case LidarType::ODIN1:
      Odin1Handler(msg);
      break;

    default:
      LOG(ERROR) << "Error LiDAR Type: " << int(lidar_type_);
      break;
    }
    pcl_out = cloud_out_;
  }

  void CloudConvert::AviaHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg)
  {
    cloud_out_.clear();
    cloud_full_.clear();
    int plsize = msg->point_num;
    cloud_out_.reserve(plsize);

    static double tm_scale = 1e9;

    double headertime = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    timespan_ = msg->points.back().offset_time / tm_scale;

    // std::cout << "span:" << timespan_ << ",0: " << msg->points[0].offset_time / tm_scale
    //           << " , 100: " << msg->points[100].offset_time / tm_scale << std::endl;

    for (int i = 0; i < plsize; i++)
    {
      if (!(std::isfinite(msg->points[i].x) &&
            std::isfinite(msg->points[i].y) &&
            std::isfinite(msg->points[i].z)))
        continue;

      if (i % point_filter_num_ != 0)
        continue;

      double range = msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y +
                     msg->points[i].z * msg->points[i].z;
      if (range > 150 * 150 || range < blind * blind)
        continue;

      if (/*(msg->points[i].line < N_SCANS) &&*/ ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        point3D point_temp;
        point_temp.raw_point = Eigen::Vector3d(msg->points[i].x, msg->points[i].y, msg->points[i].z);
        point_temp.point = point_temp.raw_point;
        point_temp.relative_time = msg->points[i].offset_time / tm_scale; // curvature unit: ms
        point_temp.intensity = msg->points[i].reflectivity;

        point_temp.timestamp = headertime + point_temp.relative_time;
        point_temp.alpha_time = point_temp.relative_time / timespan_;
        point_temp.timespan = timespan_;
        point_temp.ring = msg->points[i].line;

        cloud_out_.push_back(point_temp);
      }
    }
  }

  void CloudConvert::Oust64Handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)
  {
    cloud_out_.clear();
    cloud_full_.clear();
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    cloud_out_.reserve(plsize);

    static double tm_scale = 1e9;

    double headertime = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    timespan_ = pl_orig.points.back().t / tm_scale;
    // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[0].t / tm_scale
    //           << " , 100: " << pl_orig.points[100].t / tm_scale
    //           << std::endl;

    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      if (!(std::isfinite(pl_orig.points[i].x) &&
            std::isfinite(pl_orig.points[i].y) &&
            std::isfinite(pl_orig.points[i].z)))
        continue;

      if (i % point_filter_num_ != 0)
        continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                     pl_orig.points[i].z * pl_orig.points[i].z;
      if (range > 150 * 150 || range < blind * blind)
        continue;

      point3D point_temp;
      point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
      point_temp.point = point_temp.raw_point;
      point_temp.relative_time = pl_orig.points[i].t / tm_scale; // curvature unit: ms
      point_temp.intensity = pl_orig.points[i].intensity;

      point_temp.timestamp = headertime + point_temp.relative_time;
      point_temp.alpha_time = point_temp.relative_time / timespan_;
      point_temp.timespan = timespan_;
      point_temp.ring = pl_orig.points[i].ring;

      cloud_out_.push_back(point_temp);
    }
  }

  void CloudConvert::RobosenseHandler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)
  {
    cloud_out_.clear();
    cloud_full_.clear();
    pcl::PointCloud<robosense_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    cloud_out_.reserve(plsize);

    double headertime = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    //  FIXME:  时间戳大于0.1
    auto time_list_robosense = [&](robosense_ros::Point &point_1, robosense_ros::Point &point_2)
    {
      return (point_1.timestamp < point_2.timestamp);
    };
    sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_robosense);
    while (pl_orig.points[plsize - 1].timestamp - pl_orig.points[0].timestamp >= 0.1)
    {
      plsize--;
      pl_orig.points.pop_back();
    }

    timespan_ = pl_orig.points.back().timestamp - pl_orig.points[0].timestamp;

    // std::cout << timespan_ << std::endl;

    // std::cout << pl_orig.points[1].timestamp - pl_orig.points[0].timestamp << ", "
    //           << msg->header.stamp.toSec() - pl_orig.points[0].timestamp << ", "
    //           << msg->header.stamp.toSec() - pl_orig.points.back().timestamp << std::endl;

    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      // if (i % point_filter_num_ != 0)
      //     continue;
      if (!(std::isfinite(pl_orig.points[i].x) &&
            std::isfinite(pl_orig.points[i].y) &&
            std::isfinite(pl_orig.points[i].z)))
        continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                     pl_orig.points[i].z * pl_orig.points[i].z;
      if (range > 150 * 150 || range < blind * blind)
        continue;

      point3D point_temp;
      point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
      point_temp.point = point_temp.raw_point;
      point_temp.relative_time = pl_orig.points[i].timestamp - pl_orig.points[0].timestamp; // curvature unit: s
      point_temp.intensity = pl_orig.points[i].intensity;

      // point_temp.timestamp = headertime + point_temp.relative_time;
      point_temp.timestamp = pl_orig.points[i].timestamp;
      point_temp.alpha_time = point_temp.relative_time / timespan_;
      point_temp.timespan = timespan_;
      point_temp.ring = pl_orig.points[i].ring;
      if (point_temp.alpha_time > 1 || point_temp.alpha_time < 0)
        std::cout << point_temp.alpha_time << ", this may error." << std::endl;

      cloud_out_.push_back(point_temp);
    }
  }

  void CloudConvert::VelodyneHandler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)
  {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    cloud_out_.reserve(plsize);

    double headertime = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    static double tm_scale = 1; //   1e6 - nclt kaist or 1

    //  FIXME:  nclt 及kaist时间戳大于0.1
    auto time_list_velodyne = [&](velodyne_ros::Point &point_1, velodyne_ros::Point &point_2)
    {
      return (point_1.time < point_2.time);
    };
    sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_velodyne);
    while (pl_orig.points[plsize - 1].time / tm_scale >= 0.1)
    {
      plsize--;
      pl_orig.points.pop_back();
    }
    timespan_ = pl_orig.points.back().time / tm_scale;
    // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[0].time / tm_scale << " , 100: " << pl_orig.points[100].time / tm_scale << std::endl;

    for (int i = 0; i < plsize; i++)
    {
      if (!(std::isfinite(pl_orig.points[i].x) &&
            std::isfinite(pl_orig.points[i].y) &&
            std::isfinite(pl_orig.points[i].z)))
        continue;

      if (i % point_filter_num_ != 0)
        continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                     pl_orig.points[i].z * pl_orig.points[i].z;
      if (range > 150 * 150 || range < blind * blind)
        continue;

      point3D point_temp;
      point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
      point_temp.point = point_temp.raw_point;
      point_temp.relative_time = pl_orig.points[i].time / tm_scale; // curvature unit: s
      point_temp.intensity = pl_orig.points[i].intensity;

      point_temp.timestamp = headertime + point_temp.relative_time;
      point_temp.alpha_time = point_temp.relative_time / timespan_;
      point_temp.timespan = timespan_;
      point_temp.ring = pl_orig.points[i].ring;

      cloud_out_.push_back(point_temp);
    }
  }

  void CloudConvert::PandarHandler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)
  {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<pandar_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    cloud_out_.reserve(plsize);

    double headertime = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    static double tm_scale = 1; //   1e6

    auto time_list_pandar = [&](pandar_ros::Point &point_1, pandar_ros::Point &point_2)
    {
      return (point_1.timestamp < point_2.timestamp);
    };
    sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_pandar);
    while (pl_orig.points[plsize - 1].timestamp - pl_orig.points[0].timestamp >= 0.1)
    {
      plsize--;
      pl_orig.points.pop_back();
    }
    timespan_ = pl_orig.points.back().timestamp - pl_orig.points[0].timestamp;

    // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[1].timestamp - pl_orig.points[0].timestamp
    //           << " , 100: " << pl_orig.points[100].timestamp - pl_orig.points[0].timestamp
    //           << msg->header.stamp.toSec() - pl_orig.points[0].timestamp << ", "
    //           << msg->header.stamp.toSec() - pl_orig.points.back().timestamp << std::endl;

    for (int i = 0; i < plsize; i++)
    {
      if (!(std::isfinite(pl_orig.points[i].x) &&
            std::isfinite(pl_orig.points[i].y) &&
            std::isfinite(pl_orig.points[i].z)))
        continue;

      if (i % point_filter_num_ != 0)
        continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                     pl_orig.points[i].z * pl_orig.points[i].z;
      if (range > 150 * 150 || range < blind * blind)
        continue;

      point3D point_temp;
      point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
      point_temp.point = point_temp.raw_point;
      point_temp.relative_time = pl_orig.points[i].timestamp - pl_orig.points[0].timestamp;
      point_temp.intensity = pl_orig.points[i].intensity;

      point_temp.timestamp = headertime + point_temp.relative_time;
      point_temp.alpha_time = point_temp.relative_time / timespan_;
      point_temp.timespan = timespan_;
      point_temp.ring = pl_orig.points[i].ring;

      cloud_out_.push_back(point_temp);
    }
  }

  void CloudConvert::Odin1Handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)
  {
    // Odin1 /odin1/cloud_raw fields:
    // x/y/z float32, intensity uint8, confidence uint16, offset_time float32 (seconds)
    cloud_out_.clear();
    cloud_full_.clear();

    int x_off = -1, y_off = -1, z_off = -1, i_off = -1, c_off = -1, t_off = -1;
    uint8_t i_type = 0, c_type = 0, t_type = 0;
    for (const auto &f : msg->fields)
    {
      if (f.name == "x")
        x_off = static_cast<int>(f.offset);
      else if (f.name == "y")
        y_off = static_cast<int>(f.offset);
      else if (f.name == "z")
        z_off = static_cast<int>(f.offset);
      else if (f.name == "intensity")
      {
        i_off = static_cast<int>(f.offset);
        i_type = f.datatype;
      }
      else if (f.name == "confidence")
      {
        c_off = static_cast<int>(f.offset);
        c_type = f.datatype;
      }
      else if (f.name == "offset_time")
      {
        t_off = static_cast<int>(f.offset);
        t_type = f.datatype;
      }
    }
    if (x_off < 0 || y_off < 0 || z_off < 0)
    {
      LOG(ERROR) << "Odin1 cloud missing xyz fields";
      return;
    }

    const size_t n = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    const uint32_t step = msg->point_step;
    if (step == 0 || msg->data.size() < n * step)
    {
      LOG(ERROR) << "Odin1 cloud size mismatch";
      return;
    }

    cloud_out_.reserve(n / static_cast<size_t>(std::max(1, point_filter_num_)));
    const double headertime = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    const int min_conf = 30; // raw confidence scale is typically hundreds; keep low threshold

    auto read_f32 = [&](const uint8_t *p, int off) -> float {
      float v;
      std::memcpy(&v, p + off, sizeof(float));
      return v;
    };
    auto read_intensity = [&](const uint8_t *p) -> float {
      if (i_off < 0)
        return 0.f;
      if (i_type == sensor_msgs::msg::PointField::FLOAT32)
        return read_f32(p, i_off);
      if (i_type == sensor_msgs::msg::PointField::UINT8)
        return static_cast<float>(p[i_off]);
      if (i_type == sensor_msgs::msg::PointField::UINT16)
      {
        uint16_t v;
        std::memcpy(&v, p + i_off, sizeof(uint16_t));
        return static_cast<float>(v);
      }
      return 0.f;
    };
    auto read_conf = [&](const uint8_t *p) -> int {
      if (c_off < 0)
        return 255;
      if (c_type == sensor_msgs::msg::PointField::UINT16)
      {
        uint16_t v;
        std::memcpy(&v, p + c_off, sizeof(uint16_t));
        return static_cast<int>(v);
      }
      if (c_type == sensor_msgs::msg::PointField::UINT8)
        return static_cast<int>(p[c_off]);
      if (c_type == sensor_msgs::msg::PointField::FLOAT32)
        return static_cast<int>(read_f32(p, c_off));
      return 255;
    };
    auto read_ot = [&](const uint8_t *p) -> double {
      if (t_off < 0)
        return 0.0;
      if (t_type == sensor_msgs::msg::PointField::FLOAT32)
        return static_cast<double>(read_f32(p, t_off));
      if (t_type == sensor_msgs::msg::PointField::FLOAT64)
      {
        double v;
        std::memcpy(&v, p + t_off, sizeof(double));
        return v;
      }
      return 0.0;
    };

    double t_max = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
      if (static_cast<int>(i) % point_filter_num_ != 0)
        continue;
      const uint8_t *p = msg->data.data() + i * step;
      if (read_conf(p) < min_conf)
        continue;

      const float x = read_f32(p, x_off);
      const float y = read_f32(p, y_off);
      const float z = read_f32(p, z_off);
      if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(z)))
        continue;

      const double range2 = static_cast<double>(x) * x + static_cast<double>(y) * y + static_cast<double>(z) * z;
      if (range2 > 150.0 * 150.0 || range2 < blind * blind)
        continue;

      double ot = read_ot(p);
      // tolerate accidental microsecond payloads
      if (ot > 1.0)
        ot *= 1e-6;
      if (ot < 0.0)
        ot = 0.0;
      if (ot > t_max)
        t_max = ot;

      point3D pt;
      pt.raw_point = Eigen::Vector3d(x, y, z);
      pt.point = pt.raw_point;
      pt.relative_time = ot;
      pt.intensity = read_intensity(p);
      pt.timestamp = headertime + ot;
      pt.ring = 0;
      cloud_out_.push_back(pt);
    }

    timespan_ = t_max > 1e-6 ? t_max : 0.01;
    for (auto &pt : cloud_out_)
    {
      pt.timespan = timespan_;
      pt.alpha_time = pt.relative_time / timespan_;
    }
  }

  void CloudConvert::LoadFromYAML(std::shared_ptr<rclcpp::Node> &nh)
  {
    int lidar_type = 1;

    nh->declare_parameter<int>("lidar_type", 1);
    nh->get_parameter("lidar_type", lidar_type);

    nh->declare_parameter<int>("point_filter_num", 1);
    nh->get_parameter("point_filter_num", point_filter_num_);

    nh->declare_parameter<float>("blind", 0.1);
    nh->get_parameter("blind", blind);

    if (lidar_type == 1)
    {
      lidar_type_ = LidarType::AVIA;
      LOG(INFO) << "Using AVIA Lidar";
    }
    else if (lidar_type == 2)
    {
      lidar_type_ = LidarType::VELO32;
      LOG(INFO) << "Using Velodyne 32 Lidar";
    }
    else if (lidar_type == 3)
    {
      lidar_type_ = LidarType::OUST64;
      LOG(INFO) << "Using OUST 64 Lidar";
    }
    else if (lidar_type == 4)
    {
      lidar_type_ = LidarType::ROBOSENSE16;
      LOG(INFO) << "Using Robosense 16 LIdar";
    }
    else if (lidar_type == 5)
    {
      lidar_type_ = LidarType::PANDAR;
      LOG(INFO) << "Using Pandar LIdar";
    }
    else if (lidar_type == 6)
    {
      lidar_type_ = LidarType::ODIN1;
      LOG(INFO) << "Using Odin1 Lidar";
    }
    else
    {
      LOG(WARNING) << "unknown lidar_type";
    }
  }

} // namespace zjloc
