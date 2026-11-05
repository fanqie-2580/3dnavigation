#include "preprocess.h"

#include <cmath>
#include <cstring>
#include <limits>

#define RETURN0     0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
{
  lidar_type[0] = 1;
  lidar_type[1] = 1;
  point_filter_num[0] = 3;
  point_filter_num[1] = 3;
  N_SCANS[0] = 6;
  N_SCANS[1] = 6;
  SCAN_RATE[0] = 10;
  SCAN_RATE[1] = 10;
  time_unit[0] = 2;
  time_unit[1] = 2;
  blind[0] = 0.1;
  blind[1] = 0.1;
  conf_min[0] = -1.0;
  conf_min[1] = -1.0;
  given_offset_time = false;
}

Preprocess::~Preprocess() {}

void Preprocess::set()
{
  switch (time_unit[0])
  {
    case SEC:
      time_unit_scale[0] = 1.e3f;
      break;
    case MS:
      time_unit_scale[0] = 1.f;
      break;
    case US:
      time_unit_scale[0] = 1.e-3f;
      break;
    case NS:
      time_unit_scale[0] = 1.e-6f;
      break;
    default:
      time_unit_scale[0] = 1.f;
      break;
  }
  switch (time_unit[1])
  {
    case SEC:
      time_unit_scale[1] = 1.e3f;
      break;
    case MS:
      time_unit_scale[1] = 1.f;
      break;
    case US:
      time_unit_scale[1] = 1.e-3f;
      break;
    case NS:
      time_unit_scale[1] = 1.e-6f;
      break;
    default:
      time_unit_scale[1] = 1.f;
      break;
  }
  return;
}

void Preprocess::process(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out, const int &lidar_num)
{  
  avia_handler(msg, lidar_num);
  *pcl_out = pl_surf;
}

void Preprocess::process(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out, const int &lidar_num)
{
  switch (lidar_type[lidar_num])
  {
  case OUST64:
    oust64_handler(msg, lidar_num);
    break;

  case VELO16:
    velodyne_handler(msg, lidar_num);
    break;

  case XYZ_ONLY:
    xyz_handler(msg, lidar_num);
    break;
  
  default:
    printf("Error LiDAR Type %d\n", lidar_type[lidar_num]);
    break;
  }
  *pcl_out = pl_surf;
}

void Preprocess::avia_handler(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg, const int &lidar_num)
{
  pl_surf.clear();
  pl_full.clear();
  double t1 = omp_get_wtime();
  int plsize = msg->point_num;
  // cout<<"plsie: "<<plsize<<endl;

  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for(int i=0; i<N_SCANS[lidar_num]; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;
  
  for(uint i=1; i<plsize; i++)
  {
    if((msg->points[i].line < N_SCANS[lidar_num]) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
    {
      valid_num ++;
      if (valid_num % point_filter_num[lidar_num] == 0)
      {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms

        if( ((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7) || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
            && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind[lidar_num] * blind[lidar_num])))
        {
          pl_surf.push_back(pl_full[i]);
        }
      }
    }
  }
}

void Preprocess::oust64_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, const int &lidar_num)
{
  pl_surf.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_surf.reserve(plsize);
  double time_stamp = rclcpp::Time(msg->header.stamp).seconds();
  // cout << "===================================" << endl;
  // printf("Pt size = %d, N_SCANS[lidar_num] = %d\r\n", plsize, N_SCANS[lidar_num]);
  for (int i = 0; i < pl_orig.points.size(); i++)
  {
    if (i % point_filter_num[lidar_num] != 0) continue;

    double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
    
    if (range < (blind[lidar_num] * blind[lidar_num])) continue;
    
    Eigen::Vector3d pt_vec;
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.curvature = pl_orig.points[i].t * time_unit_scale[lidar_num]; // curvature unit: ms

    pl_surf.points.push_back(added_pt);
  }
}

void Preprocess::velodyne_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, const int &lidar_num)
{
    pl_surf.clear();
    pl_full.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE[lidar_num];       // scan angular velocity
    std::vector<bool> is_first(N_SCANS[lidar_num],true);
    std::vector<double> yaw_fp(N_SCANS[lidar_num], 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS[lidar_num], 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS[lidar_num], 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0)
    {
      given_offset_time = true;
    }
    else
    {
      given_offset_time = false;
      double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
      double yaw_end  = yaw_first;
      int layer_first = pl_orig.points[0].ring;
      for (uint i = plsize - 1; i > 0; i--)
      {
        if (pl_orig.points[i].ring == layer_first)
        {
          yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
          break;
        }
      }
    }

    for (int i = 0; i < plsize; i++)
    {
      // if ( (abs(pl_orig.points[i].time) < 1.0 / SCAN_RATE[lidar_num] / 1800) || (abs(pl_orig.points[i].time) > (1.0 / SCAN_RATE[lidar_num]) * 1.1) )
      // {
      //   continue;
      // }
      PointType added_pt;
      // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;
      
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time * time_unit_scale[lidar_num];  // curvature unit: ms // cout<<added_pt.curvature<<endl;

      if (!given_offset_time)
      {
        int layer = pl_orig.points[i].ring;
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

        if (is_first[layer])
        {
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
            yaw_fp[layer]=yaw_angle;
            is_first[layer]=false;
            added_pt.curvature = 0.0;
            yaw_last[layer]=yaw_angle;
            time_last[layer]=added_pt.curvature;
            continue;
        }

        // compute offset time
        if (yaw_angle <= yaw_fp[layer])
        {
          added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
        }
        else
        {
          added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
        }

        if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer]=added_pt.curvature;
      }

      if (i % point_filter_num[lidar_num] == 0)
      {
        if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind[lidar_num] * blind[lidar_num]))
        {
          pl_surf.points.push_back(added_pt);
        }
      }
    }
}

void Preprocess::xyz_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, const int &lidar_num)
{
  // Odin1 / generic PointCloud2: keep per-point offset_time (seconds) as curvature (ms).
  pl_surf.clear();
  pl_full.clear();

  int off_x = -1, off_y = -1, off_z = -1, off_i = -1, off_c = -1, off_t = -1;
  int type_i = -1, type_c = -1, type_t = -1;
  for (const auto &f : msg->fields)
  {
    if (f.name == "x") off_x = static_cast<int>(f.offset);
    else if (f.name == "y") off_y = static_cast<int>(f.offset);
    else if (f.name == "z") off_z = static_cast<int>(f.offset);
    else if (f.name == "intensity") { off_i = static_cast<int>(f.offset); type_i = f.datatype; }
    else if (f.name == "confidence") { off_c = static_cast<int>(f.offset); type_c = f.datatype; }
    else if (f.name == "offset_time") { off_t = static_cast<int>(f.offset); type_t = f.datatype; }
  }
  if (off_x < 0 || off_y < 0 || off_z < 0 || msg->point_step == 0) return;

  const int npts = static_cast<int>(msg->width) * static_cast<int>(msg->height);
  if (npts <= 0) return;
  pl_surf.reserve(npts / std::max(1, point_filter_num[lidar_num]) + 1);

  given_offset_time = (off_t >= 0);
  const float blind2 = static_cast<float>(blind[lidar_num] * blind[lidar_num]);
  const bool use_conf = (off_c >= 0 && conf_min[lidar_num] > 0.0);
  const float conf_thr = static_cast<float>(conf_min[lidar_num]);

  auto read_f32 = [](const uint8_t *p) {
    float v;
    std::memcpy(&v, p, sizeof(float));
    return v;
  };
  auto read_u16 = [](const uint8_t *p) {
    uint16_t v;
    std::memcpy(&v, p, sizeof(uint16_t));
    return v;
  };
  auto read_u8 = [](const uint8_t *p) { return static_cast<float>(*p); };

  for (int i = 0; i < npts; i++)
  {
    if (point_filter_num[lidar_num] > 1 && (i % point_filter_num[lidar_num]) != 0) continue;
    const uint8_t *base = &msg->data[static_cast<size_t>(i) * msg->point_step];
    const float x = read_f32(base + off_x);
    const float y = read_f32(base + off_y);
    const float z = read_f32(base + off_z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    const float r2 = x * x + y * y + z * z;
    if (r2 <= blind2) continue;

    if (use_conf)
    {
      float conf = 0.f;
      if (type_c == sensor_msgs::msg::PointField::UINT16)
        conf = static_cast<float>(read_u16(base + off_c));
      else if (type_c == sensor_msgs::msg::PointField::FLOAT32)
        conf = read_f32(base + off_c);
      else if (type_c == sensor_msgs::msg::PointField::UINT8)
        conf = read_u8(base + off_c);
      if (conf < conf_thr) continue;
    }

    float intensity = 0.f;
    if (off_i >= 0)
    {
      if (type_i == sensor_msgs::msg::PointField::FLOAT32)
        intensity = read_f32(base + off_i);
      else if (type_i == sensor_msgs::msg::PointField::UINT8)
        intensity = read_u8(base + off_i);
      else if (type_i == sensor_msgs::msg::PointField::UINT16)
        intensity = static_cast<float>(read_u16(base + off_i));
    }

    // Odin offset_time is in seconds relative to header.stamp → curvature in ms.
    float curvature_ms = 0.f;
    if (off_t >= 0)
    {
      float t_s = 0.f;
      if (type_t == sensor_msgs::msg::PointField::FLOAT32)
        t_s = read_f32(base + off_t);
      else if (type_t == sensor_msgs::msg::PointField::FLOAT64)
      {
        double td;
        std::memcpy(&td, base + off_t, sizeof(double));
        t_s = static_cast<float>(td);
      }
      if (std::isfinite(t_s) && t_s >= 0.f)
        curvature_ms = t_s * 1000.f;
    }

    PointType added_pt;
    added_pt.x = x;
    added_pt.y = y;
    added_pt.z = z;
    added_pt.intensity = intensity;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.curvature = curvature_ms;
    pl_surf.points.push_back(added_pt);
  }
}