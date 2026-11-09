#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <random>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tuple>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <plan_env/raycast.h>

#define logit(x) (log((x) / (1 - (x))))

using namespace std;

// voxel hashing
template <typename T>
struct matrix_hash : std::unary_function<T, size_t>
{
  std::size_t operator()(T const &matrix) const
  {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i)
    {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

// constant parameters

struct MappingParameters
{

  /* map properties */
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_; // map range in pos
  Eigen::Vector3i map_voxel_num_;                       // map range in index
  Eigen::Vector3i map_origin_idx_, map_bound_min_idx_, map_bound_max_idx_;
  Eigen::Vector3d local_update_range_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  double footprint_radius_;
  double footprint_offset_;
  double footprint_z_down_;
  double footprint_z_up_;
  string frame_id_;
  int pose_type_;
  bool enable_depth_fusion_input_;
  bool transform_cloud_to_frame_;
  bool transform_odom_to_frame_;
  double tf_lookup_timeout_;

  /* camera parameters */
  double cx_, cy_, fx_, fy_;

  /* time out */
  double odom_depth_timeout_;

  /* depth image projection filtering */
  double depth_filter_maxdist_, depth_filter_mindist_, depth_filter_tolerance_;
  int depth_filter_margin_;
  bool use_depth_filter_;
  double k_depth_scaling_factor_;
  int skip_pixel_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_; // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;                  // logit of occupancy probability
  double min_ray_length_, max_ray_length_; // range of doing raycasting
  double odom_z_offset_;
  double pointcloud_vertical_inflation_;
  double pointcloud_vertical_inflation_down_;
  bool map_sliding_en_;
  int map_sliding_thresh_vox_;
  std::string sensor_frame_;
  Eigen::Vector3d sensor_origin_offset_;
  bool project_pointcloud_to_ground_;

  /* local map update and clear */
  int local_map_margin_;

  /* visualization and computation time display */
  double vis_height_, virtual_ceil_height_, ground_height_, virtual_ceil_yp_, virtual_ceil_yn_;
  bool show_occ_time_;

  /* active mapping */
  double unknown_flag_;
};

// intermediate mapping data for fusion

struct MappingData
{
  // main map data, occupancy of each voxel and Euclidean distance

  std::vector<double> occupancy_buffer_;
  std::vector<char> occupancy_buffer_inflate_;
  std::vector<int> occupancy_buffer_inflate_cnt_;

  // camera position and pose data

  Eigen::Vector3d camera_pos_, last_camera_pos_;
  Eigen::Matrix3d camera_r_m_, last_camera_r_m_;
  Eigen::Matrix4d cam2body_;

  // depth image data

  cv::Mat depth_image_, last_depth_image_;
  int image_cnt_;

  // flags of map state

  bool occ_need_update_, local_updated_;
  bool has_first_depth_;
  bool has_odom_, has_cloud_, has_ray_pose_;
  bool use_cloud_update_;
  uint64_t map_revision_;
  Eigen::Vector3d ray_pos_;
  Eigen::Matrix3d ray_r_m_;

  // odom_depth_timeout_
  rclcpp::Time last_occ_update_time_;
  bool flag_depth_odom_timeout_;
  bool flag_use_depth_fusion;

  // depth image projected point cloud

  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;

  // flag buffers for speeding up raycasting

  vector<short> count_hit_, count_hit_and_miss_;
  vector<char> flag_traverse_, flag_rayend_;
  char raycast_num_;
  queue<Eigen::Vector3i> cache_voxel_;

  // range of updating grid

  Eigen::Vector3i local_bound_min_, local_bound_max_;

  // computation time

  double fuse_time_, max_fuse_time_;
  int update_num_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap
{
public:
  enum class FootprintStatus : uint8_t
  {
    FREE = 0,
    OCCUPIED = 1,
    OUT_OF_MAP = 2,
    INVALID_INPUT = 3
  };

  struct FootprintCollisionResult
  {
    FootprintStatus status{FootprintStatus::INVALID_INPUT};
    Eigen::Vector3d sample_position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d cylinder_center{Eigen::Vector3d::Zero()};
    int cylinder_index{-1};
  };

  GridMap() {}
  ~GridMap() {}

  enum
  {
    POSE_STAMPED = 1,
    ODOMETRY = 2,
    INVALID_IDX = -10000
  };

  // occupancy map management
  void resetBuffer();
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

  inline void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
  inline void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
  inline int toAddress(const Eigen::Vector3i &id);
  inline int toAddress(int &x, int &y, int &z);
  inline bool isInMap(const Eigen::Vector3d &pos);
  inline bool isInMap(const Eigen::Vector3i &idx);

  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  inline void setOccupied(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3i id);
  FootprintCollisionResult checkFootprint(const Eigen::Vector3d &body_position,
                                          double body_yaw);
  bool isFootprintFree(const Eigen::Vector3d &body_position, double body_yaw);

  inline void boundIndex(Eigen::Vector3i &id);
  inline bool isUnknown(const Eigen::Vector3i &id);
  inline bool isUnknown(const Eigen::Vector3d &pos);
  inline bool isKnownFree(const Eigen::Vector3i &id);
  inline bool isKnownOccupied(const Eigen::Vector3i &id);

  void initMap(rclcpp::Node::SharedPtr node);

  void publishMap();
  void publishMapInflate(bool all_info = false);

  void publishDepth();

  bool hasDepthObservation();
  bool hasMapObservation() const { return md_.map_revision_ > 0; }
  uint64_t getMapRevision() const { return md_.map_revision_; }
  bool odomValid();
  void getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size);
  inline double getResolution();
  Eigen::Vector3d getOrigin();
  int getVoxelNum();
  bool getOdomDepthTimeout() { return md_.flag_depth_odom_timeout_; }

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;
  std::vector<Eigen::Vector3i> footprint_offsets_;
  std::vector<Eigen::Vector3i> inflation_offsets_;

  // get depth image and camera pose
  void depthPoseCallback(const sensor_msgs::msg::Image::ConstPtr &img,
                         const geometry_msgs::msg::PoseStamped::ConstPtr &pose);
  void extrinsicCallback(const nav_msgs::msg::Odometry::ConstPtr &odom);
  void depthOdomCallback(const sensor_msgs::msg::Image::ConstPtr &img, const nav_msgs::msg::Odometry::ConstPtr &odom);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstPtr &img);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

  // update occupancy by raycasting
  void updateOccupancyCallback();
  void visCallback();

  // main update process
  void projectDepthImage();
  void raycastProcess();
  void clearAndInflateLocalMap();
  void buildFootprintOffsets();
  void buildInflationOffsets();
  void updateSlidingMap(const Eigen::Vector3d &center);
  void updateMapBoundaryFromIndex();
  void resetAllMapData();
  void resetCellByAddress(int address, const std::vector<char> *clear_mask = nullptr);
  void hashAddressToGlobalIndex(int address, Eigen::Vector3i &id) const;
  void applyOccupancyUpdate(const Eigen::Vector3i &id, double log_odds);
  void updateInflation(const Eigen::Vector3i &id, int delta,
                       const std::vector<char> *clear_mask = nullptr);
  void raycastCloudProcess();

  inline void inflatePoint(const Eigen::Vector3i &pt, int step, vector<Eigen::Vector3i> &pts);
  int setCacheOccupancy(Eigen::Vector3d pos, int occ);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt);
  bool lookupTransformSafe(const std::string &target_frame, const std::string &source_frame,
                           const rclcpp::Time &stamp, geometry_msgs::msg::TransformStamped &transform);
  bool getOdomPoseInMap(const nav_msgs::msg::Odometry &odom, Eigen::Vector3d &pos,
                        Eigen::Quaterniond &orientation);
  int getLocalIndex(int id, int dimension) const;
  int toAddressLocal(const Eigen::Vector3i &id) const;

  // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImageOdom; typedef
  // message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // geometry_msgs::PoseStamped> SyncPolicyImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry>
      SyncPolicyImageOdom;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, geometry_msgs::msg::PoseStamped>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> pose_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerImageOdom sync_image_odom_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr indep_cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr indep_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr extrinsic_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_inf_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr occ_timer_;
  rclcpp::TimerBase::SharedPtr vis_timer_;

  //
  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;
};

/* ============================== definition of inline function
 * ============================== */

inline int GridMap::toAddress(const Eigen::Vector3i &id)
{
  return getLocalIndex(id(0), 0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         getLocalIndex(id(1), 1) * mp_.map_voxel_num_(2) + getLocalIndex(id(2), 2);
}

inline int GridMap::toAddress(int &x, int &y, int &z)
{
  return getLocalIndex(x, 0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         getLocalIndex(y, 1) * mp_.map_voxel_num_(2) + getLocalIndex(z, 2);
}

inline void GridMap::boundIndex(Eigen::Vector3i &id)
{
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_.map_bound_max_idx_(0)), mp_.map_bound_min_idx_(0));
  id1(1) = max(min(id(1), mp_.map_bound_max_idx_(1)), mp_.map_bound_min_idx_(1));
  id1(2) = max(min(id(2), mp_.map_bound_max_idx_(2)), mp_.map_bound_min_idx_(2));
  id = id1;
}

inline bool GridMap::isUnknown(const Eigen::Vector3i &id)
{
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

inline bool GridMap::isUnknown(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

inline bool GridMap::isKnownFree(const Eigen::Vector3i &id)
{
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
  //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
  return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

inline bool GridMap::isKnownOccupied(const Eigen::Vector3i &id)
{
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1;
}

inline void GridMap::setOccupied(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  applyOccupancyUpdate(id, mp_.clamp_max_log_);
}

inline void GridMap::setOccupancy(Eigen::Vector3d pos, double occ)
{
  if (occ != 1 && occ != 0)
  {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos))
    return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  applyOccupancyUpdate(id, occ > 0.5 ? mp_.clamp_max_log_ : mp_.clamp_min_log_);
}

inline int GridMap::getOccupancy(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int GridMap::getOccupancy(Eigen::Vector3i id)
{
  if (!isInMap(id))
    return -1;

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline bool GridMap::isInMap(const Eigen::Vector3d &pos)
{
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4)
  {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4)
  {
    return false;
  }
  return true;
}

inline bool GridMap::isInMap(const Eigen::Vector3i &idx)
{
  if (idx(0) < mp_.map_bound_min_idx_(0) || idx(1) < mp_.map_bound_min_idx_(1) ||
      idx(2) < mp_.map_bound_min_idx_(2))
  {
    return false;
  }
  if (idx(0) > mp_.map_bound_max_idx_(0) || idx(1) > mp_.map_bound_max_idx_(1) ||
      idx(2) > mp_.map_bound_max_idx_(2))
  {
    return false;
  }
  return true;
}

inline void GridMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
{
  for (int i = 0; i < 3; ++i)
    id(i) = floor(pos(i) * mp_.resolution_inv_);
}

inline void GridMap::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
{
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * mp_.resolution_;
}

inline void GridMap::inflatePoint(const Eigen::Vector3i &pt, int step, vector<Eigen::Vector3i> &pts)
{
  int num = 0;
  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z)
      {
        pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
      }
}

inline double GridMap::getResolution() { return mp_.resolution_; }

#endif
