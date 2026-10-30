#include "plan_env/grid_map.h"

#include <pcl/common/transforms.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <stdexcept>

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

void GridMap::initMap(rclcpp::Node::SharedPtr node)
{
  node_ = node;

  /* get parameter */
  double x_size, y_size, z_size;
  node_->declare_parameter("grid_map/resolution", -1.0);
  node_->declare_parameter("grid_map/map_size_x", -1.0);
  node_->declare_parameter("grid_map/map_size_y", -1.0);
  node_->declare_parameter("grid_map/map_size_z", -1.0);
  node_->declare_parameter("grid_map/local_update_range_x", -1.0);
  node_->declare_parameter("grid_map/local_update_range_y", -1.0);
  node_->declare_parameter("grid_map/local_update_range_z", -1.0);
  node_->declare_parameter("grid_map/obstacles_inflation", -1.0);
  node_->declare_parameter("grid_map/footprint_radius", -1.0);
  node_->declare_parameter("grid_map/footprint_offset", -1.0);
  node_->declare_parameter("grid_map/footprint_z_down", -1.0);
  node_->declare_parameter("grid_map/footprint_z_up", -1.0);
  node_->declare_parameter("grid_map/fx", -1.0);
  node_->declare_parameter("grid_map/fy", -1.0);
  node_->declare_parameter("grid_map/cx", -1.0);
  node_->declare_parameter("grid_map/cy", -1.0);
  node_->declare_parameter("grid_map/use_depth_filter", true);
  node_->declare_parameter("grid_map/depth_filter_tolerance", -1.0);
  node_->declare_parameter("grid_map/depth_filter_maxdist", -1.0);
  node_->declare_parameter("grid_map/depth_filter_mindist", -1.0);
  node_->declare_parameter("grid_map/depth_filter_margin", -1);
  node_->declare_parameter("grid_map/k_depth_scaling_factor", -1.0);
  node_->declare_parameter("grid_map/skip_pixel", -1);
  node_->declare_parameter("grid_map/p_hit", 0.70);
  node_->declare_parameter("grid_map/p_miss", 0.35);
  node_->declare_parameter("grid_map/p_min", 0.12);
  node_->declare_parameter("grid_map/p_max", 0.97);
  node_->declare_parameter("grid_map/p_occ", 0.80);
  node_->declare_parameter("grid_map/min_ray_length", -0.1);
  node_->declare_parameter("grid_map/max_ray_length", -0.1);
  node_->declare_parameter("grid_map/odom_z_offset", 0.0);
  node_->declare_parameter("grid_map/pointcloud_vertical_inflation", 0.1);
  node_->declare_parameter("grid_map/pointcloud_vertical_inflation_down", 0.4);
  node_->declare_parameter("grid_map/map_sliding_en", true);
  node_->declare_parameter("grid_map/map_sliding_thresh_vox", 4);
  node_->declare_parameter("grid_map/sensor_frame", std::string(""));
  node_->declare_parameter("grid_map/sensor_origin_offset_x", 0.0);
  node_->declare_parameter("grid_map/sensor_origin_offset_y", 0.0);
  node_->declare_parameter("grid_map/sensor_origin_offset_z", 0.0);
  node_->declare_parameter("grid_map/project_pointcloud_to_ground", false);
  node_->declare_parameter("grid_map/vis_height", 0.3);
  node_->declare_parameter("grid_map/virtual_ceil_height", -0.1);
  node_->declare_parameter("grid_map/virtual_ceil_yp", -0.1);
  node_->declare_parameter("grid_map/virtual_ceil_yn", -0.1);
  node_->declare_parameter("grid_map/show_occ_time", false);
  node_->declare_parameter("grid_map/pose_type", 1);
  node_->declare_parameter("grid_map/enable_depth_fusion_input", true);
  node_->declare_parameter("grid_map/frame_id", "world");
  node_->declare_parameter("grid_map/transform_cloud_to_frame", true);
  node_->declare_parameter("grid_map/transform_odom_to_frame", true);
  node_->declare_parameter("grid_map/tf_lookup_timeout", 0.1);
  node_->declare_parameter("grid_map/local_map_margin", 1);
  node_->declare_parameter("grid_map/ground_height", 0.0);
  node_->declare_parameter("grid_map/odom_depth_timeout", 1.0);

  node_->get_parameter("grid_map/resolution", mp_.resolution_);
  node_->get_parameter("grid_map/map_size_x", x_size);
  node_->get_parameter("grid_map/map_size_y", y_size);
  node_->get_parameter("grid_map/map_size_z", z_size);
  node_->get_parameter("grid_map/local_update_range_x", mp_.local_update_range_(0));
  node_->get_parameter("grid_map/local_update_range_y", mp_.local_update_range_(1));
  node_->get_parameter("grid_map/local_update_range_z", mp_.local_update_range_(2));
  node_->get_parameter("grid_map/obstacles_inflation", mp_.obstacles_inflation_);
  node_->get_parameter("grid_map/footprint_radius", mp_.footprint_radius_);
  node_->get_parameter("grid_map/footprint_offset", mp_.footprint_offset_);
  node_->get_parameter("grid_map/footprint_z_down", mp_.footprint_z_down_);
  node_->get_parameter("grid_map/footprint_z_up", mp_.footprint_z_up_);
  node_->get_parameter("grid_map/fx", mp_.fx_);
  node_->get_parameter("grid_map/fy", mp_.fy_);
  node_->get_parameter("grid_map/cx", mp_.cx_);
  node_->get_parameter("grid_map/cy", mp_.cy_);
  node_->get_parameter("grid_map/use_depth_filter", mp_.use_depth_filter_);
  node_->get_parameter("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_);
  node_->get_parameter("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_);
  node_->get_parameter("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_);
  node_->get_parameter("grid_map/depth_filter_margin", mp_.depth_filter_margin_);
  node_->get_parameter("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_);
  node_->get_parameter("grid_map/skip_pixel", mp_.skip_pixel_);
  node_->get_parameter("grid_map/p_hit", mp_.p_hit_);
  node_->get_parameter("grid_map/p_miss", mp_.p_miss_);
  node_->get_parameter("grid_map/p_min", mp_.p_min_);
  node_->get_parameter("grid_map/p_max", mp_.p_max_);
  node_->get_parameter("grid_map/p_occ", mp_.p_occ_);
  node_->get_parameter("grid_map/min_ray_length", mp_.min_ray_length_);
  node_->get_parameter("grid_map/max_ray_length", mp_.max_ray_length_);
  node_->get_parameter("grid_map/odom_z_offset", mp_.odom_z_offset_);
  node_->get_parameter("grid_map/pointcloud_vertical_inflation", mp_.pointcloud_vertical_inflation_);
  node_->get_parameter("grid_map/pointcloud_vertical_inflation_down", mp_.pointcloud_vertical_inflation_down_);
  node_->get_parameter("grid_map/map_sliding_en", mp_.map_sliding_en_);
  node_->get_parameter("grid_map/map_sliding_thresh_vox", mp_.map_sliding_thresh_vox_);
  node_->get_parameter("grid_map/sensor_frame", mp_.sensor_frame_);
  node_->get_parameter("grid_map/sensor_origin_offset_x", mp_.sensor_origin_offset_(0));
  node_->get_parameter("grid_map/sensor_origin_offset_y", mp_.sensor_origin_offset_(1));
  node_->get_parameter("grid_map/sensor_origin_offset_z", mp_.sensor_origin_offset_(2));
  node_->get_parameter("grid_map/project_pointcloud_to_ground", mp_.project_pointcloud_to_ground_);
  node_->get_parameter("grid_map/vis_height", mp_.vis_height_);
  node_->get_parameter("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_);
  node_->get_parameter("grid_map/virtual_ceil_yp", mp_.virtual_ceil_yp_);
  node_->get_parameter("grid_map/virtual_ceil_yn", mp_.virtual_ceil_yn_);
  node_->get_parameter("grid_map/show_occ_time", mp_.show_occ_time_);
  node_->get_parameter("grid_map/pose_type", mp_.pose_type_);
  node_->get_parameter("grid_map/enable_depth_fusion_input", mp_.enable_depth_fusion_input_);
  node_->get_parameter("grid_map/frame_id", mp_.frame_id_);
  node_->get_parameter("grid_map/transform_cloud_to_frame", mp_.transform_cloud_to_frame_);
  node_->get_parameter("grid_map/transform_odom_to_frame", mp_.transform_odom_to_frame_);
  node_->get_parameter("grid_map/tf_lookup_timeout", mp_.tf_lookup_timeout_);
  node_->get_parameter("grid_map/local_map_margin", mp_.local_map_margin_);
  node_->get_parameter("grid_map/ground_height", mp_.ground_height_);
  node_->get_parameter("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_);

  if (!std::isfinite(mp_.resolution_) || mp_.resolution_ <= 0.0 ||
      !std::isfinite(mp_.obstacles_inflation_) || mp_.obstacles_inflation_ < 0.0 ||
      !std::isfinite(mp_.pointcloud_vertical_inflation_) || mp_.pointcloud_vertical_inflation_ < 0.0 ||
      !std::isfinite(mp_.pointcloud_vertical_inflation_down_) || mp_.pointcloud_vertical_inflation_down_ < 0.0 ||
      !std::isfinite(mp_.footprint_radius_) || mp_.footprint_radius_ <= 0.0 ||
      !std::isfinite(mp_.footprint_offset_) || mp_.footprint_offset_ < 0.0 ||
      !std::isfinite(mp_.footprint_z_down_) || mp_.footprint_z_down_ < 0.0 ||
      !std::isfinite(mp_.footprint_z_up_) || mp_.footprint_z_up_ <= 0.0 ||
      !std::isfinite(mp_.min_ray_length_) || mp_.min_ray_length_ < 0.0 ||
      !std::isfinite(mp_.max_ray_length_) || mp_.max_ray_length_ <= mp_.min_ray_length_)
  {
    throw std::runtime_error(
        "grid_map footprint parameters must be finite and explicitly positive "
        "(offset and obstacle inflation may be zero)");
  }
  if (mp_.map_sliding_thresh_vox_ <= 0)
    throw std::runtime_error("grid_map/map_sliding_thresh_vox must be positive");
  if (mp_.sensor_frame_.empty())
    throw std::runtime_error("grid_map/sensor_frame must name the physical ray origin frame");
  if (!mp_.sensor_origin_offset_.allFinite())
    throw std::runtime_error("grid_map/sensor_origin_offset must be finite");
  buildFootprintOffsets();
  buildInflationOffsets();

  if (mp_.virtual_ceil_height_ - mp_.ground_height_ > z_size)
  {
    mp_.virtual_ceil_height_ = mp_.ground_height_ + z_size;
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_origin_idx_ = Eigen::Vector3i(
      0, 0, static_cast<int>(std::floor((mp_.ground_height_ + 0.5 * mp_.map_size_(2)) * mp_.resolution_inv_)));
  updateMapBoundaryFromIndex();

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  md_.occupancy_buffer_inflate_cnt_ = vector<int>(buffer_size, 0);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  /* init callback */

  // 初始化 message_filters::Subscriber
  if (mp_.enable_depth_fusion_input_)
  {
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        node_, "grid_map/depth", rclcpp::QoS(50).get_rmw_qos_profile());

    extrinsic_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/vins_estimator/extrinsic", 10,
        std::bind(&GridMap::extrinsicCallback, this, std::placeholders::_1));

    if (mp_.pose_type_ == POSE_STAMPED)
    {
      pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(
          node_, "grid_map/pose", rclcpp::QoS(25).get_rmw_qos_profile());

      sync_image_pose_ = std::make_shared<message_filters::Synchronizer<SyncPolicyImagePose>>(
          SyncPolicyImagePose(100), *depth_sub_, *pose_sub_);
      sync_image_pose_->registerCallback(
          std::bind(&GridMap::depthPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
    }
    else if (mp_.pose_type_ == ODOMETRY)
    {
      odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
          node_, "grid_map/odom", rclcpp::QoS(100).get_rmw_qos_profile());

      sync_image_odom_ = std::make_shared<message_filters::Synchronizer<SyncPolicyImageOdom>>(
          SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_);
      sync_image_odom_->registerCallback(
          std::bind(&GridMap::depthOdomCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

    RCLCPP_INFO(node_->get_logger(), "Grid map depth-fusion input enabled.");
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "Grid map running in pointcloud-only mode.");
  }

  // 使用独立的里程计和点云订阅
  indep_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "grid_map/cloud", 10, std::bind(&GridMap::cloudCallback, this, std::placeholders::_1));

  indep_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "grid_map/odom", 10, std::bind(&GridMap::odomCallback, this, std::placeholders::_1));

  // 定时器
  occ_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(0.05),
      std::bind(&GridMap::updateOccupancyCallback, this));

  vis_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(0.05),
      std::bind(&GridMap::visCallback, this));

  // 发布者
  map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy_inflate", 10);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.has_ray_pose_ = false;
  md_.use_cloud_update_ = false;
  md_.map_revision_ = 0;
  md_.ray_pos_.setZero();
  md_.ray_r_m_.setIdentity();
  md_.image_cnt_ = 0;
  md_.last_occ_update_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;

  md_.flag_depth_odom_timeout_ = false;
  md_.flag_use_depth_fusion = false;

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

GridMap::FootprintCollisionResult GridMap::checkFootprint(
    const Eigen::Vector3d &body_position, double body_yaw)
{
  FootprintCollisionResult result;
  result.sample_position = body_position;

  if (!body_position.allFinite() || !std::isfinite(body_yaw))
  {
    result.status = FootprintStatus::INVALID_INPUT;
    return result;
  }

  const Eigen::Vector3d heading(std::cos(body_yaw), std::sin(body_yaw), 0.0);
  const Eigen::Vector3d centers[2] = {
      body_position + mp_.footprint_offset_ * heading,
      body_position - mp_.footprint_offset_ * heading};
  for (int cylinder = 0; cylinder < 2; ++cylinder)
  {
    const Eigen::Vector3d &sample = centers[cylinder];
    if (!isInMap(sample))
    {
      result.status = FootprintStatus::OUT_OF_MAP;
      result.sample_position = sample;
      result.cylinder_center = centers[cylinder];
      result.cylinder_index = cylinder;
      return result;
    }

    Eigen::Vector3i index;
    posToIndex(sample, index);
    if (md_.occupancy_buffer_inflate_[toAddress(index)] != 0)
    {
      result.status = FootprintStatus::OCCUPIED;
      result.sample_position = sample;
      result.cylinder_center = centers[cylinder];
      result.cylinder_index = cylinder;
      return result;
    }
  }

  result.status = FootprintStatus::FREE;
  result.cylinder_index = -1;
  return result;
}

bool GridMap::isFootprintFree(const Eigen::Vector3d &body_position, double body_yaw)
{
  return checkFootprint(body_position, body_yaw).status == FootprintStatus::FREE;
}

void GridMap::buildFootprintOffsets()
{
  footprint_offsets_.clear();
  const int radius_steps = static_cast<int>(std::ceil(mp_.footprint_radius_ / mp_.resolution_));
  const int down_steps = static_cast<int>(std::ceil(mp_.footprint_z_down_ / mp_.resolution_));
  const int up_steps = static_cast<int>(std::ceil(mp_.footprint_z_up_ / mp_.resolution_));
  const double radius_limit = mp_.footprint_radius_ + 0.5 * mp_.resolution_;

  for (int dx = -radius_steps; dx <= radius_steps; ++dx)
    for (int dy = -radius_steps; dy <= radius_steps; ++dy)
    {
      const Eigen::Vector2d radial_offset(dx * mp_.resolution_, dy * mp_.resolution_);
      if (radial_offset.norm() > radius_limit)
        continue;
      for (int dz = -down_steps; dz <= up_steps; ++dz)
        footprint_offsets_.emplace_back(dx, dy, dz);
    }

  if (footprint_offsets_.empty())
    throw std::runtime_error("Go2 footprint sampling produced no collision-check offsets");
}

void GridMap::buildInflationOffsets()
{
  inflation_offsets_.clear();
  const int xy_steps = static_cast<int>(std::ceil(mp_.obstacles_inflation_ / mp_.resolution_));
  const int z_up_steps = static_cast<int>(std::ceil(mp_.pointcloud_vertical_inflation_ / mp_.resolution_));
  const int z_down_steps = static_cast<int>(std::ceil(mp_.pointcloud_vertical_inflation_down_ / mp_.resolution_));
  for (int dx = -xy_steps; dx <= xy_steps; ++dx)
    for (int dy = -xy_steps; dy <= xy_steps; ++dy)
    {
      if (std::hypot(dx * mp_.resolution_, dy * mp_.resolution_) >=
          mp_.obstacles_inflation_)
        continue;
      for (int dz = -z_down_steps; dz <= z_up_steps; ++dz)
        inflation_offsets_.emplace_back(dx, dy, dz);
    }

  if (inflation_offsets_.empty())
    throw std::runtime_error("grid_map obstacle inflation produced no offsets");
}

int GridMap::getLocalIndex(int id, int dimension) const
{
  int local = id % mp_.map_voxel_num_(dimension);
  if (local < 0)
    local += mp_.map_voxel_num_(dimension);
  return local;
}

int GridMap::toAddressLocal(const Eigen::Vector3i &id) const
{
  return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         id(1) * mp_.map_voxel_num_(2) + id(2);
}

void GridMap::updateMapBoundaryFromIndex()
{
  mp_.map_bound_min_idx_ = mp_.map_origin_idx_ - mp_.map_voxel_num_ / 2;
  mp_.map_bound_max_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  mp_.map_min_boundary_ = mp_.map_bound_min_idx_.cast<double>() * mp_.resolution_;
  mp_.map_max_boundary_ = (mp_.map_bound_max_idx_.cast<double>() + Eigen::Vector3d::Ones()) * mp_.resolution_;
  mp_.map_origin_ = mp_.map_min_boundary_;
}

void GridMap::resetAllMapData()
{
  std::fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(),
            mp_.clamp_min_log_ - mp_.unknown_flag_);
  std::fill(md_.occupancy_buffer_inflate_.begin(), md_.occupancy_buffer_inflate_.end(), 0);
  std::fill(md_.occupancy_buffer_inflate_cnt_.begin(), md_.occupancy_buffer_inflate_cnt_.end(), 0);
  std::fill(md_.count_hit_.begin(), md_.count_hit_.end(), 0);
  std::fill(md_.count_hit_and_miss_.begin(), md_.count_hit_and_miss_.end(), 0);
  std::fill(md_.flag_rayend_.begin(), md_.flag_rayend_.end(), -1);
  std::fill(md_.flag_traverse_.begin(), md_.flag_traverse_.end(), -1);
  std::queue<Eigen::Vector3i> empty;
  std::swap(md_.cache_voxel_, empty);
}

void GridMap::hashAddressToGlobalIndex(int address, Eigen::Vector3i &id) const
{
  Eigen::Vector3i local;
  local(0) = address / (mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2));
  local(1) = (address - local(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) /
             mp_.map_voxel_num_(2);
  local(2) = address - local(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) -
             local(1) * mp_.map_voxel_num_(2);
  for (int dimension = 0; dimension < 3; ++dimension)
  {
    int distance = local(dimension) - getLocalIndex(mp_.map_bound_min_idx_(dimension), dimension);
    if (distance < 0)
      distance += mp_.map_voxel_num_(dimension);
    id(dimension) = mp_.map_bound_min_idx_(dimension) + distance;
  }
}

void GridMap::updateInflation(const Eigen::Vector3i &id, int delta,
                              const std::vector<char> *clear_mask)
{
  for (const Eigen::Vector3i &offset : inflation_offsets_)
  {
    const Eigen::Vector3i inflated = id + offset;
    if (!isInMap(inflated))
      continue;
    const int address = toAddress(inflated);
    if (clear_mask != nullptr && (*clear_mask)[address] != 0)
      continue;
    md_.occupancy_buffer_inflate_cnt_[address] += delta;
    if (md_.occupancy_buffer_inflate_cnt_[address] < 0)
      md_.occupancy_buffer_inflate_cnt_[address] = 0;
    md_.occupancy_buffer_inflate_[address] =
        md_.occupancy_buffer_inflate_cnt_[address] > 0 ? 1 : 0;
  }
}

void GridMap::applyOccupancyUpdate(const Eigen::Vector3i &id, double log_odds)
{
  if (!isInMap(id) || !std::isfinite(log_odds))
    return;
  const int address = toAddress(id);
  const bool was_occupied = md_.occupancy_buffer_[address] > mp_.min_occupancy_log_;
  const double clamped = std::clamp(log_odds, mp_.clamp_min_log_, mp_.clamp_max_log_);
  const bool is_occupied = clamped > mp_.min_occupancy_log_;
  md_.occupancy_buffer_[address] = clamped;
  if (was_occupied != is_occupied)
    updateInflation(id, is_occupied ? 1 : -1);
}

void GridMap::resetCellByAddress(int address, const std::vector<char> *clear_mask)
{
  Eigen::Vector3i id;
  hashAddressToGlobalIndex(address, id);
  if (md_.occupancy_buffer_[address] > mp_.min_occupancy_log_)
    updateInflation(id, -1, clear_mask);
  md_.occupancy_buffer_[address] = mp_.clamp_min_log_ - mp_.unknown_flag_;
  md_.occupancy_buffer_inflate_[address] = 0;
  md_.occupancy_buffer_inflate_cnt_[address] = 0;
  md_.count_hit_[address] = 0;
  md_.count_hit_and_miss_[address] = 0;
  md_.flag_rayend_[address] = -1;
  md_.flag_traverse_[address] = -1;
}

void GridMap::updateSlidingMap(const Eigen::Vector3d &center)
{
  if (!mp_.map_sliding_en_ || !center.allFinite())
    return;

  Eigen::Vector3i new_origin;
  posToIndex(center, new_origin);
  const Eigen::Vector3i shift = new_origin - mp_.map_origin_idx_;
  if (shift.cwiseAbs().maxCoeff() < mp_.map_sliding_thresh_vox_)
    return;

  if ((shift.cwiseAbs().array() >= mp_.map_voxel_num_.array()).any())
  {
    resetAllMapData();
    mp_.map_origin_idx_ = new_origin;
    updateMapBoundaryFromIndex();
    md_.local_bound_min_ = mp_.map_bound_min_idx_;
    md_.local_bound_max_ = mp_.map_bound_max_idx_;
    return;
  }

  const Eigen::Vector3i old_min = mp_.map_bound_min_idx_;
  const Eigen::Vector3i old_max = mp_.map_bound_max_idx_;
  const int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
  std::vector<char> clear_mask(buffer_size, 0);
  std::vector<int> clear_addresses;
  clear_addresses.reserve(buffer_size / 8);

  auto markForClear = [&](const Eigen::Vector3i &global_id)
  {
    const int address = toAddress(global_id);
    if (clear_mask[address] == 0)
    {
      clear_mask[address] = 1;
      clear_addresses.push_back(address);
    }
  };

  for (int dimension = 0; dimension < 3; ++dimension)
  {
    const int delta = shift(dimension);
    if (delta == 0)
      continue;
    const int first = (dimension + 1) % 3;
    const int second = (dimension + 2) % 3;
    const int slab_count = std::abs(delta);
    for (int layer = 0; layer < slab_count; ++layer)
    {
      const int slab_index = delta > 0 ? old_min(dimension) + layer
                                       : old_max(dimension) - layer;
      for (int first_index = old_min(first); first_index <= old_max(first); ++first_index)
        for (int second_index = old_min(second); second_index <= old_max(second); ++second_index)
        {
          Eigen::Vector3i global_id;
          global_id(dimension) = slab_index;
          global_id(first) = first_index;
          global_id(second) = second_index;
          markForClear(global_id);
        }
    }
  }

  for (const int address : clear_addresses)
    resetCellByAddress(address, &clear_mask);

  mp_.map_origin_idx_ = new_origin;
  updateMapBoundaryFromIndex();
  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;
}

void GridMap::resetBuffer()
{
  resetAllMapData();
  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* Reset raw occupancy through the ownership-aware path so inflation counts
   * remain consistent with the raw layer. */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        applyOccupancyUpdate(Eigen::Vector3i(x, y, z), mp_.clamp_min_log_);
      }
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;
  int skip_pix = mp_.skip_pixel_;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_r_m_;

  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v += skip_pix)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u += skip_pix)
      {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240)
          std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else
  {

    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_r_m_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
      {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_)
        {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          if (*row_ptr == 0)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }
          else if (depth < mp_.depth_filter_mindist_)
          {
            continue;
          }
          else if (depth > mp_.depth_filter_maxdist_)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }

          // project to world frame
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;
          // if (!isInMap(pt_world)) {
          //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
          // }

          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          // check consistency with last image, disabled...
          if (false)
          {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
            {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_)
              {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            }
            else
            {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* maintain camera pose for consistency check */

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_r_m_ = md_.camera_r_m_;
  md_.last_depth_image_ = md_.depth_image_;
}

void GridMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return;

  rclcpp::Time t1, t2;

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true;

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }

    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }
}

void GridMap::raycastCloudProcess()
{
  if (!md_.use_cloud_update_ || md_.proj_points_cnt <= 0 || !md_.ray_pos_.allFinite() ||
      !isInMap(md_.ray_pos_))
    return;

  ++md_.raycast_num_;
  RayCaster raycaster;
  const Eigen::Vector3d half(0.5, 0.5, 0.5);
  Eigen::Vector3d min_point = md_.ray_pos_;
  Eigen::Vector3d max_point = md_.ray_pos_;

  for (int point_id = 0; point_id < md_.proj_points_cnt; ++point_id)
  {
    const Eigen::Vector3d measured_endpoint = md_.proj_points_[point_id];
    if (!measured_endpoint.allFinite())
      continue;

    Eigen::Vector3d endpoint = measured_endpoint;
    bool endpoint_is_hit = true;
    if (!isInMap(endpoint))
    {
      endpoint = closetPointInMap(endpoint, md_.ray_pos_);
      endpoint_is_hit = false;
    }
    const Eigen::Vector3d ray = endpoint - md_.ray_pos_;
    const double ray_length = ray.norm();
    if (ray_length > mp_.max_ray_length_)
    {
      endpoint = md_.ray_pos_ + ray * (mp_.max_ray_length_ / ray_length);
      endpoint_is_hit = false;
    }
    if (!isInMap(endpoint))
      continue;

    const int endpoint_address = setCacheOccupancy(endpoint, endpoint_is_hit ? 1 : 0);
    if (endpoint_address == INVALID_IDX)
      continue;
    if (md_.flag_rayend_[endpoint_address] == md_.raycast_num_)
      continue;
    md_.flag_rayend_[endpoint_address] = md_.raycast_num_;

    min_point = min_point.cwiseMin(endpoint);
    max_point = max_point.cwiseMax(endpoint);
    raycaster.setInput(endpoint * mp_.resolution_inv_, md_.ray_pos_ * mp_.resolution_inv_);

    Eigen::Vector3d ray_index;
    while (raycaster.step(ray_index))
    {
      const Eigen::Vector3d traversed = (ray_index + half) * mp_.resolution_;
      if (!isInMap(traversed))
        break;
      const int traversed_address = setCacheOccupancy(traversed, 0);
      if (traversed_address == INVALID_IDX)
        continue;
      if (md_.flag_traverse_[traversed_address] == md_.raycast_num_)
        break;
      md_.flag_traverse_[traversed_address] = md_.raycast_num_;
    }
  }

  bool applied_update = false;
  while (!md_.cache_voxel_.empty())
  {
    const Eigen::Vector3i id = md_.cache_voxel_.front();
    md_.cache_voxel_.pop();
    const int address = toAddress(id);
    const int hit_count = md_.count_hit_[address];
    const int observation_count = md_.count_hit_and_miss_[address];
    md_.count_hit_[address] = 0;
    md_.count_hit_and_miss_[address] = 0;
    if (observation_count <= 0)
      continue;

    const double update = hit_count >= observation_count - hit_count ? mp_.prob_hit_log_ : mp_.prob_miss_log_;
    applyOccupancyUpdate(id, md_.occupancy_buffer_[address] + update);
    applied_update = true;
  }

  if (!applied_update)
    return;

  posToIndex(min_point, md_.local_bound_min_);
  posToIndex(max_point, md_.local_bound_max_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);
  md_.local_updated_ = true;
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void GridMap::clearAndInflateLocalMap()
{
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // clear data outside the local range

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    {

      for (int z = min_cut_m(2); z < min_cut(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    {

      for (int y = min_cut_m(1); y < min_cut(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    {

      for (int x = min_cut_m(0); x < min_cut(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  // inflate occupied voxels to compensate robot size

  // clear outdated data
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_)
        {
          const Eigen::Vector3i occupied(x, y, z);
          for (const Eigen::Vector3i &offset : inflation_offsets_)
          {
            const Eigen::Vector3i inflated = occupied + offset;
            if (!isInMap(inflated))
              continue;
            md_.occupancy_buffer_inflate_[toAddress(inflated)] = 1;
          }
        }
      }

  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.5)
  {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_) - 1;
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
  }
}

void GridMap::visCallback()
{
  publishMapInflate(true);
  publishMap();
}

void GridMap::updateOccupancyCallback()
{
  if (md_.last_occ_update_time_.seconds() < 1.0)
    md_.last_occ_update_time_ = node_->now();

  if (!md_.occ_need_update_)
  {
    if ((md_.flag_use_depth_fusion || md_.has_cloud_) &&
        (node_->now() - md_.last_occ_update_time_).seconds() > mp_.odom_depth_timeout_)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "local sensing update lost! now=%f, last_occ_update_time=%f, timeout=%f",
                   node_->now().seconds(),
                   md_.last_occ_update_time_.seconds(),
                   mp_.odom_depth_timeout_);
      md_.flag_depth_odom_timeout_ = true;
    }
    return;
  }
  md_.last_occ_update_time_ = node_->now();
  md_.flag_depth_odom_timeout_ = false;

  if (md_.use_cloud_update_)
  {
    raycastCloudProcess();
    if (md_.local_updated_)
    {
      md_.has_cloud_ = true;
      ++md_.map_revision_;
    }
    md_.occ_need_update_ = false;
    md_.local_updated_ = false;
    md_.use_cloud_update_ = false;
    return;
  }

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  projectDepthImage();
  // t2 = ros::Time::now();
  raycastProcess();
  // t3 = ros::Time::now();

  const bool fused_local_map = md_.local_updated_;
  if (fused_local_map)
    clearAndInflateLocalMap();

  if (fused_local_map && md_.has_first_depth_)
    ++md_.map_revision_;

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
}

void GridMap::depthPoseCallback(const sensor_msgs::msg::Image::ConstPtr &img,
                                const geometry_msgs::msg::PoseStamped::ConstPtr &pose)
{
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  Eigen::Quaterniond camera_q(pose->pose.orientation.w, pose->pose.orientation.x,
                              pose->pose.orientation.y, pose->pose.orientation.z);
  if (!md_.camera_pos_.allFinite() || !camera_q.coeffs().allFinite() || camera_q.norm() < 1e-6)
  {
    md_.occ_need_update_ = false;
    return;
  }
  camera_q.normalize();
  md_.camera_r_m_ = camera_q.toRotationMatrix();
  md_.ray_pos_ = md_.camera_pos_;
  md_.ray_r_m_ = md_.camera_r_m_;
  updateSlidingMap(md_.ray_pos_);
  if (isInMap(md_.camera_pos_))
  {
    md_.has_odom_ = true;
    md_.has_ray_pose_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }

  md_.flag_use_depth_fusion = true;
}

void GridMap::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  if (md_.has_first_depth_)
    return;

  Eigen::Vector3d odom_pos;
  Eigen::Quaterniond odom_orientation;
  if (!getOdomPoseInMap(*odom, odom_pos, odom_orientation))
    return;

  md_.camera_pos_ = odom_pos;
  md_.camera_r_m_ = odom_orientation.toRotationMatrix();

  md_.has_odom_ = true;
}

void GridMap::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstPtr &img)
{

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  if (mp_.transform_cloud_to_frame_ && !img->header.frame_id.empty() &&
      img->header.frame_id != mp_.frame_id_)
  {
    geometry_msgs::msg::TransformStamped cloud_transform;
    if (!lookupTransformSafe(mp_.frame_id_, img->header.frame_id, rclcpp::Time(img->header.stamp),
                             cloud_transform))
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Failed to transform pointcloud from %s to %s",
                           img->header.frame_id.c_str(), mp_.frame_id_.c_str());
      return;
    }

    const auto &translation = cloud_transform.transform.translation;
    const auto &rotation = cloud_transform.transform.rotation;
    Eigen::Translation3f translation_eig(
        static_cast<float>(translation.x),
        static_cast<float>(translation.y),
        static_cast<float>(translation.z));
    Eigen::Quaternionf rotation_eig(
        static_cast<float>(rotation.w),
        static_cast<float>(rotation.x),
        static_cast<float>(rotation.y),
        static_cast<float>(rotation.z));
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(latest_cloud, transformed_cloud,
                             (translation_eig * rotation_eig).matrix());
    latest_cloud.swap(transformed_cloud);
  }

  if (!mp_.transform_cloud_to_frame_ && !img->header.frame_id.empty() &&
      img->header.frame_id != mp_.frame_id_)
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Reject pointcloud: cloud frame %s is not %s and transform_cloud_to_frame is false.",
                          img->header.frame_id.c_str(), mp_.frame_id_.c_str());
    return;
  }

  geometry_msgs::msg::TransformStamped sensor_transform;
  if (!lookupTransformSafe(mp_.frame_id_, mp_.sensor_frame_, rclcpp::Time(img->header.stamp),
                           sensor_transform))
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Reject pointcloud: no timestamped sensor pose from %s to %s.",
                         mp_.sensor_frame_.c_str(), mp_.frame_id_.c_str());
    return;
  }
  md_.ray_pos_ = Eigen::Vector3d(sensor_transform.transform.translation.x,
                                 sensor_transform.transform.translation.y,
                                 sensor_transform.transform.translation.z);
  md_.ray_r_m_ = Eigen::Quaterniond(sensor_transform.transform.rotation.w,
                                    sensor_transform.transform.rotation.x,
                                    sensor_transform.transform.rotation.y,
                                    sensor_transform.transform.rotation.z)
                     .toRotationMatrix();
  md_.ray_pos_ += md_.ray_r_m_ * mp_.sensor_origin_offset_;
  if (!md_.ray_pos_.allFinite())
    return;

  updateSlidingMap(md_.ray_pos_);
  if (!isInMap(md_.ray_pos_))
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Reject pointcloud: timestamped sensor origin is outside the active map.");
    return;
  }
  md_.has_ray_pose_ = true;

  if (latest_cloud.points.size() == 0)
    return;

  md_.proj_points_cnt = 0;
  for (const pcl::PointXYZ &point : latest_cloud.points)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      continue;
    const Eigen::Vector3d measured_endpoint(point.x, point.y, point.z);
    const Eigen::Vector3d measured_deviation = measured_endpoint - md_.ray_pos_;
    const double measured_range = measured_deviation.norm();
    if (!std::isfinite(measured_range) || measured_range < mp_.min_ray_length_)
      continue;

    const Eigen::Vector3d deviation = measured_endpoint - md_.ray_pos_;
    const bool in_local_range =
        std::abs(deviation.x()) <= mp_.local_update_range_(0) &&
        std::abs(deviation.y()) <= mp_.local_update_range_(1) &&
        std::abs(deviation.z()) <= mp_.local_update_range_(2);
    if (!in_local_range && measured_range <= mp_.max_ray_length_)
      continue;
    if (md_.proj_points_cnt >= static_cast<int>(md_.proj_points_.size()))
    {
      md_.proj_points_.push_back(measured_endpoint);
    }
    else
    {
      md_.proj_points_[md_.proj_points_cnt] = measured_endpoint;
    }
    ++md_.proj_points_cnt;
  }

  if (md_.proj_points_cnt == 0)
    return;

  md_.camera_pos_ = md_.ray_pos_;
  md_.camera_r_m_ = md_.ray_r_m_;
  md_.use_cloud_update_ = true;
  md_.occ_need_update_ = true;
}

bool GridMap::lookupTransformSafe(const std::string &target_frame, const std::string &source_frame,
                                  const rclcpp::Time &stamp,
                                  geometry_msgs::msg::TransformStamped &transform)
{
  if (source_frame.empty() || source_frame == target_frame)
  {
    transform.header.frame_id = target_frame;
    transform.child_frame_id = source_frame;
    transform.transform.rotation.w = 1.0;
    return true;
  }

  try
  {
    transform = tf_buffer_->lookupTransform(
        target_frame, source_frame, stamp,
        rclcpp::Duration::from_seconds(mp_.tf_lookup_timeout_));
    return true;
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "TF lookup failed from %s to %s: %s",
                         source_frame.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

bool GridMap::getOdomPoseInMap(const nav_msgs::msg::Odometry &odom, Eigen::Vector3d &pos,
                               Eigen::Quaterniond &orientation)
{
  tf2::Transform pose_tf;
  pose_tf.setOrigin(tf2::Vector3(
      odom.pose.pose.position.x,
      odom.pose.pose.position.y,
      odom.pose.pose.position.z));
  pose_tf.setRotation(tf2::Quaternion(
      odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z,
      odom.pose.pose.orientation.w));

  if (mp_.transform_odom_to_frame_ && !odom.header.frame_id.empty() &&
      odom.header.frame_id != mp_.frame_id_)
  {
    geometry_msgs::msg::TransformStamped odom_transform;
    if (!lookupTransformSafe(mp_.frame_id_, odom.header.frame_id, rclcpp::Time(odom.header.stamp),
                             odom_transform))
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Failed to transform odom from %s to %s",
                           odom.header.frame_id.c_str(), mp_.frame_id_.c_str());
      return false;
    }

    tf2::Transform tf_target_source;
    tf_target_source.setOrigin(tf2::Vector3(
        odom_transform.transform.translation.x,
        odom_transform.transform.translation.y,
        odom_transform.transform.translation.z));
    tf_target_source.setRotation(tf2::Quaternion(
        odom_transform.transform.rotation.x,
        odom_transform.transform.rotation.y,
        odom_transform.transform.rotation.z,
        odom_transform.transform.rotation.w));
    pose_tf = tf_target_source * pose_tf;
  }

  const tf2::Vector3 origin = pose_tf.getOrigin();
  const tf2::Quaternion rotation = pose_tf.getRotation();
  pos = Eigen::Vector3d(origin.x(), origin.y(), origin.z() + mp_.odom_z_offset_);
  orientation = Eigen::Quaterniond(rotation.w(), rotation.x(), rotation.y(), rotation.z());
  return true;
}

void GridMap::publishMap()
{

  if (map_pub_->get_subscription_count() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  const Eigen::Vector3i min_cut = mp_.map_bound_min_idx_;
  const Eigen::Vector3i max_cut = mp_.map_bound_max_idx_;

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_->publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info)
{

  if (map_inf_pub_->get_subscription_count()<= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  (void)all_info;
  const Eigen::Vector3i min_cut = mp_.map_bound_min_idx_;
  const Eigen::Vector3i max_cut = mp_.map_bound_max_idx_;

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_->publish(cloud_msg);

  // RCLCPP_INFO(rclcpp::get_logger("publishMapInflate"), "pub map");
}

bool GridMap::odomValid() { return md_.has_odom_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void GridMap::extrinsicCallback(const nav_msgs::msg::Odometry::ConstPtr &odom)
{
  Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
  Eigen::Matrix3d cam2body_r_m = cam2body_q.toRotationMatrix();
  md_.cam2body_.block<3, 3>(0, 0) = cam2body_r_m;
  md_.cam2body_(0, 3) = odom->pose.pose.position.x;
  md_.cam2body_(1, 3) = odom->pose.pose.position.y;
  md_.cam2body_(2, 3) = odom->pose.pose.position.z;
  md_.cam2body_(3, 3) = 1.0;
}

void GridMap::depthOdomCallback(const sensor_msgs::msg::Image::ConstPtr &img,
                                const nav_msgs::msg::Odometry::ConstPtr &odom)
{
  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  if (!body_q.coeffs().allFinite() || body_q.norm() < 1e-6)
  {
    md_.occ_need_update_ = false;
    return;
  }
  body_q.normalize();
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world = Eigen::Matrix4d::Identity();
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);
  if (!md_.camera_pos_.allFinite() || !md_.camera_r_m_.allFinite())
  {
    md_.occ_need_update_ = false;
    return;
  }
  md_.ray_pos_ = md_.camera_pos_;
  md_.ray_r_m_ = md_.camera_r_m_;
  updateSlidingMap(md_.ray_pos_);

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  if (isInMap(md_.camera_pos_))
  {
    md_.has_odom_ = true;
    md_.has_ray_pose_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }
  md_.flag_use_depth_fusion = true;
}
