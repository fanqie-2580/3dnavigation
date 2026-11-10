/*
* BSD 3-Clause License
*
* Copyright (c) 2024, DDDMobileRobot
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <global_planner/ego_reference_path_provider.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace
{

double poseDistance(const geometry_msgs::msg::PoseStamped &a,
                    const geometry_msgs::msg::PoseStamped &b)
{
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  const double dz = a.pose.position.z - b.pose.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double poseDistanceXY(const geometry_msgs::msg::PoseStamped &a,
                      const geometry_msgs::msg::PoseStamped &b)
{
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

pcl::PointXYZI toPclPoint(const geometry_msgs::msg::PoseStamped &pose)
{
  pcl::PointXYZI point;
  point.x = pose.pose.position.x;
  point.y = pose.pose.position.y;
  point.z = pose.pose.position.z;
  point.intensity = 0.0;
  return point;
}

geometry_msgs::msg::PoseStamped interpolatePose(
  const geometry_msgs::msg::PoseStamped &from,
  const geometry_msgs::msg::PoseStamped &to,
  double ratio)
{
  const double bounded_ratio = std::clamp(ratio, 0.0, 1.0);
  geometry_msgs::msg::PoseStamped pose = to;
  if (pose.header.frame_id.empty())
    pose.header.frame_id = from.header.frame_id;
  pose.pose.position.x =
    from.pose.position.x + (to.pose.position.x - from.pose.position.x) * bounded_ratio;
  pose.pose.position.y =
    from.pose.position.y + (to.pose.position.y - from.pose.position.y) * bounded_ratio;
  pose.pose.position.z =
    from.pose.position.z + (to.pose.position.z - from.pose.position.z) * bounded_ratio;
  return pose;
}

}  // namespace

namespace global_planner
{

EgoReferencePathProvider::EgoReferencePathProvider(const std::string &name)
  : Node(name)
{
}

void EgoReferencePathProvider::initial(
  const std::shared_ptr<perception_3d::Perception3D_ROS> &perception_3d,
  const std::shared_ptr<global_planner::GlobalPlanner> &global_planner)
{
  perception_3d_ros_ = perception_3d;
  global_planner_ = global_planner;

  declare_parameter("input_path_topic", std::string("/global_path"));
  declare_parameter("output_path_topic", std::string("/ego_repaired_global_path"));
  declare_parameter("static_layer_name", std::string("map"));
  declare_parameter("dynamic_layer_name", std::string("lidar"));
  declare_parameter("publish_frequency", 2.0);
  declare_parameter("repair_window_distance", 5.0);
  declare_parameter("rejoin_search_distance", 8.0);
  declare_parameter("rejoin_candidate_step", 0.5);
  declare_parameter("edge_check_sample_step", 0.1);
  declare_parameter("min_rejoin_distance_after_block", 1.0);
  declare_parameter("ground_lookup_radius", 0.35);
  declare_parameter("min_clearance_margin", 0.0);
  declare_parameter("no_support_block_distance", 0.4);
  declare_parameter("min_pose_separation", 0.05);
  declare_parameter("min_stop_path_distance", 0.25);
  declare_parameter("min_publish_interval", 0.8);
  declare_parameter("path_start_change_threshold", 0.25);
  declare_parameter("path_goal_change_threshold", 0.2);
  declare_parameter("path_length_change_threshold", 0.5);
  declare_parameter("rejoin_change_threshold", 0.3);
  declare_parameter("keepalive_publish_interval", 5.0);
  declare_parameter("blocked_confirm_count", 2);
  declare_parameter("clear_confirm_count", 2);
  declare_parameter("enable_stop_prefix", true);
  declare_parameter("validate_repaired_path", true);
  declare_parameter("repair_dynamic_blockage", true);
  declare_parameter("repair_soft_terrain", true);
  declare_parameter("publish_original_when_repair_fails", false);
  declare_parameter("soft_terrain_repair_min_cost", 0.25);
  declare_parameter("soft_terrain_repair_min_improvement", 0.2);
  declare_parameter("path_soft_cost_sample_step", 0.1);
  declare_parameter("soft_terrain_cost_weight", 1.0);

  get_parameter("input_path_topic", input_path_topic_);
  get_parameter("output_path_topic", output_path_topic_);
  get_parameter("static_layer_name", static_layer_name_);
  get_parameter("dynamic_layer_name", dynamic_layer_name_);
  get_parameter("publish_frequency", publish_frequency_);
  get_parameter("repair_window_distance", repair_window_distance_);
  get_parameter("rejoin_search_distance", rejoin_search_distance_);
  get_parameter("rejoin_candidate_step", rejoin_candidate_step_);
  get_parameter("edge_check_sample_step", edge_check_sample_step_);
  get_parameter("min_rejoin_distance_after_block", min_rejoin_distance_after_block_);
  get_parameter("ground_lookup_radius", ground_lookup_radius_);
  get_parameter("min_clearance_margin", min_clearance_margin_);
  get_parameter("no_support_block_distance", no_support_block_distance_);
  get_parameter("min_pose_separation", min_pose_separation_);
  get_parameter("min_stop_path_distance", min_stop_path_distance_);
  get_parameter("min_publish_interval", min_publish_interval_);
  get_parameter("path_start_change_threshold", path_start_change_threshold_);
  get_parameter("path_goal_change_threshold", path_goal_change_threshold_);
  get_parameter("path_length_change_threshold", path_length_change_threshold_);
  get_parameter("rejoin_change_threshold", rejoin_change_threshold_);
  get_parameter("keepalive_publish_interval", keepalive_publish_interval_);
  get_parameter("blocked_confirm_count", blocked_confirm_count_);
  get_parameter("clear_confirm_count", clear_confirm_count_);
  get_parameter("enable_stop_prefix", enable_stop_prefix_);
  get_parameter("validate_repaired_path", validate_repaired_path_);
  get_parameter("repair_dynamic_blockage", repair_dynamic_blockage_);
  get_parameter("repair_soft_terrain", repair_soft_terrain_);
  get_parameter("publish_original_when_repair_fails", publish_original_when_repair_fails_);
  get_parameter("soft_terrain_repair_min_cost", soft_terrain_repair_min_cost_);
  get_parameter("soft_terrain_repair_min_improvement", soft_terrain_repair_min_improvement_);
  get_parameter("path_soft_cost_sample_step", path_soft_cost_sample_step_);
  get_parameter("soft_terrain_cost_weight", soft_terrain_cost_weight_);

  publish_frequency_ = std::max(0.1, publish_frequency_);
  repair_window_distance_ = std::max(0.1, repair_window_distance_);
  rejoin_search_distance_ = std::max(repair_window_distance_, rejoin_search_distance_);
  rejoin_candidate_step_ = std::max(0.1, rejoin_candidate_step_);
  edge_check_sample_step_ = std::max(0.05, edge_check_sample_step_);
  min_rejoin_distance_after_block_ = std::max(0.0, min_rejoin_distance_after_block_);
  ground_lookup_radius_ = std::max(0.05, ground_lookup_radius_);
  no_support_block_distance_ = std::max(edge_check_sample_step_, no_support_block_distance_);
  min_pose_separation_ = std::max(0.0, min_pose_separation_);
  min_stop_path_distance_ = std::max(0.0, min_stop_path_distance_);
  min_publish_interval_ = std::max(0.0, min_publish_interval_);
  path_start_change_threshold_ = std::max(0.0, path_start_change_threshold_);
  path_goal_change_threshold_ = std::max(0.0, path_goal_change_threshold_);
  path_length_change_threshold_ = std::max(0.0, path_length_change_threshold_);
  rejoin_change_threshold_ = std::max(0.0, rejoin_change_threshold_);
  keepalive_publish_interval_ = std::max(0.0, keepalive_publish_interval_);
  soft_terrain_repair_min_cost_ = std::max(0.0, soft_terrain_repair_min_cost_);
  soft_terrain_repair_min_improvement_ = std::max(0.0, soft_terrain_repair_min_improvement_);
  path_soft_cost_sample_step_ = std::max(0.05, path_soft_cost_sample_step_);
  soft_terrain_cost_weight_ = std::max(0.0, soft_terrain_cost_weight_);
  blocked_confirm_count_ = std::max(1, blocked_confirm_count_);
  clear_confirm_count_ = std::max(1, clear_confirm_count_);
  last_publish_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

  if (input_path_topic_ == output_path_topic_)
  {
    disabled_ = true;
    RCLCPP_ERROR(
      get_logger(),
      "EGO reference path provider disabled because input_path_topic and "
      "output_path_topic are both '%s'.",
      input_path_topic_.c_str());
    return;
  }

  repaired_path_pub_ = create_publisher<nav_msgs::msg::Path>(output_path_topic_, 1);
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    input_path_topic_, 1,
    std::bind(&EgoReferencePathProvider::pathCallback, this, std::placeholders::_1));

  const auto loop_time = std::chrono::milliseconds(
    static_cast<int>(1000.0 / publish_frequency_));
  publish_timer_ = create_wall_timer(
    loop_time, std::bind(&EgoReferencePathProvider::publishLoop, this));

  RCLCPP_INFO(
    get_logger(),
    "EGO reference path provider ready. input=%s output=%s static_layer=%s dynamic_layer=%s window=%.2f rejoin=%.2f min_rejoin=%.2f sample=%.2f clearance=inscribed+%.2f publish_interval=%.2f repair_dynamic=%d repair_soft=%d publish_original_on_repair_fail=%d soft_min=%.2f soft_improvement=%.2f soft_weight=%.2f",
    input_path_topic_.c_str(), output_path_topic_.c_str(),
    static_layer_name_.c_str(), dynamic_layer_name_.c_str(),
    repair_window_distance_, rejoin_search_distance_, min_rejoin_distance_after_block_,
    edge_check_sample_step_, min_clearance_margin_, min_publish_interval_,
    repair_dynamic_blockage_, repair_soft_terrain_, publish_original_when_repair_fails_,
    soft_terrain_repair_min_cost_, soft_terrain_repair_min_improvement_,
    soft_terrain_cost_weight_);
}

void EgoReferencePathProvider::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty())
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Received empty source global path; keeping the previous source path.");
    return;
  }

  std::lock_guard<std::mutex> lock(path_mutex_);
  original_path_ = *msg;
  have_original_path_ = true;
  source_path_revision_++;
}

bool EgoReferencePathProvider::perceptionReady() const
{
  if (!perception_3d_ros_)
    return false;

  auto shared_data = perception_3d_ros_->getSharedDataPtr();
  return shared_data && shared_data->is_static_layer_ready_ &&
         shared_data->kdtree_ground_ && shared_data->pcl_ground_ &&
         shared_data->pcl_ground_->size() > 0;
}

bool EgoReferencePathProvider::getRobotPose(geometry_msgs::msg::PoseStamped &pose) const
{
  if (!perception_3d_ros_)
    return false;

  perception_3d_ros_->getGlobalPose(pose);
  pose.header.stamp = now();
  if (pose.header.frame_id.empty())
    pose.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  return true;
}

double EgoReferencePathProvider::minRequiredClearance() const
{
  if (!perception_3d_ros_)
    return std::max(0.0, min_clearance_margin_);

  return std::max(
    0.0,
    perception_3d_ros_->getGlobalUtils()->getInscribedRadius() + min_clearance_margin_);
}

EgoReferencePathProvider::PoseAssessment EgoReferencePathProvider::assessPose(
  const geometry_msgs::msg::PoseStamped &pose) const
{
  PoseAssessment assessment;

  if (!perceptionReady())
  {
    assessment.status = Traversability::HardBlocked;
    assessment.reason = "perception is not ready";
    return assessment;
  }

  auto shared_data = perception_3d_ros_->getSharedDataPtr();
  std::unique_lock<std::recursive_mutex> lock(shared_data->ground_kdtree_cb_mutex_);

  std::vector<int> indices;
  std::vector<float> distances;
  const pcl::PointXYZI query = toPclPoint(pose);
  if (shared_data->kdtree_ground_->nearestKSearch(query, 1, indices, distances) < 1 ||
      indices.empty())
  {
    assessment.status = Traversability::NoSupport;
    assessment.reason = "no ground near pose";
    return assessment;
  }

  const double ground_distance = std::sqrt(distances.front());
  if (ground_distance > ground_lookup_radius_)
  {
    assessment.status = Traversability::NoSupport;
    assessment.ground_index = indices.front();
    assessment.reason = "nearest ground is outside lookup radius";
    return assessment;
  }

  assessment.ground_index = indices.front();
  assessment.clearance = perception_3d_ros_->get_min_dGraphValue(indices.front());
  assessment.has_map_clearance = perception_3d_ros_->get_dGraphValueByLayer(
    static_layer_name_, indices.front(), assessment.map_clearance);
  assessment.has_lidar_clearance = perception_3d_ros_->get_dGraphValueByLayer(
    dynamic_layer_name_, indices.front(), assessment.lidar_clearance);
  assessment.has_soft_terrain_cost = perception_3d_ros_->get_softTerrainCostByLayer(
    dynamic_layer_name_, indices.front(), assessment.soft_terrain_cost);

  const double required_clearance = minRequiredClearance();
  if (assessment.has_map_clearance && assessment.map_clearance < required_clearance)
  {
    assessment.status = Traversability::HardBlocked;
    assessment.reason = "static map dGraph clearance is below required clearance";
    return assessment;
  }

  if (assessment.has_lidar_clearance && assessment.lidar_clearance < required_clearance)
  {
    assessment.status = Traversability::DynamicBlocked;
    assessment.reason = "lidar-only dGraph clearance is below required clearance";
    return assessment;
  }

  if (assessment.clearance < required_clearance)
  {
    assessment.status = Traversability::HardBlocked;
    assessment.reason = "unclassified dGraph clearance is below required clearance";
    return assessment;
  }

  if (assessment.has_soft_terrain_cost && assessment.soft_terrain_cost > 1e-6)
  {
    assessment.status = Traversability::DynamicSoftTerrain;
    assessment.reason = "lidar soft terrain cost is present";
    return assessment;
  }

  assessment.status = Traversability::Clear;
  assessment.reason = "clear";
  return assessment;
}

bool EgoReferencePathProvider::isPoseUsableForReference(
  const geometry_msgs::msg::PoseStamped &pose) const
{
  const PoseAssessment assessment = assessPose(pose);
  return assessment.status == Traversability::Clear ||
         assessment.status == Traversability::DynamicSoftTerrain ||
         (!repair_dynamic_blockage_ && assessment.status == Traversability::DynamicBlocked);
}

bool EgoReferencePathProvider::isHardBlockage(Traversability status) const
{
  return status == Traversability::HardBlocked || status == Traversability::NoSupport;
}

bool EgoReferencePathProvider::isRepairableBlockage(Traversability status) const
{
  return isHardBlockage(status) ||
         (repair_dynamic_blockage_ && status == Traversability::DynamicBlocked);
}

size_t EgoReferencePathProvider::findNearestPathIndex(
  const nav_msgs::msg::Path &path,
  const geometry_msgs::msg::PoseStamped &pose) const
{
  size_t best_index = 0;
  double best_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < path.poses.size(); ++i)
  {
    const double distance = poseDistance(path.poses[i], pose);
    if (distance < best_distance)
    {
      best_distance = distance;
      best_index = i;
    }
  }

  return best_index;
}

bool EgoReferencePathProvider::findFirstBlockage(
  const nav_msgs::msg::Path &path,
  const geometry_msgs::msg::PoseStamped &start_pose,
  size_t start_index,
  double max_distance,
  PathBlockage &blockage) const
{
  blockage = PathBlockage{};
  if (path.poses.empty() || start_index >= path.poses.size())
    return false;

  geometry_msgs::msg::PoseStamped previous =
    makeReferenceLayerStartPose(path, start_pose, start_index);
  double accumulated_distance = 0.0;
  bool has_last_safe = false;
  size_t last_safe_index = start_index;
  geometry_msgs::msg::PoseStamped last_safe_pose = previous;
  double no_support_start_distance = -1.0;

  for (size_t i = start_index; i < path.poses.size(); ++i)
  {
    const double segment_length = poseDistance(previous, path.poses[i]);
    const int sample_count =
      std::max(1, static_cast<int>(std::ceil(segment_length / edge_check_sample_step_)));

    for (int sample_index = 1; sample_index <= sample_count; ++sample_index)
    {
      const double ratio = static_cast<double>(sample_index) / sample_count;
      const double sample_distance = accumulated_distance + segment_length * ratio;
      if (sample_distance > max_distance)
        return false;

      const geometry_msgs::msg::PoseStamped sample_pose =
        interpolatePose(previous, path.poses[i], ratio);

      const PoseAssessment assessment = assessPose(sample_pose);
      if (assessment.status == Traversability::NoSupport)
      {
        if (no_support_start_distance < 0.0)
          no_support_start_distance = sample_distance;

        if (sample_distance - no_support_start_distance < no_support_block_distance_)
          continue;
      }
      else
      {
        no_support_start_distance = -1.0;
      }

      if (isRepairableBlockage(assessment.status))
      {
        blockage.blocked = true;
        blockage.blocked_index = i;
        blockage.last_safe_index = last_safe_index;
        blockage.has_last_safe = has_last_safe;
        blockage.blocked_pose = sample_pose;
        blockage.last_safe_pose = last_safe_pose;
        blockage.distance_from_start = sample_distance;
        blockage.clearance = assessment.clearance;
        blockage.ground_index = assessment.ground_index;
        blockage.status = assessment.status;
        blockage.reason = assessment.reason;
        return true;
      }

      has_last_safe = true;
      last_safe_pose = sample_pose;
    }

    last_safe_index = i;
    previous = path.poses[i];
    accumulated_distance += segment_length;
  }

  return false;
}

bool EgoReferencePathProvider::findFirstSoftTerrain(
  const nav_msgs::msg::Path &path,
  const geometry_msgs::msg::PoseStamped &start_pose,
  size_t start_index,
  double max_distance,
  PathBlockage &soft_terrain) const
{
  soft_terrain = PathBlockage{};
  if (path.poses.empty() || start_index >= path.poses.size() ||
      soft_terrain_repair_min_cost_ <= 0.0)
    return false;

  geometry_msgs::msg::PoseStamped previous =
    makeReferenceLayerStartPose(path, start_pose, start_index);
  double accumulated_distance = 0.0;
  double accumulated_soft_cost = 0.0;
  bool has_last_safe = false;
  size_t last_safe_index = start_index;
  geometry_msgs::msg::PoseStamped last_safe_pose = previous;

  for (size_t i = start_index; i < path.poses.size(); ++i)
  {
    const double segment_length = poseDistance(previous, path.poses[i]);
    const int sample_count =
      std::max(1, static_cast<int>(std::ceil(segment_length / path_soft_cost_sample_step_)));

    for (int sample_index = 1; sample_index <= sample_count; ++sample_index)
    {
      const double ratio = static_cast<double>(sample_index) / sample_count;
      const double sample_distance = accumulated_distance + segment_length * ratio;
      if (sample_distance > max_distance)
        return false;

      const geometry_msgs::msg::PoseStamped sample_pose =
        interpolatePose(previous, path.poses[i], ratio);
      const PoseAssessment assessment = assessPose(sample_pose);

      if (isRepairableBlockage(assessment.status))
        return false;

      if (assessment.status == Traversability::DynamicSoftTerrain)
      {
        accumulated_soft_cost +=
          assessment.soft_terrain_cost * (segment_length / static_cast<double>(sample_count));
        if (accumulated_soft_cost >= soft_terrain_repair_min_cost_)
        {
          soft_terrain.blocked = true;
          soft_terrain.blocked_index = i;
          soft_terrain.last_safe_index = last_safe_index;
          soft_terrain.has_last_safe = has_last_safe;
          soft_terrain.blocked_pose = sample_pose;
          soft_terrain.last_safe_pose = last_safe_pose;
          soft_terrain.distance_from_start = sample_distance;
          soft_terrain.clearance = assessment.clearance;
          soft_terrain.ground_index = assessment.ground_index;
          soft_terrain.status = assessment.status;
          soft_terrain.reason = "lidar soft terrain cost exceeds repair threshold";
          return true;
        }
      }

      has_last_safe = true;
      last_safe_pose = sample_pose;
    }

    last_safe_index = i;
    previous = path.poses[i];
    accumulated_distance += segment_length;
  }

  return false;
}

bool EgoReferencePathProvider::buildRepairedPath(
  const nav_msgs::msg::Path &original_path,
  const geometry_msgs::msg::PoseStamped &robot_pose,
  size_t start_index,
  const PathBlockage &blockage,
  nav_msgs::msg::Path &repaired_path,
  geometry_msgs::msg::PoseStamped &selected_rejoin_pose)
{
  if (!global_planner_ || !blockage.blocked ||
      blockage.blocked_index >= original_path.poses.size())
    return false;

  double searched_distance = 0.0;
  double distance_since_candidate = rejoin_candidate_step_;

  for (size_t i = blockage.blocked_index + 1; i < original_path.poses.size(); ++i)
  {
    searched_distance += poseDistance(original_path.poses[i - 1], original_path.poses[i]);
    distance_since_candidate += poseDistance(original_path.poses[i - 1], original_path.poses[i]);
    if (searched_distance > rejoin_search_distance_)
      break;
    if (searched_distance + 1e-6 < min_rejoin_distance_after_block_ &&
        i + 1 < original_path.poses.size())
      continue;
    if (distance_since_candidate + 1e-6 < rejoin_candidate_step_ &&
        i + 1 < original_path.poses.size())
      continue;

    distance_since_candidate = 0.0;

    if (!isPoseUsableForReference(original_path.poses[i]))
      continue;

    geometry_msgs::msg::PoseStamped rejoin_pose = original_path.poses[i];
    if (rejoin_pose.header.frame_id.empty())
      rejoin_pose.header.frame_id = original_path.header.frame_id;

    nav_msgs::msg::Path repair_segment = global_planner_->makeROSPlan(robot_pose, rejoin_pose);
    if (repair_segment.poses.empty())
      continue;

    if (validate_repaired_path_)
    {
      PathBlockage repair_blockage;
      const double validation_distance = pathLength(repair_segment) + 0.1;
      if (findFirstBlockage(repair_segment, robot_pose, 0, validation_distance, repair_blockage))
        continue;
    }

    repaired_path = composeRepairedPath(repair_segment, original_path, robot_pose, start_index, i);
    if (validate_repaired_path_)
    {
      PathBlockage composed_blockage;
      const double validation_distance =
        std::min(pathLength(repaired_path), std::max(repair_window_distance_, pathLength(repair_segment) + 0.5));
      if (findFirstBlockage(repaired_path, robot_pose, 0, validation_distance, composed_blockage))
        continue;
    }

    if (repaired_path.poses.size() >= 2)
    {
      selected_rejoin_pose = rejoin_pose;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Repaired EGO reference path around blocked source path. blocked_index=%zu rejoin_index=%zu start_index=%zu rejoin_distance=%.2f",
        blockage.blocked_index, i, start_index, searched_distance);
      return true;
    }
  }

  return false;
}

nav_msgs::msg::Path EgoReferencePathProvider::buildTrimmedOriginalPath(
  const nav_msgs::msg::Path &original_path,
  const geometry_msgs::msg::PoseStamped &robot_pose,
  size_t start_index) const
{
  nav_msgs::msg::Path trimmed;
  trimmed.header = original_path.header;
  trimmed.header.stamp = now();
  if (trimmed.header.frame_id.empty() && perception_3d_ros_)
    trimmed.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();

  geometry_msgs::msg::PoseStamped start_pose =
    makeReferenceLayerStartPose(original_path, robot_pose, start_index);
  start_pose.header = trimmed.header;
  appendPoseIfSeparated(trimmed, start_pose, 0.0);

  for (size_t i = start_index; i < original_path.poses.size(); ++i)
    appendPoseIfSeparated(trimmed, original_path.poses[i], min_pose_separation_);

  return trimmed;
}

bool EgoReferencePathProvider::buildSafePrefixPath(
  const nav_msgs::msg::Path &original_path,
  const geometry_msgs::msg::PoseStamped &robot_pose,
  size_t start_index,
  const PathBlockage &blockage,
  nav_msgs::msg::Path &prefix_path) const
{
  if (original_path.poses.empty() || start_index >= original_path.poses.size() ||
      !blockage.has_last_safe || blockage.last_safe_index < start_index)
    return false;

  prefix_path.header = original_path.header;
  prefix_path.header.stamp = now();
  if (prefix_path.header.frame_id.empty() && perception_3d_ros_)
    prefix_path.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();

  geometry_msgs::msg::PoseStamped start_pose =
    makeReferenceLayerStartPose(original_path, robot_pose, start_index);
  start_pose.header = prefix_path.header;
  appendPoseIfSeparated(prefix_path, start_pose, 0.0);

  for (size_t i = start_index; i <= blockage.last_safe_index && i < original_path.poses.size(); ++i)
    appendPoseIfSeparated(prefix_path, original_path.poses[i], min_pose_separation_);

  appendPoseIfSeparated(prefix_path, blockage.last_safe_pose, min_pose_separation_);

  return prefix_path.poses.size() >= 2 && pathLength(prefix_path) >= min_stop_path_distance_;
}

nav_msgs::msg::Path EgoReferencePathProvider::composeRepairedPath(
  const nav_msgs::msg::Path &repair_segment,
  const nav_msgs::msg::Path &original_path,
  const geometry_msgs::msg::PoseStamped &robot_pose,
  size_t start_index,
  size_t rejoin_index) const
{
  nav_msgs::msg::Path composed;
  composed.header = original_path.header;
  composed.header.stamp = now();
  if (composed.header.frame_id.empty() && perception_3d_ros_)
    composed.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();

  geometry_msgs::msg::PoseStamped start_pose =
    makeReferenceLayerStartPose(original_path, robot_pose, start_index);
  start_pose.header = composed.header;
  appendPoseIfSeparated(composed, start_pose, 0.0);

  for (const auto &pose : repair_segment.poses)
  {
    if (composed.poses.size() == 1 &&
        poseDistanceXY(start_pose, pose) < std::max(0.01, min_pose_separation_))
    {
      continue;
    }
    appendPoseIfSeparated(composed, pose, min_pose_separation_);
  }

  for (size_t i = rejoin_index + 1; i < original_path.poses.size(); ++i)
    appendPoseIfSeparated(composed, original_path.poses[i], min_pose_separation_);

  return composed;
}

geometry_msgs::msg::PoseStamped EgoReferencePathProvider::makeReferenceLayerStartPose(
  const nav_msgs::msg::Path &original_path,
  const geometry_msgs::msg::PoseStamped &robot_pose,
  size_t start_index) const
{
  geometry_msgs::msg::PoseStamped start_pose = robot_pose;
  start_pose.header = original_path.header;
  start_pose.header.stamp = now();
  if (start_pose.header.frame_id.empty() && perception_3d_ros_)
    start_pose.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();

  if (!original_path.poses.empty())
  {
    const size_t z_index = std::min(start_index, original_path.poses.size() - 1);
    start_pose.pose.position.z = original_path.poses[z_index].pose.position.z;
  }

  return start_pose;
}

double EgoReferencePathProvider::pathLength(const nav_msgs::msg::Path &path) const
{
  if (path.poses.size() < 2)
    return 0.0;

  double length = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i)
    length += poseDistance(path.poses[i - 1], path.poses[i]);
  return length;
}

double EgoReferencePathProvider::sampledPathCost(
  const nav_msgs::msg::Path &path,
  const geometry_msgs::msg::PoseStamped &start_pose,
  size_t start_index,
  double max_distance) const
{
  if (path.poses.empty() || start_index >= path.poses.size())
    return std::numeric_limits<double>::infinity();

  geometry_msgs::msg::PoseStamped previous =
    makeReferenceLayerStartPose(path, start_pose, start_index);
  double accumulated_distance = 0.0;
  double cost = 0.0;

  for (size_t i = start_index; i < path.poses.size(); ++i)
  {
    const double segment_length = poseDistance(previous, path.poses[i]);
    const int sample_count =
      std::max(1, static_cast<int>(std::ceil(segment_length / path_soft_cost_sample_step_)));

    for (int sample_index = 1; sample_index <= sample_count; ++sample_index)
    {
      const double previous_sample_distance =
        accumulated_distance +
        segment_length * static_cast<double>(sample_index - 1) / static_cast<double>(sample_count);
      const double sample_distance =
        accumulated_distance +
        segment_length * static_cast<double>(sample_index) / static_cast<double>(sample_count);
      if (previous_sample_distance >= max_distance)
        return cost;

      const double sample_length = std::min(
        segment_length / static_cast<double>(sample_count),
        std::max(0.0, max_distance - previous_sample_distance));
      const double ratio = static_cast<double>(sample_index) / sample_count;
      const geometry_msgs::msg::PoseStamped sample_pose =
        interpolatePose(previous, path.poses[i], ratio);
      const PoseAssessment assessment = assessPose(sample_pose);

      if (assessment.status == Traversability::HardBlocked ||
          assessment.status == Traversability::DynamicBlocked ||
          assessment.status == Traversability::NoSupport)
      {
        return std::numeric_limits<double>::infinity();
      }

      const double soft_cost =
        assessment.status == Traversability::DynamicSoftTerrain ?
        assessment.soft_terrain_cost : 0.0;
      cost += sample_length * (1.0 + soft_terrain_cost_weight_ * soft_cost);

      if (sample_distance >= max_distance)
        return cost;
    }

    previous = path.poses[i];
    accumulated_distance += segment_length;
  }

  return cost;
}

double EgoReferencePathProvider::maxSampledPathDistanceXY(
  const nav_msgs::msg::Path &a,
  const nav_msgs::msg::Path &b) const
{
  if (a.poses.empty() || b.poses.empty())
    return std::numeric_limits<double>::infinity();

  const size_t sample_count =
    std::min<size_t>(20, std::max(a.poses.size(), b.poses.size()));
  if (sample_count <= 1)
    return poseDistanceXY(a.poses.front(), b.poses.front());

  double max_distance = 0.0;
  for (size_t sample = 0; sample < sample_count; ++sample)
  {
    const double ratio =
      static_cast<double>(sample) / static_cast<double>(sample_count - 1);
    const size_t a_index = std::min(
      a.poses.size() - 1,
      static_cast<size_t>(std::round(ratio * static_cast<double>(a.poses.size() - 1))));
    const size_t b_index = std::min(
      b.poses.size() - 1,
      static_cast<size_t>(std::round(ratio * static_cast<double>(b.poses.size() - 1))));
    max_distance = std::max(max_distance, poseDistanceXY(a.poses[a_index], b.poses[b_index]));
  }

  return max_distance;
}

bool EgoReferencePathProvider::isPathMateriallyDifferent(const nav_msgs::msg::Path &path) const
{
  if (!have_last_published_path_)
    return true;

  if (path.poses.empty() || last_published_path_.poses.empty())
    return true;

  if (poseDistanceXY(path.poses.front(), last_published_path_.poses.front()) >
      path_start_change_threshold_)
    return true;

  if (poseDistanceXY(path.poses.back(), last_published_path_.poses.back()) >
      path_goal_change_threshold_)
    return true;

  if (std::abs(pathLength(path) - pathLength(last_published_path_)) >
      path_length_change_threshold_)
    return true;

  return maxSampledPathDistanceXY(path, last_published_path_) > rejoin_change_threshold_;
}

bool EgoReferencePathProvider::shouldPublishPath(
  const nav_msgs::msg::Path &path,
  OutputMode mode,
  const geometry_msgs::msg::PoseStamped *rejoin_pose,
  const rclcpp::Time &publish_time) const
{
  if (!have_last_published_path_)
    return true;

  const double elapsed = (publish_time - last_publish_time_).seconds();
  const bool publish_interval_elapsed = elapsed >= min_publish_interval_;

  if (mode != last_published_mode_)
    return mode != OutputMode::ClearOriginal || publish_interval_elapsed;

  if (rejoin_pose != nullptr)
  {
    if (!have_last_published_rejoin_pose_ ||
        poseDistanceXY(*rejoin_pose, last_published_rejoin_pose_) > rejoin_change_threshold_)
      return publish_interval_elapsed;
  }
  else if (have_last_published_rejoin_pose_)
  {
    return publish_interval_elapsed;
  }

  if (isPathMateriallyDifferent(path))
    return publish_interval_elapsed;

  return keepalive_publish_interval_ > 0.0 && elapsed >= keepalive_publish_interval_;
}

void EgoReferencePathProvider::rememberPublishedPath(
  const nav_msgs::msg::Path &path,
  OutputMode mode,
  const geometry_msgs::msg::PoseStamped *rejoin_pose,
  const rclcpp::Time &publish_time)
{
  last_published_path_ = path;
  last_published_mode_ = mode;
  last_publish_time_ = publish_time;
  have_last_published_path_ = true;

  if (rejoin_pose != nullptr)
  {
    last_published_rejoin_pose_ = *rejoin_pose;
    have_last_published_rejoin_pose_ = true;
  }
  else
  {
    have_last_published_rejoin_pose_ = false;
  }
}

void EgoReferencePathProvider::appendPoseIfSeparated(
  nav_msgs::msg::Path &path,
  const geometry_msgs::msg::PoseStamped &pose,
  double min_separation) const
{
  geometry_msgs::msg::PoseStamped normalized_pose = pose;
  if (normalized_pose.header.frame_id.empty())
    normalized_pose.header.frame_id = path.header.frame_id;
  if (normalized_pose.header.stamp.sec == 0 && normalized_pose.header.stamp.nanosec == 0)
    normalized_pose.header.stamp = path.header.stamp;

  if (path.poses.empty() ||
      poseDistance(path.poses.back(), normalized_pose) >= min_separation)
  {
    path.poses.push_back(normalized_pose);
  }
}

bool EgoReferencePathProvider::publishPath(
  const nav_msgs::msg::Path &path,
  OutputMode mode,
  const PathBlockage *blockage,
  const geometry_msgs::msg::PoseStamped *rejoin_pose)
{
  if (path.poses.size() < 2)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Not publishing EGO reference path because it has fewer than two poses.");
    return false;
  }

  const rclcpp::Time publish_time = now();
  if (!shouldPublishPath(path, mode, rejoin_pose, publish_time))
    return false;

  repaired_path_pub_->publish(path);
  rememberPublishedPath(path, mode, rejoin_pose, publish_time);

  if (mode == OutputMode::ClearOriginal)
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Publishing trimmed original EGO reference path with %zu poses.",
      path.poses.size());
  }
  else if (mode == OutputMode::Repaired && blockage)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Publishing repaired EGO reference path with %zu poses. blocked=[%.2f, %.2f, %.2f] reason=%s",
      path.poses.size(),
      blockage->blocked_pose.pose.position.x,
      blockage->blocked_pose.pose.position.y,
      blockage->blocked_pose.pose.position.z,
      blockage->reason.c_str());
  }
  else if (mode == OutputMode::StopBeforeBlock && blockage)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Publishing stop-before-block EGO reference path with %zu poses. blocked=[%.2f, %.2f, %.2f] reason=%s",
      path.poses.size(),
      blockage->blocked_pose.pose.position.x,
      blockage->blocked_pose.pose.position.y,
      blockage->blocked_pose.pose.position.z,
      blockage->reason.c_str());
  }

  return true;
}

void EgoReferencePathProvider::publishLoop()
{
  if (disabled_)
    return;

  nav_msgs::msg::Path source_path;
  size_t source_path_revision = 0;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (!have_original_path_)
      return;
    source_path = original_path_;
    source_path_revision = source_path_revision_;
  }

  const bool source_path_changed =
    source_path_revision != processed_source_path_revision_;
  if (source_path_changed)
  {
    processed_source_path_revision_ = source_path_revision;
    blocked_observation_count_ = 0;
    clear_observation_count_ = 0;
    confirmed_blocked_ = false;
  }

  if (!perceptionReady())
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Waiting for perception static layer before repairing EGO reference path.");
    return;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Unable to get robot pose for EGO reference path repair.");
    return;
  }

  const size_t start_index = findNearestPathIndex(source_path, robot_pose);
  PathBlockage blockage;
  const bool blockage_observed = findFirstBlockage(
    source_path, robot_pose, start_index, repair_window_distance_, blockage);
  if (!blockage_observed)
  {
    blocked_observation_count_ = 0;
    if (confirmed_blocked_)
    {
      clear_observation_count_++;
      if (clear_observation_count_ < clear_confirm_count_)
      {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "EGO reference path blockage not observed; waiting for stable clear confirmation %d/%d.",
          clear_observation_count_, clear_confirm_count_);
        return;
      }
      confirmed_blocked_ = false;
    }
    else
    {
      clear_observation_count_ = std::min(clear_confirm_count_, clear_observation_count_ + 1);
    }

    const nav_msgs::msg::Path trimmed_path =
      buildTrimmedOriginalPath(source_path, robot_pose, start_index);
    if (repair_soft_terrain_)
    {
      PathBlockage soft_terrain;
      if (findFirstSoftTerrain(
          source_path, robot_pose, start_index, repair_window_distance_, soft_terrain))
      {
        nav_msgs::msg::Path repaired_path;
        geometry_msgs::msg::PoseStamped rejoin_pose;
        if (buildRepairedPath(
            source_path, robot_pose, start_index, soft_terrain, repaired_path, rejoin_pose))
        {
          const double original_cost =
            sampledPathCost(trimmed_path, robot_pose, 0, repair_window_distance_);
          const double repaired_cost =
            sampledPathCost(repaired_path, robot_pose, 0, repair_window_distance_);
          if (repaired_cost + soft_terrain_repair_min_improvement_ < original_cost)
          {
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "Publishing lower-cost EGO reference path around lidar soft terrain. original_cost=%.2f repaired_cost=%.2f",
              original_cost, repaired_cost);
            publishPath(repaired_path, OutputMode::Repaired, &soft_terrain, &rejoin_pose);
            return;
          }
        }
      }
    }
    publishPath(trimmed_path, OutputMode::ClearOriginal, nullptr);
    return;
  }

  clear_observation_count_ = 0;
  blocked_observation_count_++;
  if (!confirmed_blocked_ && blocked_observation_count_ < blocked_confirm_count_)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Potential EGO reference path blockage observed %d/%d. blocked=[%.2f, %.2f, %.2f] reason=%s",
      blocked_observation_count_, blocked_confirm_count_,
      blockage.blocked_pose.pose.position.x,
      blockage.blocked_pose.pose.position.y,
      blockage.blocked_pose.pose.position.z,
      blockage.reason.c_str());
    return;
  }
  confirmed_blocked_ = true;

  nav_msgs::msg::Path repaired_path;
  geometry_msgs::msg::PoseStamped rejoin_pose;
  if (buildRepairedPath(source_path, robot_pose, start_index, blockage, repaired_path, rejoin_pose))
  {
    publishPath(repaired_path, OutputMode::Repaired, &blockage, &rejoin_pose);
    return;
  }

  if (publish_original_when_repair_fails_)
  {
    const nav_msgs::msg::Path trimmed_path =
      buildTrimmedOriginalPath(source_path, robot_pose, start_index);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Source EGO reference path is blocked within %.2f m and no safe rejoin path was found; publishing original reference for EGO local handling. blocked=[%.2f, %.2f, %.2f] reason=%s",
      repair_window_distance_,
      blockage.blocked_pose.pose.position.x,
      blockage.blocked_pose.pose.position.y,
      blockage.blocked_pose.pose.position.z,
      blockage.reason.c_str());
    publishPath(trimmed_path, OutputMode::ClearOriginal, nullptr);
    return;
  }

  if (enable_stop_prefix_)
  {
    nav_msgs::msg::Path stop_path;
    if (buildSafePrefixPath(source_path, robot_pose, start_index, blockage, stop_path))
    {
      publishPath(stop_path, OutputMode::StopBeforeBlock, &blockage);
      return;
    }

    // Fallback: hold at robot so EGO does not keep tracking a stale blocked path.
    stop_path.header = source_path.header;
    stop_path.header.stamp = now();
    if (stop_path.header.frame_id.empty() && perception_3d_ros_)
      stop_path.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
    geometry_msgs::msg::PoseStamped hold = robot_pose;
    hold.header = stop_path.header;
    stop_path.poses.push_back(hold);
    stop_path.poses.push_back(hold);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Repair failed and no safe prefix; publishing hold-in-place stop path. blocked=[%.2f, %.2f, %.2f] reason=%s",
      blockage.blocked_pose.pose.position.x,
      blockage.blocked_pose.pose.position.y,
      blockage.blocked_pose.pose.position.z,
      blockage.reason.c_str());
    publishPath(stop_path, OutputMode::StopBeforeBlock, &blockage);
    return;
  }

  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "Source EGO reference path is blocked within %.2f m and no safe rejoin path was found. blocked=[%.2f, %.2f, %.2f] reason=%s",
    repair_window_distance_,
    blockage.blocked_pose.pose.position.x,
    blockage.blocked_pose.pose.position.y,
    blockage.blocked_pose.pose.position.z,
    blockage.reason.c_str());
}

}  // namespace global_planner
