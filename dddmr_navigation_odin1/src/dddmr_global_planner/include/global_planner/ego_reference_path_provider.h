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

#ifndef GLOBAL_PLANNER_EGO_REFERENCE_PATH_PROVIDER_H_
#define GLOBAL_PLANNER_EGO_REFERENCE_PATH_PROVIDER_H_

#include <global_planner/global_planner.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace global_planner
{

class EgoReferencePathProvider : public rclcpp::Node
{
public:
  explicit EgoReferencePathProvider(const std::string &name);
  ~EgoReferencePathProvider() override = default;

  void initial(const std::shared_ptr<perception_3d::Perception3D_ROS> &perception_3d,
               const std::shared_ptr<global_planner::GlobalPlanner> &global_planner);

private:
  enum class OutputMode
  {
    ClearOriginal,
    Repaired,
    StopBeforeBlock
  };

  enum class Traversability
  {
    Clear,
    DynamicSoftTerrain,
    DynamicBlocked,
    HardBlocked,
    NoSupport
  };

  struct PoseAssessment
  {
    Traversability status{Traversability::Clear};
    double clearance{0.0};
    double map_clearance{0.0};
    double lidar_clearance{0.0};
    double soft_terrain_cost{0.0};
    bool has_map_clearance{false};
    bool has_lidar_clearance{false};
    bool has_soft_terrain_cost{false};
    int ground_index{-1};
    std::string reason;
  };

  struct PathBlockage
  {
    bool blocked{false};
    size_t blocked_index{0};
    size_t last_safe_index{0};
    bool has_last_safe{false};
    geometry_msgs::msg::PoseStamped blocked_pose;
    geometry_msgs::msg::PoseStamped last_safe_pose;
    double distance_from_start{0.0};
    double clearance{0.0};
    int ground_index{-1};
    Traversability status{Traversability::Clear};
    std::string reason;
  };

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void publishLoop();

  bool perceptionReady() const;
  bool getRobotPose(geometry_msgs::msg::PoseStamped &pose) const;
  PoseAssessment assessPose(const geometry_msgs::msg::PoseStamped &pose) const;
  bool isPoseUsableForReference(const geometry_msgs::msg::PoseStamped &pose) const;
  bool isHardBlockage(Traversability status) const;
  bool isRepairableBlockage(Traversability status) const;

  size_t findNearestPathIndex(const nav_msgs::msg::Path &path,
                              const geometry_msgs::msg::PoseStamped &pose) const;
  bool findFirstBlockage(const nav_msgs::msg::Path &path,
                         const geometry_msgs::msg::PoseStamped &start_pose,
                         size_t start_index,
                         double max_distance,
                         PathBlockage &blockage) const;
  bool findFirstSoftTerrain(const nav_msgs::msg::Path &path,
                            const geometry_msgs::msg::PoseStamped &start_pose,
                            size_t start_index,
                            double max_distance,
                            PathBlockage &soft_terrain) const;
  bool buildRepairedPath(const nav_msgs::msg::Path &original_path,
                         const geometry_msgs::msg::PoseStamped &robot_pose,
                         size_t start_index,
                         const PathBlockage &blockage,
                         nav_msgs::msg::Path &repaired_path,
                         geometry_msgs::msg::PoseStamped &rejoin_pose);

  nav_msgs::msg::Path buildTrimmedOriginalPath(const nav_msgs::msg::Path &original_path,
                                               const geometry_msgs::msg::PoseStamped &robot_pose,
                                               size_t start_index) const;
  bool buildSafePrefixPath(const nav_msgs::msg::Path &original_path,
                           const geometry_msgs::msg::PoseStamped &robot_pose,
                           size_t start_index,
                           const PathBlockage &blockage,
                           nav_msgs::msg::Path &prefix_path) const;
  nav_msgs::msg::Path composeRepairedPath(const nav_msgs::msg::Path &repair_segment,
                                          const nav_msgs::msg::Path &original_path,
                                          const geometry_msgs::msg::PoseStamped &robot_pose,
                                          size_t start_index,
                                          size_t rejoin_index) const;
  geometry_msgs::msg::PoseStamped makeReferenceLayerStartPose(
      const nav_msgs::msg::Path &original_path,
      const geometry_msgs::msg::PoseStamped &robot_pose,
      size_t start_index) const;

  double minRequiredClearance() const;
  double pathLength(const nav_msgs::msg::Path &path) const;
  double sampledPathCost(const nav_msgs::msg::Path &path,
                         const geometry_msgs::msg::PoseStamped &start_pose,
                         size_t start_index,
                         double max_distance) const;
  double maxSampledPathDistanceXY(const nav_msgs::msg::Path &a,
                                  const nav_msgs::msg::Path &b) const;
  bool isPathMateriallyDifferent(const nav_msgs::msg::Path &path) const;
  bool shouldPublishPath(const nav_msgs::msg::Path &path,
                         OutputMode mode,
                         const geometry_msgs::msg::PoseStamped *rejoin_pose,
                         const rclcpp::Time &publish_time) const;
  void rememberPublishedPath(const nav_msgs::msg::Path &path,
                             OutputMode mode,
                             const geometry_msgs::msg::PoseStamped *rejoin_pose,
                             const rclcpp::Time &publish_time);
  void appendPoseIfSeparated(nav_msgs::msg::Path &path,
                             const geometry_msgs::msg::PoseStamped &pose,
                             double min_separation) const;
  bool publishPath(const nav_msgs::msg::Path &path,
                   OutputMode mode,
                   const PathBlockage *blockage,
                   const geometry_msgs::msg::PoseStamped *rejoin_pose = nullptr);

  std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d_ros_;
  std::shared_ptr<global_planner::GlobalPlanner> global_planner_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr repaired_path_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  mutable std::mutex path_mutex_;
  nav_msgs::msg::Path original_path_;
  bool have_original_path_{false};
  bool disabled_{false};
  size_t source_path_revision_{0};
  size_t processed_source_path_revision_{0};

  nav_msgs::msg::Path last_published_path_;
  geometry_msgs::msg::PoseStamped last_published_rejoin_pose_;
  OutputMode last_published_mode_{OutputMode::ClearOriginal};
  rclcpp::Time last_publish_time_;
  bool have_last_published_path_{false};
  bool have_last_published_rejoin_pose_{false};
  int blocked_observation_count_{0};
  int clear_observation_count_{0};
  bool confirmed_blocked_{false};

  std::string input_path_topic_;
  std::string output_path_topic_;
  std::string static_layer_name_{"map"};
  std::string dynamic_layer_name_{"lidar"};
  double publish_frequency_{2.0};
  double repair_window_distance_{5.0};
  double rejoin_search_distance_{8.0};
  double rejoin_candidate_step_{0.5};
  double edge_check_sample_step_{0.1};
  double min_rejoin_distance_after_block_{1.0};
  double ground_lookup_radius_{0.35};
  double min_clearance_margin_{0.0};
  double no_support_block_distance_{0.4};
  double min_pose_separation_{0.05};
  double min_stop_path_distance_{0.25};
  double min_publish_interval_{0.8};
  double path_start_change_threshold_{0.25};
  double path_goal_change_threshold_{0.2};
  double path_length_change_threshold_{0.5};
  double rejoin_change_threshold_{0.3};
  double keepalive_publish_interval_{5.0};
  double soft_terrain_repair_min_cost_{0.25};
  double soft_terrain_repair_min_improvement_{0.2};
  double path_soft_cost_sample_step_{0.1};
  double soft_terrain_cost_weight_{1.0};
  int blocked_confirm_count_{2};
  int clear_confirm_count_{2};
  bool enable_stop_prefix_{true};
  bool validate_repaired_path_{true};
  bool repair_dynamic_blockage_{true};
  bool repair_soft_terrain_{true};
  bool publish_original_when_repair_fails_{false};
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER_EGO_REFERENCE_PATH_PROVIDER_H_
