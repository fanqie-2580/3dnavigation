/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

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
#include <global_planner/dynamic_window_aware_global_planner.h>
#include <algorithm>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

namespace global_planner
{

DWA_GlobalPlanner::DWA_GlobalPlanner(const std::string& name)
    : Node(name) 
{
  clock_ = this->get_clock();
  last_selected_pivot_ = -1;
  pivot_hold_counter_ = 0;
  has_last_selected_pivot_ = false;
}

rclcpp_action::GoalResponse DWA_GlobalPlanner::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::GetPlan::Goal> goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DWA_GlobalPlanner::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DWA_GlobalPlanner::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  rclcpp::Rate r(20);
  while (is_active(current_handle_)) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Wait for current handle to join");
    r.sleep();
  }
  current_handle_.reset();
  current_handle_ = goal_handle;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&DWA_GlobalPlanner::makePlan, this, std::placeholders::_1), goal_handle}.detach();
}
  
void DWA_GlobalPlanner::initial(const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d, 
                                    const std::shared_ptr<global_planner::GlobalPlanner>& global_planner){

  perception_3d_ros_ = perception_3d;
  global_planner_ = global_planner;
  robot_frame_ = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  pcl_global_path_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  declare_parameter("look_ahead_distance", rclcpp::ParameterValue(5.0));
  this->get_parameter("look_ahead_distance", look_ahead_distance_);
  RCLCPP_INFO(this->get_logger(), "look_ahead_distance: %.2f", look_ahead_distance_);    
  declare_parameter("recompute_frequency", rclcpp::ParameterValue(10.0));
  this->get_parameter("recompute_frequency", recompute_frequency_);
  RCLCPP_INFO(this->get_logger(), "recompute_frequency: %.2f", recompute_frequency_);

  // [FARStyleGoalSelect] Robust local-goal picking near dynamic obstacles.
  declare_parameter("goal_check_radius", rclcpp::ParameterValue(0.25));
  this->get_parameter("goal_check_radius", goal_check_radius_);
  declare_parameter("goal_clearance_margin", rclcpp::ParameterValue(0.05));
  this->get_parameter("goal_clearance_margin", goal_clearance_margin_);
  declare_parameter("goal_blocked_ratio_threshold", rclcpp::ParameterValue(0.60));
  this->get_parameter("goal_blocked_ratio_threshold", goal_blocked_ratio_threshold_);
  declare_parameter("goal_min_neighbor_points", rclcpp::ParameterValue(1));
  this->get_parameter("goal_min_neighbor_points", goal_min_neighbor_points_);
  declare_parameter("blocked_goal_search_step", rclcpp::ParameterValue(0.8));
  this->get_parameter("blocked_goal_search_step", blocked_goal_search_step_);
  declare_parameter("blocked_goal_max_extra_lookahead", rclcpp::ParameterValue(12.0));
  this->get_parameter("blocked_goal_max_extra_lookahead", blocked_goal_max_extra_lookahead_);
  declare_parameter("goal_backward_search_points", rclcpp::ParameterValue(30));
  this->get_parameter("goal_backward_search_points", goal_backward_search_points_);
  declare_parameter("goal_hold_cycles", rclcpp::ParameterValue(5));
  this->get_parameter("goal_hold_cycles", goal_hold_cycles_);
  RCLCPP_INFO(
    this->get_logger(),
    "goal_check_radius: %.2f, goal_clearance_margin: %.2f, blocked_ratio_th: %.2f, min_neighbors: %d, search_step: %.2f, max_extra_lookahead: %.2f, backward_search_points: %d, hold_cycles: %d",
    goal_check_radius_,
    goal_clearance_margin_,
    goal_blocked_ratio_threshold_,
    goal_min_neighbor_points_,
    blocked_goal_search_step_,
    blocked_goal_max_extra_lookahead_,
    goal_backward_search_points_,
    goal_hold_cycles_);
  
  pub_path_ = this->create_publisher<nav_msgs::msg::Path>("awared_global_path", 1);

  action_server_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //@ Callback should be the last, because all parameters should be ready before cb
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = action_server_group_;

  auto loop_time = std::chrono::milliseconds(int(1000/recompute_frequency_));
  threading_timer_ = this->create_wall_timer(loop_time, std::bind(&DWA_GlobalPlanner::determineDWAPlan, this), action_server_group_);
  threading_timer_->cancel();
  //@Create action server
  this->action_server_global_planner_ = rclcpp_action::create_server<dddmr_sys_core::action::GetPlan>(
    this,
    "/get_dwa_plan",
    std::bind(&DWA_GlobalPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&DWA_GlobalPlanner::handle_cancel, this, std::placeholders::_1),
    std::bind(&DWA_GlobalPlanner::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_);
  
}

DWA_GlobalPlanner::~DWA_GlobalPlanner(){
  action_server_global_planner_.reset();
}

bool DWA_GlobalPlanner::isNewGoal(){
  if(new_goal_.pose.position.x==current_goal_.pose.position.x && 
      new_goal_.pose.position.y==current_goal_.pose.position.y && 
        new_goal_.pose.position.z==current_goal_.pose.position.z && 
          new_goal_.pose.orientation.x==current_goal_.pose.orientation.x && 
            new_goal_.pose.orientation.y==current_goal_.pose.orientation.y && 
              new_goal_.pose.orientation.z==current_goal_.pose.orientation.z && 
                new_goal_.pose.orientation.w==current_goal_.pose.orientation.w)
     {
      
      return false;
     }
  
  RCLCPP_INFO(this->get_logger(), "Received new goal at: %.2f, %.2f, %.2f", new_goal_.pose.position.x, new_goal_.pose.position.y, new_goal_.pose.position.z);
  return true;
}

void DWA_GlobalPlanner::makePlan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle){
  
  new_goal_ = goal_handle->get_goal()->goal;

  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Waiting for static layer");
    threading_timer_->cancel();
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    goal_handle->abort(result);
    return;
  }

  if(!goal_handle->get_goal()->activate_threading){
    threading_timer_->cancel();
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    goal_handle->succeed(result);
    return;
  }
  
  if(isNewGoal()){
    
    geometry_msgs::msg::PoseStamped start;
    perception_3d_ros_->getGlobalPose(start);

    global_path_ = global_planner_->makeROSPlan(start, goal_handle->get_goal()->goal);
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    result->path = global_path_;

    if(global_path_.poses.empty()){
      const double goal_delta_xy = std::hypot(
        new_goal_.pose.position.x - current_goal_.pose.position.x,
        new_goal_.pose.position.y - current_goal_.pose.position.y);
      const bool can_reuse_previous_path =
        !global_dwa_path_.poses.empty() &&
        goal_delta_xy <= 1.0;
      if(can_reuse_previous_path){
        current_goal_ = new_goal_;
        global_path_ = global_dwa_path_;
        result->path = global_dwa_path_;
        goal_handle->succeed(result);
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *clock_,
          1000,
          "Global path failed for nearby goal (delta=%.2f m), reusing previous DWA path for continuity.",
          goal_delta_xy);
        threading_timer_->reset();
      }
      else{
        goal_handle->abort(result);
      }
    }
    else{
      current_goal_ = new_goal_;
      result->path = global_path_;
      goal_handle->succeed(result);
      
      //@move global plan to pcl
      pcl_global_path_->points.clear();
      for(auto it=global_path_.poses.begin(); it!=global_path_.poses.end(); it++){
        pcl::PointXYZ pt;
        pt.x = (*it).pose.position.x; pt.y = (*it).pose.position.y; pt.z = (*it).pose.position.z;
        pcl_global_path_->push_back(pt);
      }
      //@ generate kd-tree
      kdtree_global_path_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
      kdtree_global_path_->setInputCloud(pcl_global_path_); 
      has_last_selected_pivot_ = false;
      last_selected_pivot_ = -1;
      pivot_hold_counter_ = 0;

      threading_timer_->reset();
    }
    pub_path_->publish(global_path_);
  }
  else{
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    result->path = global_dwa_path_;
    goal_handle->succeed(result);
    pub_path_->publish(global_dwa_path_);
  }
  
}

bool DWA_GlobalPlanner::isPivotClear(int pivot_idx, int &blocked_count, int &sample_count) const
{
  blocked_count = 0;
  sample_count = 0;
  if(!perception_3d_ros_ ||
     !perception_3d_ros_->getSharedDataPtr()->kdtree_ground_ ||
     !pcl_global_path_ ||
     pcl_global_path_->empty())
  {
    return false;
  }
  if(pivot_idx < 0 || pivot_idx >= static_cast<int>(pcl_global_path_->points.size())){
    return false;
  }

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  pcl::PointXYZI ipt;
  ipt.x = pcl_global_path_->points[pivot_idx].x;
  ipt.y = pcl_global_path_->points[pivot_idx].y;
  ipt.z = pcl_global_path_->points[pivot_idx].z;

  const double check_radius = std::max(0.05, goal_check_radius_);
  perception_3d_ros_->getSharedDataPtr()->kdtree_ground_->radiusSearch(
    ipt, check_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
  sample_count = static_cast<int>(pointIdxRadiusSearch.size());
  if(sample_count <= 0){
    return false;
  }

  const double clearance_threshold = perception_3d_ros_->getGlobalUtils()->getInscribedRadius() +
                                     std::max(0.0, goal_clearance_margin_);
  for(const int idx : pointIdxRadiusSearch){
    if(perception_3d_ros_->get_min_dGraphValue(idx) < clearance_threshold){
      blocked_count++;
    }
  }

  // Sparse neighborhood: accept if no lethal neighbors are observed.
  if(sample_count < std::max(1, goal_min_neighbor_points_)){
    return blocked_count == 0;
  }

  const double blocked_ratio = static_cast<double>(blocked_count) / static_cast<double>(sample_count);
  return blocked_ratio <= std::max(0.0, std::min(1.0, goal_blocked_ratio_threshold_));
}

int DWA_GlobalPlanner::advanceIndexByDistance(int start_idx, double distance_m) const
{
  if(!pcl_global_path_ || pcl_global_path_->empty()){
    return start_idx;
  }
  const int last_idx = static_cast<int>(pcl_global_path_->points.size()) - 1;
  start_idx = std::max(0, std::min(start_idx, last_idx));

  if(distance_m <= 0.0 || start_idx >= last_idx){
    return start_idx;
  }

  double acc_dist = 0.0;
  int pivot = start_idx;
  pcl::PointXYZ last_point = pcl_global_path_->points[pivot];
  while(acc_dist < distance_m && pivot < last_idx){
    const pcl::PointXYZ next_point = pcl_global_path_->points[pivot + 1];
    const double dx = next_point.x - last_point.x;
    const double dy = next_point.y - last_point.y;
    const double dz = next_point.z - last_point.z;
    acc_dist += std::sqrt(dx * dx + dy * dy + dz * dz);
    last_point = next_point;
    pivot++;
  }
  return std::max(0, std::min(pivot, last_idx));
}

int DWA_GlobalPlanner::selectRobustPivotIndex(int nearest_idx)
{
  const int last_idx = static_cast<int>(pcl_global_path_->points.size()) - 1;
  nearest_idx = std::max(0, std::min(nearest_idx, last_idx));

  int best_idx = advanceIndexByDistance(nearest_idx, std::max(0.1, look_ahead_distance_));
  const int seed_idx = best_idx;
  bool found_clear = false;
  double best_clear_score = -std::numeric_limits<double>::infinity();
  int best_clear_idx = best_idx;
  double best_unclear_score = std::numeric_limits<double>::infinity();
  int best_unclear_idx = best_idx;

  auto evalCandidate = [&](int idx) {
    idx = std::max(0, std::min(idx, last_idx));
    int b = 0;
    int s = 0;
    const bool clear = isPivotClear(idx, b, s);
    if(clear){
      // [FARStyleGoalSelect] Keep scanning forward candidates and pick the one with larger progress.
      const double progress_score = static_cast<double>(idx - nearest_idx);
      const double hold_bonus = (has_last_selected_pivot_ && idx == last_selected_pivot_) ? 0.5 : 0.0;
      const double clear_score = progress_score + hold_bonus;
      if(!found_clear || clear_score > best_clear_score){
        best_clear_score = clear_score;
        best_clear_idx = idx;
      }
      found_clear = true;
      return;
    }

    // Pick the least-bad pivot when every candidate is constrained.
    const double ratio = (s > 0) ? static_cast<double>(b) / static_cast<double>(s) : 2.0;
    const double sparse_penalty = (s <= 0) ? 2.0 : ((s < std::max(1, goal_min_neighbor_points_)) ? 0.25 : 0.0);
    const double backward_penalty = (idx < nearest_idx) ? 1.0 : 0.0;
    const double score = ratio + sparse_penalty + backward_penalty;
    if(score + 1e-6 < best_unclear_score ||
       (std::fabs(score - best_unclear_score) <= 1e-6 && idx > best_unclear_idx))
    {
      best_unclear_score = score;
      best_unclear_idx = idx;
    }
  };

  evalCandidate(seed_idx);

  const double search_step = std::max(0.1, blocked_goal_search_step_);
  const double extra_limit = std::max(0.0, blocked_goal_max_extra_lookahead_);
  for(double extra = search_step; extra <= extra_limit; extra += search_step){
    const int idx = advanceIndexByDistance(nearest_idx, std::max(0.1, look_ahead_distance_) + extra);
    evalCandidate(idx);
  }

  if(!found_clear){
    const int back_search = std::max(1, goal_backward_search_points_);
    const int search_min = std::max(nearest_idx, seed_idx - back_search);
    for(int idx = seed_idx; idx >= search_min; idx--){
      evalCandidate(idx);
    }
  }

  if(!found_clear && has_last_selected_pivot_){
    int last_blocked = 0;
    int last_samples = 0;
    if(last_selected_pivot_ >= nearest_idx &&
       last_selected_pivot_ <= last_idx &&
       isPivotClear(last_selected_pivot_, last_blocked, last_samples))
    {
      best_clear_idx = last_selected_pivot_;
      found_clear = true;
    }
  }

  best_idx = found_clear ? best_clear_idx : best_unclear_idx;

  if(has_last_selected_pivot_){
    const bool backward_jump = best_idx < last_selected_pivot_ && last_selected_pivot_ >= nearest_idx;
    if(backward_jump && pivot_hold_counter_ < std::max(0, goal_hold_cycles_)){
      int last_blocked = 0;
      int last_samples = 0;
      if(isPivotClear(last_selected_pivot_, last_blocked, last_samples)){
        best_idx = last_selected_pivot_;
        pivot_hold_counter_++;
      } else {
        pivot_hold_counter_ = 0;
      }
    } else if(best_idx != last_selected_pivot_){
      pivot_hold_counter_ = 0;
    }
  }

  has_last_selected_pivot_ = true;
  last_selected_pivot_ = best_idx;
  return best_idx;
}

void DWA_GlobalPlanner::determineDWAPlan(){

  if(!kdtree_global_path_ || !pcl_global_path_ || pcl_global_path_->empty() || global_path_.poses.empty()){
    return;
  }

  geometry_msgs::msg::PoseStamped start;
  perception_3d_ros_->getGlobalPose(start);
  pcl::PointXYZ start_pt;
  start_pt.x = start.pose.position.x; start_pt.y = start.pose.position.y; start_pt.z = start.pose.position.z;
  std::vector<float> resultant_distances;
  std::vector<int> indices;
  kdtree_global_path_->nearestKSearch(start_pt, 1, indices, resultant_distances);

  if(indices.empty()){
    RCLCPP_INFO(this->get_logger(), "k-NN search fail, something wrong.");
    return;
  }
  //@ protect kdtree_ground
  std::unique_lock<std::recursive_mutex> lock(perception_3d_ros_->getSharedDataPtr()->ground_kdtree_cb_mutex_);
  const int nearest_idx = indices[0];
  int dwa_pivot = selectRobustPivotIndex(nearest_idx);
  const int path_last_idx = static_cast<int>(pcl_global_path_->points.size()) - 1;
  dwa_pivot = std::max(0, std::min(dwa_pivot, path_last_idx));

  int blocked_count = 0;
  int sample_count = 0;
  const bool pivot_clear = isPivotClear(dwa_pivot, blocked_count, sample_count);
  if(!pivot_clear){
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *clock_,
      1000,
      "DWA pivot remains constrained at idx=%d (blocked=%d, samples=%d). Falling back to nearest-safe behavior.",
      dwa_pivot,
      blocked_count,
      sample_count);
  }
  if(dwa_pivot >= path_last_idx){
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *clock_,
      5000,
      "DWA goal reach the global path end at: %.2f, %.2f, %.2f",
      pcl_global_path_->points[dwa_pivot].x,
      pcl_global_path_->points[dwa_pivot].y,
      pcl_global_path_->points[dwa_pivot].z);
  }

  geometry_msgs::msg::PoseStamped dwa_goal;
  dwa_goal.header.frame_id = global_frame_;
  dwa_goal.pose.position.x = pcl_global_path_->points[dwa_pivot].x;
  dwa_goal.pose.position.y = pcl_global_path_->points[dwa_pivot].y;
  dwa_goal.pose.position.z = pcl_global_path_->points[dwa_pivot].z;
  
  // Keep high-frequency DWA refinement lightweight; robust endpoint fallback is reserved for full-goal planning.
  nav_msgs::msg::Path dwa_path = global_planner_->makeROSPlan(start, dwa_goal);
  for(size_t i = static_cast<size_t>(dwa_pivot); i < global_path_.poses.size(); i++){
    dwa_path.poses.push_back(global_path_.poses[i]);
  }
  if(dwa_path.poses.empty()){
    for(size_t i = static_cast<size_t>(dwa_pivot); i < global_path_.poses.size(); i++){
      dwa_path.poses.push_back(global_path_.poses[i]);
    }
  }
  if(dwa_path.poses.empty()){
    dwa_path = global_path_;
  }
  global_dwa_path_ = dwa_path;
}

}
