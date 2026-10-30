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
#include <p2p_move_base/p2p_move_base.h>
#include <cmath>

namespace p2p_move_base
{

P2PMoveBase::P2PMoveBase(std::string name): Node(name)
{
  name_ = name;
  clock_ = this->get_clock();
}

rclcpp_action::GoalResponse P2PMoveBase::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::PToPMoveBase::Goal> goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse P2PMoveBase::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void P2PMoveBase::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle)
{

  if (is_active(current_handle_)){
    RCLCPP_INFO(this->get_logger(), "An older goal is active, cancelling current one.");
    auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
    current_handle_->abort(result);
    return;
  }
  else{
    current_handle_ = goal_handle;
  }
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&P2PMoveBase::executeCb, this, std::placeholders::_1), goal_handle}.detach();
}

void P2PMoveBase::initial(const std::shared_ptr<local_planner::Local_Planner>& lp
                    ,const std::shared_ptr<p2p_move_base::P2PGlobalPlanManager>& gpm){
  
  LP_ = lp;
  GPM_ = gpm;

  FSM_ = std::make_shared<p2p_move_base::FSM>(this->get_node_logging_interface(), this->get_node_parameters_interface());

  // [SmoothOmniGO2] Use configurable control trajectory generator (e.g. omni_drive_simple for GO2).
  this->declare_parameter("control_trajectory_generator_name", rclcpp::ParameterValue("differential_drive_simple"));
  this->get_parameter("control_trajectory_generator_name", control_trajectory_generator_name_);
  RCLCPP_INFO(this->get_logger(), "control_trajectory_generator_name: %s", control_trajectory_generator_name_.c_str());

  // [SmoothOmniGO2] Use omni fallback only when blocked/failed, keeping straight tracking in normal scenes.
  this->declare_parameter("blocked_control_trajectory_generator_name", rclcpp::ParameterValue("omni_drive_simple"));
  this->get_parameter("blocked_control_trajectory_generator_name", blocked_control_trajectory_generator_name_);
  // [FARStyleBypass] If evasive fallback becomes pure side-shift, try secondary generator for forward progress.
  this->declare_parameter("blocked_control_secondary_generator_name", rclcpp::ParameterValue("omni_drive_simple"));
  this->get_parameter("blocked_control_secondary_generator_name", blocked_control_secondary_generator_name_);
  this->declare_parameter("enable_blocked_control_fallback", rclcpp::ParameterValue(true));
  this->get_parameter("enable_blocked_control_fallback", enable_blocked_control_fallback_);
  this->declare_parameter("enable_blocked_control_secondary_fallback", rclcpp::ParameterValue(true));
  this->get_parameter("enable_blocked_control_secondary_fallback", enable_blocked_control_secondary_fallback_);
  this->declare_parameter("blocked_fallback_min_forward_speed", rclcpp::ParameterValue(0.08));
  this->get_parameter("blocked_fallback_min_forward_speed", blocked_fallback_min_forward_speed_);
  RCLCPP_INFO(
    this->get_logger(),
    "blocked_control_trajectory_generator_name: %s, secondary: %s, enable_blocked_control_fallback: %d, enable_blocked_control_secondary_fallback: %d, blocked_fallback_min_forward_speed: %.2f",
    blocked_control_trajectory_generator_name_.c_str(),
    blocked_control_secondary_generator_name_.c_str(),
    enable_blocked_control_fallback_,
    enable_blocked_control_secondary_fallback_,
    blocked_fallback_min_forward_speed_);

  // [SmoothOmniGO2] Skip heading alignment after in-motion replanning to avoid rotate-then-go behavior.
  this->declare_parameter("skip_align_heading_on_replan", rclcpp::ParameterValue(true));
  this->get_parameter("skip_align_heading_on_replan", skip_align_heading_on_replan_);
  RCLCPP_INFO(this->get_logger(), "skip_align_heading_on_replan: %d", skip_align_heading_on_replan_);
  // [FARStyleBypass] If control generator is omni, allow heading-align state to move-and-turn instead of rotate-in-place.
  this->declare_parameter("enable_omni_align_heading_bypass", rclcpp::ParameterValue(true));
  this->get_parameter("enable_omni_align_heading_bypass", enable_omni_align_heading_bypass_);
  RCLCPP_INFO(this->get_logger(), "enable_omni_align_heading_bypass: %d", enable_omni_align_heading_bypass_);

  // [SmoothOmniGO2] Compensate GO2 low-speed deadzone where robot leans but does not move.
  this->declare_parameter("enable_min_effective_velocity", rclcpp::ParameterValue(false));
  this->get_parameter("enable_min_effective_velocity", enable_min_effective_velocity_);
  this->declare_parameter("min_effective_vx", rclcpp::ParameterValue(0.0));
  this->get_parameter("min_effective_vx", min_effective_vx_);
  this->declare_parameter("min_effective_vy", rclcpp::ParameterValue(0.0));
  this->get_parameter("min_effective_vy", min_effective_vy_);
  RCLCPP_INFO(this->get_logger(), "enable_min_effective_velocity: %d, min_effective_vx: %.3f, min_effective_vy: %.3f",
              enable_min_effective_velocity_, min_effective_vx_, min_effective_vy_);

  // [GO2ReactiveBypassV5] During in-motion replanning, keep command continuity for a short timeout.
  this->declare_parameter("keep_last_cmd_during_replan", rclcpp::ParameterValue(true));
  this->get_parameter("keep_last_cmd_during_replan", keep_last_cmd_during_replan_);
  this->declare_parameter("replan_keep_cmd_timeout_sec", rclcpp::ParameterValue(0.6));
  this->get_parameter("replan_keep_cmd_timeout_sec", replan_keep_cmd_timeout_sec_);
  RCLCPP_INFO(this->get_logger(), "keep_last_cmd_during_replan: %d, replan_keep_cmd_timeout_sec: %.2f",
              keep_last_cmd_during_replan_, replan_keep_cmd_timeout_sec_);

  has_controlled_once_ = false;
  replan_requested_from_motion_ = false;
  has_last_cmd_ = false;
  last_cmd_vx_ = 0.0;
  last_cmd_vy_ = 0.0;
  last_cmd_w_ = 0.0;
  last_cmd_time_ = clock_->now();
  
  if(FSM_->use_twist_stamped_){
    stamped_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped", 1);
  }
  else{
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  }
  

  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_server_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //@Initialize transform listener and broadcaster
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);
  
  recovery_behaviors_client_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  recovery_behaviors_client_ptr_ = rclcpp_action::create_client<dddmr_sys_core::action::RecoveryBehaviors>(
      this,
      "recovery_behaviors", recovery_behaviors_client_group_);

  //@Create action server
  action_server_p2p_move_base_ = rclcpp_action::create_server<dddmr_sys_core::action::PToPMoveBase>(
    this,
    "/p2p_move_base",
    std::bind(&P2PMoveBase::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&P2PMoveBase::handle_cancel, this, std::placeholders::_1),
    std::bind(&P2PMoveBase::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_);

  RCLCPP_INFO(this->get_logger(), "\033[1;32m---->\033[0m P2P move base launched.");

}

P2PMoveBase::~P2PMoveBase(){
  FSM_.reset();
  tf2Buffer_.reset();
  tfl_.reset();
  LP_.reset();
  GPM_.reset();
}

bool P2PMoveBase::isQuaternionValid(const geometry_msgs::msg::Quaternion& q){
  //first we need to check if the quaternion has nan's or infs
  if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
    RCLCPP_ERROR(this->get_logger(), "Quaternion has nans or infs... discarding as a navigation goal");
    return false;
  }

  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

  //next, we need to check if the length of the quaternion is close to zero
  if(tf_q.length2() < 1e-6){
    RCLCPP_ERROR(this->get_logger(), "Quaternion has length close to zero... discarding as navigation goal");
    return false;
  }

  //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
  tf_q.normalize();

  tf2::Vector3 up(0, 0, 1);

  double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

  if(fabs(dot - 1) > 1e-3){
    RCLCPP_ERROR(this->get_logger(), "Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
    return false;
  }

  return true;
}

void P2PMoveBase::publishZeroVelocity(){
  has_last_cmd_ = false;
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  if(FSM_->use_twist_stamped_){
    geometry_msgs::msg::TwistStamped stamped_cmd_vel;
    stamped_cmd_vel.header.frame_id = FSM_->twist_frame_id_;
    stamped_cmd_vel.header.stamp = clock_->now();
    stamped_cmd_vel.twist = cmd_vel;
    stamped_cmd_vel_pub_->publish(stamped_cmd_vel);
  }
  else{
    cmd_vel_pub_->publish(cmd_vel);
  }
}

void P2PMoveBase::publishVelocity(double vx, double vy, double angular_z){
  // [GO2SmoothBypassV2] Lift tiny commands by translational vector norm to avoid artificial diagonal drift.
  if(enable_min_effective_velocity_){
    constexpr double eps = 1e-5;
    const double trans_speed = std::hypot(vx, vy);
    const double min_effective_trans = std::max(min_effective_vx_, min_effective_vy_);
    if(trans_speed > eps && trans_speed < min_effective_trans){
      const double scale = min_effective_trans / trans_speed;
      vx *= scale;
      vy *= scale;
    }
  }

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = vx;
  cmd_vel.linear.y = vy;
  cmd_vel.angular.z = angular_z;
  // [GO2ReactiveBypassV5] Cache latest command for short replan continuity.
  last_cmd_vx_ = vx;
  last_cmd_vy_ = vy;
  last_cmd_w_ = angular_z;
  last_cmd_time_ = clock_->now();
  has_last_cmd_ = true;
  if(FSM_->use_twist_stamped_){
    geometry_msgs::msg::TwistStamped stamped_cmd_vel;
    stamped_cmd_vel.header.frame_id = FSM_->twist_frame_id_;
    stamped_cmd_vel.header.stamp = clock_->now();
    stamped_cmd_vel.twist = cmd_vel;
    stamped_cmd_vel_pub_->publish(stamped_cmd_vel);
  }
  else{
    cmd_vel_pub_->publish(cmd_vel);
  }
}

bool P2PMoveBase::shouldTrySecondaryBlockedFallback(const base_trajectory::Trajectory & traj) const{
  if(!enable_blocked_control_secondary_fallback_){
    return false;
  }
  if(blocked_control_secondary_generator_name_.empty() ||
     blocked_control_secondary_generator_name_ == blocked_control_trajectory_generator_name_){
    return false;
  }

  const double min_forward = std::max(0.0, blocked_fallback_min_forward_speed_);
  const double lateral_motion = std::abs(traj.yv_);
  const double lateral_gate = std::max(0.18, min_effective_vy_);
  return (std::abs(traj.xv_) < min_forward) && (lateral_motion > lateral_gate);
}

void P2PMoveBase::executeCb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle)
{
  auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
  auto move_base_goal = goal_handle->get_goal();

  if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
    RCLCPP_WARN(this->get_logger(),"Aborting on goal because it was sent with an invalid quaternion");
    goal_handle->abort(result);
    publishZeroVelocity();
    return;
  }

  rclcpp::Rate r(FSM_->controller_frequency_);

  //@ if we dont initialize oscillation pose here, the first controlling entry will cause recovery behavior.
  //@ the rclcpp::Time initial are all done in FSM class
  FSM_->initialParams(LP_->getGlobalPose(), clock_->now());
  // [SmoothOmniGO2] Reset runtime replanning flags for each new goal.
  has_controlled_once_ = false;
  replan_requested_from_motion_ = false;
  FSM_->current_goal_ = move_base_goal->target_pose;
  GPM_->setGoal(FSM_->current_goal_);
  GPM_->resume();

  while(rclcpp::ok()){

    if(!goal_handle->is_active()){
      
      if(FSM_->isCurrentDecision("d_recovery_waitdone")){
        RCLCPP_INFO(this->get_logger(), "P2P is in recovery state, cancel recovery behaviors.");
        recovery_behaviors_client_ptr_->async_cancel_all_goals();
      }

      RCLCPP_INFO(this->get_logger(), "P2P move base preempted.");
      publishZeroVelocity();
      GPM_->stop();
      return;
    }

    if(goal_handle->is_canceling()){

      if(FSM_->isCurrentDecision("d_recovery_waitdone")){
        RCLCPP_INFO(this->get_logger(), "P2P is in recovery state, cancel recovery behaviors.");
        recovery_behaviors_client_ptr_->async_cancel_all_goals();
      }

      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "P2P move base cancelled.");
      publishZeroVelocity();
      GPM_->stop();
      return;
    }

    //the real work on pursuing a goal is done here
    bool done = executeCycle(goal_handle);
    
    auto feedback = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Feedback>();
    feedback->base_position = FSM_->global_pose_;
    feedback->last_decision = FSM_->getLastDecision();
    feedback->current_decision = FSM_->getCurrentDecision();
    goal_handle->publish_feedback(feedback);

    //if we're done, then we'll return from execute
    if(done){
      GPM_->stop();
      return;
    }
    
    r.sleep();

    //if(FSM_->isCurrentDecision("d_controlling") && r.cycleTime() > ros::Duration(1 / FSM_->controller_frequency_))
    //  ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", FSM_->controller_frequency_, r.cycleTime().toSec());
  }
  GPM_->stop();
}

bool P2PMoveBase::executeCycle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle){

    FSM_->global_pose_ = LP_->getGlobalPose();
    if(FSM_->getDistance(FSM_->global_pose_, FSM_->oscillation_pose_) >= FSM_->oscillation_distance_ ||
          FSM_->getAngle(FSM_->global_pose_, FSM_->oscillation_pose_) >= FSM_->oscillation_angle_)
    {
      FSM_->oscillation_pose_ = FSM_->global_pose_;
      FSM_->last_oscillation_reset_ = clock_->now();
    }


    if(FSM_->isCurrentDecision("d_initial")){
      FSM_->setDecision("d_planning");
    }

    else if(FSM_->isCurrentDecision("d_planning")){
      GPM_->queryThread();
      FSM_->setDecision("d_planning_waitdone");
      return false;
    }

    else if(FSM_->isCurrentDecision("d_planning_waitdone")){
      // [GO2ReactiveBypassV5] Keep command continuity while waiting async replan result.
      if(keep_last_cmd_during_replan_ && replan_requested_from_motion_ && has_last_cmd_){
        const double cmd_age = (clock_->now() - last_cmd_time_).seconds();
        if(cmd_age >= 0.0 && cmd_age <= replan_keep_cmd_timeout_sec_){
          if(FSM_->use_twist_stamped_){
            geometry_msgs::msg::TwistStamped stamped_cmd_vel;
            stamped_cmd_vel.header.frame_id = FSM_->twist_frame_id_;
            stamped_cmd_vel.header.stamp = clock_->now();
            stamped_cmd_vel.twist.linear.x = last_cmd_vx_;
            stamped_cmd_vel.twist.linear.y = last_cmd_vy_;
            stamped_cmd_vel.twist.angular.z = last_cmd_w_;
            stamped_cmd_vel_pub_->publish(stamped_cmd_vel);
          }
          else{
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = last_cmd_vx_;
            cmd_vel.linear.y = last_cmd_vy_;
            cmd_vel.angular.z = last_cmd_w_;
            cmd_vel_pub_->publish(cmd_vel);
          }
        }
      }
      
      //@If global planner keep return empty plan, we will enter this state for n seconds, then abort
      //@see: decision_planning
      std::vector<geometry_msgs::msg::PoseStamped> plan;
      if(GPM_->hasPlan()){
        GPM_->copyPlan(plan);
        //if the planner fails or returns a zero length plan, planning failed
        if(plan.empty()){
          RCLCPP_DEBUG(this->get_logger(), "Failed to find a plan to point (%.2f, %.2f, %.2f)", 
              FSM_->current_goal_.pose.position.x, FSM_->current_goal_.pose.position.y, FSM_->current_goal_.pose.position.z);
          FSM_->setDecision("d_planning");
        }
        else{
          FSM_->last_valid_plan_ = clock_->now();
          LP_->setPlan(plan);
          // [SmoothOmniGO2] Keep motion continuity after in-flight replanning.
          if(skip_align_heading_on_replan_ && has_controlled_once_ && replan_requested_from_motion_){
            FSM_->setDecision("d_controlling");
            RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 2000, "Replan succeeded during motion, skip heading align.");
          }
          else{
            FSM_->setDecision("d_align_heading");
          }
          replan_requested_from_motion_ = false;
        }
      }

      if((clock_->now()-FSM_->last_valid_plan_).seconds()>FSM_->planner_patience_){
        RCLCPP_WARN(this->get_logger(), "Time out to find a plan to point (%.2f, %.2f, %.2f)", 
            FSM_->current_goal_.pose.position.x, FSM_->current_goal_.pose.position.y, FSM_->current_goal_.pose.position.z);
        startRecoveryBehaviors();
        FSM_->setDecision("d_recovery_waitdone");
        return false;
      }
      return false;
    }
    
    else if(FSM_->isCurrentDecision("d_align_heading")){

      if(LP_->isInitialHeadingAligned()){
        FSM_->setDecision("d_controlling");  
      }
      else{

        if(FSM_->oscillation_patience_ > 0 && (clock_->now()-FSM_->last_oscillation_reset_).seconds() >= FSM_->oscillation_patience_){
          //@go to recovery
          auto diff = (clock_->now()-FSM_->last_oscillation_reset_).seconds();
          RCLCPP_WARN(this->get_logger(), "Oscillation time out is detected: %.2f secs for %.2f m.", diff, FSM_->getDistance(FSM_->global_pose_, FSM_->oscillation_pose_));
          startRecoveryBehaviors();
          FSM_->setDecision("d_recovery_waitdone");  
          return false;
        }
        
        base_trajectory::Trajectory best_traj;
        const bool use_omni_align_bypass =
          enable_omni_align_heading_bypass_ &&
          (control_trajectory_generator_name_.find("omni") != std::string::npos);
        dddmr_sys_core::PlannerState PS = dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL;
        if(use_omni_align_bypass){
          PS = LP_->computeVelocityCommand(control_trajectory_generator_name_, best_traj);
        }
        else{
          PS = LP_->computeVelocityCommand("differential_drive_rotate_shortest_angle", best_traj);
        }

        if(PS == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
          FSM_->last_valid_control_ = clock_->now();
          if(use_omni_align_bypass){
            has_controlled_once_ = true;
            replan_requested_from_motion_ = false;
            FSM_->setDecision("d_controlling");
            RCLCPP_WARN_THROTTLE(
              this->get_logger(),
              *clock_,
              1000,
              "Align-heading bypass active (omni). Switch to controlling with cmd(vx, vy, w)=(%.2f, %.2f, %.2f).",
              best_traj.xv_, best_traj.yv_, best_traj.thetav_);
          }
          else{
            FSM_->setDecision("d_align_heading");
          }
          publishVelocity(best_traj.xv_, best_traj.yv_, best_traj.thetav_);
          return false;
        }
        // [GO2ReactiveBypassV8] If rotate-shortest is blocked near obstacles, switch to evasive omni instead of deadlocking.
        else if(enable_blocked_control_fallback_ &&
                (PS == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL ||
                 PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT ||
                 PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING ||
                 PS == dddmr_sys_core::PlannerState::PRUNE_PLAN_FAIL))
        {
          base_trajectory::Trajectory blocked_traj;
          dddmr_sys_core::PlannerState blocked_ps = LP_->computeVelocityCommand(blocked_control_trajectory_generator_name_, blocked_traj);
          if(blocked_ps == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
            std::string selected_blocked_generator = blocked_control_trajectory_generator_name_;
            if(shouldTrySecondaryBlockedFallback(blocked_traj)){
              base_trajectory::Trajectory secondary_traj;
              dddmr_sys_core::PlannerState secondary_ps =
                LP_->computeVelocityCommand(blocked_control_secondary_generator_name_, secondary_traj);
              if(secondary_ps == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
                const double min_forward = std::max(0.0, blocked_fallback_min_forward_speed_);
                const bool blocked_has_progress = blocked_traj.xv_ >= min_forward;
                const bool secondary_has_progress = secondary_traj.xv_ >= min_forward;
                if((secondary_has_progress && !blocked_has_progress) ||
                   (secondary_traj.xv_ > blocked_traj.xv_ + 0.03))
                {
                  blocked_traj = secondary_traj;
                  selected_blocked_generator = blocked_control_secondary_generator_name_;
                }
              }
            }
            FSM_->last_valid_control_ = clock_->now();
            has_controlled_once_ = true;
            replan_requested_from_motion_ = true;
            FSM_->setDecision("d_controlling");
            publishVelocity(blocked_traj.xv_, blocked_traj.yv_, blocked_traj.thetav_);
            RCLCPP_WARN_THROTTLE(
              this->get_logger(), *clock_, 1000,
              "Align-heading rotate blocked, switch to blocked generator: %s | cmd(vx, vy, w)=(%.2f, %.2f, %.2f)",
              selected_blocked_generator.c_str(),
              blocked_traj.xv_, blocked_traj.yv_, blocked_traj.thetav_);
            return false;
          }
        }
        else if(PS == dddmr_sys_core::PlannerState::PERCEPTION_MALFUNCTION){
          RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Sensor data is out of date, we're not going to allow commanding of the base for safety");
          publishZeroVelocity();
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::TF_FAIL){
          RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Detect TF fail in local planner, we're not going to allow commanding of the base for safety");
          publishZeroVelocity();
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::PRUNE_PLAN_FAIL){
          //@ this assignment will allow at least one time planning query
          FSM_->last_valid_plan_ = clock_->now();
          publishZeroVelocity();
          FSM_->setDecision("d_planning");  
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL){
          //At least implement last_valid_control_ timeout to abort here
          //@ this assignment will allow at least one time planning query
          if((clock_->now() - FSM_->last_valid_control_).seconds() > FSM_->controller_patience_){
            RCLCPP_WARN(this->get_logger(), "Controller time out, go to recovery");
            startRecoveryBehaviors();
            FSM_->setDecision("d_recovery_waitdone");
          }
          else{
            FSM_->last_valid_plan_ = clock_->now();
            FSM_->setDecision("d_planning");  
          }
          publishZeroVelocity();
          return false;
        }

        else if(PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT || PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING){
          FSM_->last_valid_plan_ = clock_->now();
          FSM_->setDecision("d_planning");
          publishZeroVelocity();
          return false;
        }

        else{
          RCLCPP_FATAL(this->get_logger(), "Should not happen here, we did not catch dddmr_sys_core::PlannerState");
          publishZeroVelocity();
          return false;
        }
      }

    }

    else if(FSM_->isCurrentDecision("d_align_goal_heading")){
      if(LP_->isGoalHeadingAligned()){
        RCLCPP_INFO(this->get_logger(), "Goal reach.");
        auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
        goal_handle->succeed(result);
        publishZeroVelocity();
        return true;
      }
      else{

        if(FSM_->oscillation_patience_ >0 && (clock_->now()-FSM_->last_oscillation_reset_).seconds() >= FSM_->oscillation_patience_){
          //@go to recovery
          auto diff = (clock_->now()-FSM_->last_oscillation_reset_).seconds();
          RCLCPP_WARN(this->get_logger(), "Oscillation time out is detected: %.2f secs for %.2f m.", diff, FSM_->getDistance(FSM_->global_pose_, FSM_->oscillation_pose_));
          startRecoveryBehaviors();
          FSM_->setDecision("d_recovery_waitdone"); 
          return false;
        }
        
        base_trajectory::Trajectory best_traj;
        dddmr_sys_core::PlannerState PS = LP_->computeVelocityCommand("differential_drive_rotate_shortest_angle", best_traj);

        if(PS == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
          FSM_->last_valid_control_ = clock_->now();
          FSM_->setDecision("d_align_goal_heading");  
          publishVelocity(best_traj.xv_, best_traj.yv_, best_traj.thetav_);
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::PERCEPTION_MALFUNCTION){
          RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Sensor data is out of date, we're not going to allow commanding of the base for safety");
          publishZeroVelocity();
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::TF_FAIL){
          RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Detect TF fail in local planner, we're not going to allow commanding of the base for safety");
          publishZeroVelocity();
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::PRUNE_PLAN_FAIL){
          //@ this assignment will allow at least one time planning query
          FSM_->last_valid_plan_ = clock_->now();
          publishZeroVelocity();
          FSM_->setDecision("d_planning");  
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL ||
                PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT || 
                PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING){
          //At least implement last_valid_control_ timeout to abort here
          //@ this assignment will allow at least one time planning query
          if((clock_->now() - FSM_->last_valid_control_).seconds() > FSM_->controller_patience_){
            RCLCPP_WARN(this->get_logger(), "Controller time out, go to recovery");
            startRecoveryBehaviors();
            FSM_->setDecision("d_recovery_waitdone");
          }
          else{
            FSM_->setDecision("d_align_goal_heading");  
          }
          publishZeroVelocity();
          return false;
        }
        else{
          RCLCPP_FATAL(this->get_logger(), "Should not happen here, we did not catch dddmr_sys_core::PlannerState");
          publishZeroVelocity();
          return false;
        }
      }
    }

    else if(FSM_->isCurrentDecision("d_controlling")){

      //@Check is goal xy tolerance reach
      if(LP_->isGoalReached()){
        publishZeroVelocity();
        RCLCPP_INFO(this->get_logger(), "Goal xy tolerance reach, switch to align goal heading state.");
        FSM_->setDecision("d_align_goal_heading");  
        return false;
      }
      
      //@ update global plan
      if(GPM_->hasPlan()){
        std::vector<geometry_msgs::msg::PoseStamped> plan;
        GPM_->copyPlan(plan);
        LP_->setPlan(plan);
      }
      //@Behavior for oscillation here
      
      if(FSM_->oscillation_patience_ >0 && (clock_->now()-FSM_->last_oscillation_reset_).seconds() >= FSM_->oscillation_patience_){
        //@go to recovery
        auto diff = (clock_->now()-FSM_->last_oscillation_reset_).seconds();
        RCLCPP_WARN(this->get_logger(), "Oscillation time out is detected: %.2f secs for %.2f m.", diff, FSM_->getDistance(FSM_->global_pose_, FSM_->oscillation_pose_));
        startRecoveryBehaviors();
        FSM_->setDecision("d_recovery_waitdone");
        return false;
      }

      base_trajectory::Trajectory best_traj;
      dddmr_sys_core::PlannerState PS = LP_->computeVelocityCommand(control_trajectory_generator_name_, best_traj);
      bool used_blocked_fallback = false;

      // [SmoothOmniGO2] Hybrid control: keep differential tracking in clear space, switch to omni when blocked.
      if(enable_blocked_control_fallback_ &&
         (PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING ||
          PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT ||
          PS == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL))
      {
        base_trajectory::Trajectory blocked_traj;
        dddmr_sys_core::PlannerState blocked_ps = LP_->computeVelocityCommand(blocked_control_trajectory_generator_name_, blocked_traj);
        if(blocked_ps == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
          std::string selected_blocked_generator = blocked_control_trajectory_generator_name_;
          if(shouldTrySecondaryBlockedFallback(blocked_traj)){
            base_trajectory::Trajectory secondary_traj;
            dddmr_sys_core::PlannerState secondary_ps =
              LP_->computeVelocityCommand(blocked_control_secondary_generator_name_, secondary_traj);
            if(secondary_ps == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
              const double min_forward = std::max(0.0, blocked_fallback_min_forward_speed_);
              const bool blocked_has_progress = blocked_traj.xv_ >= min_forward;
              const bool secondary_has_progress = secondary_traj.xv_ >= min_forward;
              if((secondary_has_progress && !blocked_has_progress) ||
                 (secondary_traj.xv_ > blocked_traj.xv_ + 0.03))
              {
                blocked_traj = secondary_traj;
                selected_blocked_generator = blocked_control_secondary_generator_name_;
              }
            }
          }
          best_traj = blocked_traj;
          PS = dddmr_sys_core::PlannerState::TRAJECTORY_FOUND;
          used_blocked_fallback = true;
          replan_requested_from_motion_ = true;
          RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 1000,
            "Switch to blocked-control generator: %s | cmd(vx, vy, w)=(%.2f, %.2f, %.2f)",
            selected_blocked_generator.c_str(),
            best_traj.xv_, best_traj.yv_, best_traj.thetav_);
        }
      }

      if(PS == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
        FSM_->last_valid_control_ = clock_->now();
        FSM_->setDecision("d_controlling");  
        has_controlled_once_ = true;
        replan_requested_from_motion_ = used_blocked_fallback;
        publishVelocity(best_traj.xv_, best_traj.yv_, best_traj.thetav_);
        return false;
      }
      else if(PS == dddmr_sys_core::PlannerState::PERCEPTION_MALFUNCTION){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Sensor data is out of date, we're not going to allow commanding of the base for safety");
        publishZeroVelocity();
        return false;
      }
      else if(PS == dddmr_sys_core::PlannerState::TF_FAIL){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Detect TF fail in local planner, we're not going to allow commanding of the base for safety");
        publishZeroVelocity();
        return false;
      }
      else if(PS == dddmr_sys_core::PlannerState::PRUNE_PLAN_FAIL){
        //@ this assignment will allow at least one time planning query
        FSM_->last_valid_plan_ = clock_->now();
        publishZeroVelocity();
        replan_requested_from_motion_ = true;
        FSM_->setDecision("d_planning");
        return false;
      }
      else if(PS == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL){
        //At least implement last_valid_control_ timeout to abort here
        //@ this assignment will allow at least one time planning query
        if((clock_->now() - FSM_->last_valid_control_).seconds() > FSM_->controller_patience_){
          RCLCPP_WARN(this->get_logger(), "Controller time out, go to recovery");
          startRecoveryBehaviors();
          FSM_->setDecision("d_recovery_waitdone");
        }
        else{
          FSM_->last_valid_plan_ = clock_->now();
          replan_requested_from_motion_ = true;
          FSM_->setDecision("d_planning");
        }

        return false;
      }

      else if(PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING){
        FSM_->last_valid_plan_ = clock_->now();
        replan_requested_from_motion_ = true;
        FSM_->setDecision("d_planning");
        // [SmoothOmniGO2] Do not force stop on replanning; keep last command continuity.
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 1000, "Path conflict: keep moving while replanning.");
       	return false;
      }

      else if(PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT){
        // [FARSmoothBypass] Treat sudden wait-level blockage as a replanning event first to avoid hard stop.
        FSM_->last_valid_plan_ = clock_->now();
        replan_requested_from_motion_ = true;
        FSM_->setDecision("d_planning");
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 1000, "Path blocked(wait) detected, keep motion continuity and replan first.");
       	return false;
      }

      else{
        RCLCPP_FATAL(this->get_logger(), "Should not happen here, we did not catch dddmr_sys_core::PlannerState");
        publishZeroVelocity();
        return false;
      }

    }

    else if(FSM_->isCurrentDecision("d_recovery_waitdone")){
      
      if(is_recoverying_){
        return false;
      }
        

      if(FSM_->no_plan_recovery_count_>=FSM_->no_plan_retry_num_){
        RCLCPP_ERROR(this->get_logger(), "No global plan has been found even we try recovery %d times", FSM_->no_plan_recovery_count_);
        auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
        goal_handle->abort(result);
        publishZeroVelocity();
        return true;        
      }
      
      if(is_recoverying_succeed_){
        //we go to planning and we also need to count second recovery then abort
        RCLCPP_INFO(this->get_logger(), "Recovery succeed, back to planning state.");
        FSM_->no_plan_recovery_count_++;
        FSM_->last_valid_plan_ = clock_->now();
        FSM_->setDecision("d_planning");
        return false;  
      }
      else{
        //we may abort or go to another recovery
        RCLCPP_ERROR(this->get_logger(), "The potential collision has been detected when doing recovery.");
        auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
        goal_handle->abort(result);
        publishZeroVelocity();
        return true;  
      }
      
    }

    else if(FSM_->isCurrentDecision("d_waiting")){
      
      //if continue conflict over 10s,to recalculate the path
      if((clock_->now()-FSM_->waiting_time_).seconds() >= FSM_->waiting_patience_){ 
       	FSM_->last_valid_plan_ = clock_->now();
        FSM_->setDecision("d_planning");
        RCLCPP_WARN(this->get_logger(), "waiting time over %.2f,change to d_planning", FSM_->waiting_patience_);
        return false;
      }

      //@ update global plan
      if(GPM_->hasPlan()){
        std::vector<geometry_msgs::msg::PoseStamped> plan;
        GPM_->copyPlan(plan);
        LP_->setPlan(plan);
      }
      base_trajectory::Trajectory best_traj;
      dddmr_sys_core::PlannerState PS = LP_->computeVelocityCommand(control_trajectory_generator_name_, best_traj);
      bool used_blocked_fallback = false;

      // [SmoothOmniGO2] In waiting state, attempt omni fallback before committing to stop/recovery.
      if(enable_blocked_control_fallback_ &&
         (PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING ||
          PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT ||
          PS == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL))
      {
        base_trajectory::Trajectory blocked_traj;
        dddmr_sys_core::PlannerState blocked_ps = LP_->computeVelocityCommand(blocked_control_trajectory_generator_name_, blocked_traj);
        if(blocked_ps == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
          std::string selected_blocked_generator = blocked_control_trajectory_generator_name_;
          if(shouldTrySecondaryBlockedFallback(blocked_traj)){
            base_trajectory::Trajectory secondary_traj;
            dddmr_sys_core::PlannerState secondary_ps =
              LP_->computeVelocityCommand(blocked_control_secondary_generator_name_, secondary_traj);
            if(secondary_ps == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
              const double min_forward = std::max(0.0, blocked_fallback_min_forward_speed_);
              const bool blocked_has_progress = blocked_traj.xv_ >= min_forward;
              const bool secondary_has_progress = secondary_traj.xv_ >= min_forward;
              if((secondary_has_progress && !blocked_has_progress) ||
                 (secondary_traj.xv_ > blocked_traj.xv_ + 0.03))
              {
                blocked_traj = secondary_traj;
                selected_blocked_generator = blocked_control_secondary_generator_name_;
              }
            }
          }
          best_traj = blocked_traj;
          PS = dddmr_sys_core::PlannerState::TRAJECTORY_FOUND;
          used_blocked_fallback = true;
          replan_requested_from_motion_ = true;
          RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 1000,
            "Waiting-state fallback to blocked-control generator: %s | cmd(vx, vy, w)=(%.2f, %.2f, %.2f)",
            selected_blocked_generator.c_str(),
            best_traj.xv_, best_traj.yv_, best_traj.thetav_);
        }
      }

      if(PS == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
        FSM_->last_valid_control_ = clock_->now();
        FSM_->setDecision("d_controlling");
        has_controlled_once_ = true;
        replan_requested_from_motion_ = used_blocked_fallback;
        // [SmoothLaneChange] Immediately send velocity when leaving waiting state to avoid pause/jump.
        publishVelocity(best_traj.xv_, best_traj.yv_, best_traj.thetav_);
        return false;
      }

      else if(PS == dddmr_sys_core::PlannerState::PERCEPTION_MALFUNCTION){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Sensor data is out of date, we're not going to allow commanding of the base for safety");
        publishZeroVelocity();
        return false;
      }

      else if(PS == dddmr_sys_core::PlannerState::TF_FAIL){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Detect TF fail in local planner, we're not going to allow commanding of the base for safety");
        publishZeroVelocity();
        return false;
      }

      else if(PS == dddmr_sys_core::PlannerState::PRUNE_PLAN_FAIL){
        FSM_->last_valid_plan_ = clock_->now();
        publishZeroVelocity();
        replan_requested_from_motion_ = true;
        FSM_->setDecision("d_planning");  
        return false;
      }

      else if(PS == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL){
        //At least implement last_valid_control_ timeout to abort here
        //@ this assignment will allow at least one time planning query
        if((clock_->now() - FSM_->last_valid_control_).seconds() > FSM_->controller_patience_){
          RCLCPP_WARN(this->get_logger(), "Controller time out, go to recovery");
          startRecoveryBehaviors();
          FSM_->setDecision("d_recovery_waitdone");
        }
        else{
          FSM_->last_valid_plan_ = clock_->now();
          replan_requested_from_motion_ = true;
          FSM_->setDecision("d_planning");  
        }
        return false;
      }

      else if(PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING){
        FSM_->last_valid_plan_ = clock_->now();
        replan_requested_from_motion_ = true;
        FSM_->setDecision("d_planning");
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 1000, "Path conflict in waiting: switch to replanning without forced stop.");
	      return false;
      }

      else if(PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT){
	      FSM_->setDecision("d_waiting");
        publishZeroVelocity();
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Path conflits in waiting state, keep waiting.");
	      return false;
      }

      else{
        RCLCPP_FATAL(this->get_logger(), "Should not happen here, we did not catch dddmr_sys_core::PlannerState");
        publishZeroVelocity();
        return false;
      }
    }

  return false;
}

void P2PMoveBase::startRecoveryBehaviors(){

  auto goal_msg = dddmr_sys_core::action::RecoveryBehaviors::Goal();
  goal_msg.behavior_name = "rotate_inplace";

  auto send_goal_options = rclcpp_action::Client<dddmr_sys_core::action::RecoveryBehaviors>::SendGoalOptions();
  
  send_goal_options.goal_response_callback =
    std::bind(&P2PMoveBase::recovery_behaviors_client_goal_response_callback, this, std::placeholders::_1);
  send_goal_options.result_callback =
    std::bind(&P2PMoveBase::recovery_behaviors_client_result_callback, this, std::placeholders::_1);
  
  is_recoverying_ = true;
  recovery_behaviors_client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void P2PMoveBase::recovery_behaviors_client_goal_response_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by recovery behaviors server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by recovery behaviors server, waiting for result");
  }
}

void P2PMoveBase::recovery_behaviors_client_result_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>::WrappedResult & result)
{
  is_recoverying_succeed_ = false;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      is_recoverying_succeed_ = true;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Recovery Behaviors: Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Recovery Behaviors: Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Recovery Behaviors: Unknown result code");
      break;
  }
  
  is_recoverying_ = false;
}

}//end of name space
