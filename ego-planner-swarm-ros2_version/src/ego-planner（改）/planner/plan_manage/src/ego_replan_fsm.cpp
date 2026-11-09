
#include <ego_planner/ego_replan_fsm.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace ego_planner
{
  namespace
  {
    bool planarYaw(const Eigen::Vector3d &from, const Eigen::Vector3d &to,
                   double stationary_yaw, double &yaw)
    {
      const Eigen::Vector2d direction = (to - from).head<2>();
      if (!direction.allFinite() || !std::isfinite(stationary_yaw))
        return false;
      if (direction.squaredNorm() < 1e-8)
      {
        yaw = stationary_yaw;
        return true;
      }
      yaw = std::atan2(direction.y(), direction.x());
      return true;
    }

    double quaternionYaw(const Eigen::Quaterniond &orientation)
    {
      const Eigen::Matrix3d rotation = orientation.normalized().toRotationMatrix();
      return std::atan2(rotation(1, 0), rotation(0, 0));
    }

    std::string formatPointState(const GridMap::Ptr &map, const Eigen::Vector3d &pt, double yaw)
    {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(2)
          << "[" << pt.x() << ", " << pt.y() << ", " << pt.z() << "]";

      if (map == nullptr)
      {
        oss << " map=null";
        return oss.str();
      }

      const auto collision = map->checkFootprint(pt, yaw);
      oss << " footprint_status=" << static_cast<int>(collision.status)
          << " cylinder=" << collision.cylinder_index;
      return oss.str();
    }

  } // namespace


  void EGOReplanFSM::init(const rclcpp::Node::SharedPtr &node)
  {
    node_ = node;
    
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = false;
    have_execution_state_ = false;
    execution_traj_id_ = -1;
    trajectory_execution_state_ = traj_utils::msg::TrajectoryExecutionState::WAIT_TRAJECTORY;
    trajectory_execution_time_ = 0.0;
    trajectory_execution_frozen_ = false;
    execution_state_received_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);

    node_->declare_parameter("fsm/flight_type", -1);
    node_->declare_parameter("fsm/thresh_replan_time", -1.0);
    node_->declare_parameter("fsm/thresh_no_replan_meter", -1.0);
    node_->declare_parameter("fsm/planning_horizon", -1.0);
    node_->declare_parameter("fsm/planning_horizen_time", -1.0);
    node_->declare_parameter("fsm/emergency_time", 1.0);
    node_->declare_parameter("fsm/realworld_experiment", false);
    node_->declare_parameter("fsm/fail_safe", true);
    node_->declare_parameter("fsm/continuous_replan", true);
    node_->declare_parameter("fsm/reference_path_topic", std::string("/global_path"));
    node_->declare_parameter("fsm/reference_path_z_offset", 0.0);
    node_->declare_parameter("fsm/reference_path_start_skip_dist", 0.2);
    node_->declare_parameter("fsm/reference_path_sample_spacing", 1.0);
    node_->declare_parameter("fsm/odom_world_frame", std::string(""));
    node_->declare_parameter("fsm/odom_z_offset", 0.0);
    node_->declare_parameter("fsm/replan_distance", 1.0);
    node_->declare_parameter("fsm/exec_interval_ms", 200);
    node_->declare_parameter("fsm/safety_interval_ms", 100);
    node_->declare_parameter("fsm/max_replan_failures", 5);
    node_->declare_parameter("fsm/execution_state_timeout", 0.30);

    node_->get_parameter("fsm/flight_type", target_type_);
    node_->get_parameter("fsm/thresh_replan_time", replan_thresh_);
    node_->get_parameter("fsm/thresh_no_replan_meter", no_replan_thresh_);
    node_->get_parameter("fsm/planning_horizon", planning_horizen_);
    node_->get_parameter("fsm/planning_horizen_time", planning_horizen_time_);
    node_->get_parameter("fsm/emergency_time", emergency_time_);
    node_->get_parameter("fsm/realworld_experiment", flag_realworld_experiment_);
    node_->get_parameter("fsm/fail_safe", enable_fail_safe_);
    node_->get_parameter("fsm/continuous_replan", continuous_replan_);
    node_->get_parameter("fsm/reference_path_topic", reference_path_topic_);
    node_->get_parameter("fsm/reference_path_z_offset", reference_path_z_offset_);
    node_->get_parameter("fsm/reference_path_start_skip_dist", reference_path_start_skip_dist_);
    node_->get_parameter("fsm/reference_path_sample_spacing", reference_path_sample_spacing_);
    node_->get_parameter("fsm/odom_world_frame", odom_world_frame_);
    node_->get_parameter("fsm/odom_z_offset", odom_z_offset_);
    node_->get_parameter("fsm/replan_distance", replan_distance_);
    node_->get_parameter("fsm/exec_interval_ms", exec_interval_ms_);
    node_->get_parameter("fsm/safety_interval_ms", safety_interval_ms_);
    node_->get_parameter("fsm/max_replan_failures", max_replan_failures_);
    node_->get_parameter("fsm/execution_state_timeout", execution_state_timeout_);

    if (execution_state_timeout_ <= 0.0)
      throw std::runtime_error("fsm/execution_state_timeout must be positive");
    if (max_replan_failures_ <= 0)
      throw std::runtime_error("fsm/max_replan_failures must be positive");
    if (!std::isfinite(planning_horizen_) || planning_horizen_ <= 0.0)
      throw std::runtime_error("fsm/planning_horizon must be positive");
    if (!std::isfinite(replan_distance_) || replan_distance_ <= 0.0)
      throw std::runtime_error("fsm/replan_distance must be positive");

    if (odom_world_frame_.empty())
    {
      node_->get_parameter("grid_map/frame_id", odom_world_frame_);
    }
    if (odom_world_frame_.empty())
      odom_world_frame_ = "world";

    if (reference_path_topic_.empty())
      throw std::runtime_error("fsm/reference_path_topic must not be empty");
    reference_path_start_skip_dist_ = std::max(0.0, reference_path_start_skip_dist_);
    if (!std::isfinite(reference_path_sample_spacing_) || reference_path_sample_spacing_ <= 0.0)
      throw std::runtime_error("fsm/reference_path_sample_spacing must be positive");

    have_trigger_ = !flag_realworld_experiment_;
    replan_failure_count_ = 0;
    stop_sent_in_wait_replan_ = false;
    planning_failure_exhausted_ = false;
    force_wait_replan_ = false;
    force_wait_replan_reason_.clear();
    reference_progress_initialized_ = false;
    reference_path_total_length_ = 0.0;
    reference_path_progress_s_ = 0.0;
    reference_path_id_ = 0;
    blocked_map_revision_ = 0;
    blocked_reference_path_id_ = 0;
    odom_pos_.setZero();
    odom_vel_.setZero();
    odom_acc_.setZero();
    init_pt_.setZero();
    start_pt_.setZero();
    start_vel_.setZero();
    start_acc_.setZero();
    end_pt_.setZero();
    end_vel_.setZero();
    local_target_pt_.setZero();
    local_target_vel_.setZero();

    node_->declare_parameter("fsm/waypoint_num", -1);
    node_->get_parameter("fsm/waypoint_num", waypoint_num_);

    for (int i = 0; i < waypoint_num_; i++)
    {
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_x", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_y", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_z", -1.0);

      node_->get_parameter("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2]);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));

    planner_manager_.reset(new EGOPlannerManager);

    planner_manager_->initPlanModules(node_, visualization_);

    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    /* callback*/
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(std::max(1, exec_interval_ms_)),
                                           std::bind(&EGOReplanFSM::execFSMCallback, this));

    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(std::max(20, safety_interval_ms_)),
                                             std::bind(&EGOReplanFSM::checkCollisionCallback, this));

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom_world",
        1,
        [this](const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
        {
          this->odometryCallback(msg);
        });
    // std::bind(&EGOReplanFSM::odometryCallback, this, std::placeholders::_1));

    if (planner_manager_->pp_.drone_id >= 1)
    {
      string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1) + string("_planning/swarm_trajs");
      swarm_trajs_sub_ = node_->create_subscription<traj_utils::msg::MultiBsplines>(
          sub_topic_name,
          10,
          [this](const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg)
          {
            this->swarmTrajsCallback(msg);
          });
    }

    // ros2 ��topic�����в��ܳ��ָ��ţ�����id��-1��Ҫ����
    // string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    string pub_topic_name;
    if (planner_manager_->pp_.drone_id <= -1)
    {
      RCLCPP_INFO(node_->get_logger(), "single drone:%d", planner_manager_->pp_.drone_id);
      pub_topic_name = string("/drone_") + "single" + string("_planning/swarm_trajs");
    }else
    {
      pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    }
    
    swarm_trajs_pub_ = node_->create_publisher<traj_utils::msg::MultiBsplines>(pub_topic_name, 10);

    broadcast_bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/broadcast_bspline_from_planner", 10);
    broadcast_bspline_sub_ = node_->create_subscription<traj_utils::msg::Bspline>(
        "planning/broadcast_bspline_to_planner",
        100,
        [this](const std::shared_ptr<const traj_utils::msg::Bspline> &msg)
        {
          this->BroadcastBsplineCallback(msg);
        });

    bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/bspline", 10);
    data_disp_pub_ = node_->create_publisher<traj_utils::msg::DataDisp>("planning/data_display", 100);
    execution_state_sub_ = node_->create_subscription<traj_utils::msg::TrajectoryExecutionState>(
        "planning/trajectory_execution_state", 20,
        [this](const std::shared_ptr<const traj_utils::msg::TrajectoryExecutionState> &msg)
        {
          this->executionStateCallback(msg);
        });

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/move_base_simple/goal",
          1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->waypointCallback(msg);
          });
    }
    else if (target_type_ == TARGET_TYPE::REFENCE_PATH)
    {
      path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          reference_path_topic_,
          1,
          [this](const std::shared_ptr<const nav_msgs::msg::Path> &msg)
          {
            this->pathCallback(msg);
          });
      RCLCPP_INFO(node_->get_logger(), "Reference path mode enabled, subscribing to %s", reference_path_topic_.c_str());
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/traj_start_trigger",
          1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->triggerCallback(msg);
          });

      RCLCPP_INFO(node_->get_logger(), "Wait for 1 second.");
      int count = 0;
      while (rclcpp::ok() && count++ < 1000)
      {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      RCLCPP_WARN(node_->get_logger(), "Waiting for trigger from [n3ctrl] from RC");

      while (rclcpp::ok() && (!have_odom_ || !have_trigger_))
      {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      readGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::readGivenWps()

  {
    if (waypoint_num_ <= 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    // �� visualization_->displayGoalPoint() ������waypoint���п��ӻ�
    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // plan first global waypoint
    wp_id_ = 0;
    planNextWaypoint(wps_[wp_id_]);
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    end_pt_ = next_wp;
    bool success = false;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), next_wp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    if (success)
      success = adjustGlobalTargetIfOccupied();

    if (success)
    {
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;
      replan_failure_count_ = 0;
      stop_sent_in_wait_replan_ = false;

      /*** FSM״̬ת�� ***/
      if (exec_state_ == WAIT_TARGET || exec_state_ == INIT || exec_state_ == WAIT_REPLAN ||
          exec_state_ == WAIT_TRAJ_START)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::triggerCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }

  void EGOReplanFSM::waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    have_trigger_ = true;

    init_pt_ = odom_pos_;

    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, 1.0);

    planNextWaypoint(end_wp);
  }

  void EGOReplanFSM::pathCallback(const std::shared_ptr<const nav_msgs::msg::Path> &msg)
  {
    if (msg->poses.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "[EGO] Received empty reference path.");
      return;
    }

    if (!have_odom_)
    {
      pending_reference_path_ = std::make_shared<nav_msgs::msg::Path>(*msg);
      RCLCPP_WARN(node_->get_logger(), "[EGO] Cache reference path until the first valid odometry arrives.");
      return;
    }

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(msg->poses.size());

    const double start_skip_dist = std::max(0.0, reference_path_start_skip_dist_);
    size_t skipped_start_waypoints = 0;
    Eigen::Vector3d last_wp = odom_pos_;
    for (const auto &pose_stamped : msg->poses)
    {
      geometry_msgs::msg::PoseStamped pose_in = pose_stamped;
      if (pose_in.header.frame_id.empty())
        pose_in.header.frame_id = msg->header.frame_id;
      if (pose_in.header.stamp.sec == 0 && pose_in.header.stamp.nanosec == 0)
        pose_in.header.stamp = msg->header.stamp;

      geometry_msgs::msg::PoseStamped pose_world;
      if (pose_in.header.frame_id.empty() || pose_in.header.frame_id == odom_world_frame_)
      {
        pose_world = pose_in;
        pose_world.header.frame_id = odom_world_frame_;
      }
      else
      {
        geometry_msgs::msg::TransformStamped path_transform;
        if (!lookupTransformSafe(odom_world_frame_, pose_in.header.frame_id,
                                 rclcpp::Time(pose_in.header.stamp), path_transform))
        {
          RCLCPP_ERROR_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "[EGO] Reject reference path: cannot transform %s to %s.",
              pose_in.header.frame_id.c_str(), odom_world_frame_.c_str());
          return;
        }
        tf2::doTransform(pose_in, pose_world, path_transform);
      }

      Eigen::Vector3d wp;
      wp << pose_world.pose.position.x,
          pose_world.pose.position.y,
          pose_world.pose.position.z + reference_path_z_offset_;
      if (!wp.allFinite())
      {
        RCLCPP_ERROR(node_->get_logger(), "[EGO] Reject reference path containing a non-finite pose.");
        return;
      }

      if (waypoints.empty())
      {
        if (start_skip_dist > 1e-6 && (wp - odom_pos_).norm() < start_skip_dist)
        {
          skipped_start_waypoints++;
          continue;
        }

        waypoints.push_back(wp);
        last_wp = wp;
        continue;
      }

      if ((wp - last_wp).norm() > 1e-6)
      {
        waypoints.push_back(wp);
        last_wp = wp;
      }
    }

    if (waypoints.empty())
    {
      RCLCPP_WARN(
          node_->get_logger(),
          "[EGO] Reference path has no waypoint after skipping %zu near-start poses.",
          skipped_start_waypoints);
      return;
    }

    if (waypoints.size() > 2)
    {
      std::vector<Eigen::Vector3d> sampled;
      sampled.reserve(waypoints.size());
      sampled.push_back(waypoints.front());
      constexpr double kCornerAngle = 0.12;
      for (size_t i = 1; i + 1 < waypoints.size(); ++i)
      {
        const Eigen::Vector3d incoming = waypoints[i] - waypoints[i - 1];
        const Eigen::Vector3d outgoing = waypoints[i + 1] - waypoints[i];
        double turn_angle = 0.0;
        if (incoming.norm() > 1e-6 && outgoing.norm() > 1e-6)
        {
          const double cosine = std::clamp(
              incoming.dot(outgoing) / (incoming.norm() * outgoing.norm()), -1.0, 1.0);
          turn_angle = std::acos(cosine);
        }
        if ((waypoints[i] - sampled.back()).norm() >= reference_path_sample_spacing_ ||
            turn_angle >= kCornerAngle)
          sampled.push_back(waypoints[i]);
      }
      if ((waypoints.back() - sampled.back()).norm() > 1e-6)
        sampled.push_back(waypoints.back());
      waypoints.swap(sampled);
    }
    if (skipped_start_waypoints > 0)
    {
      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "[EGO] Skipped %zu near-start reference path poses before fitting the global guide.",
          skipped_start_waypoints);
    }

    const Eigen::Vector3d requested_end = waypoints.back();
    // Global planner often latches/re-publishes the same path. Re-arming after
    // arrival makes traj_server keep ALIGN_HEADING (~π yaw error) and spin.
    if ((requested_end - odom_pos_).head<2>().norm() <= std::max(0.25, no_replan_thresh_) &&
        (waypoints.front() - odom_pos_).head<2>().norm() <= std::max(1.0, planning_horizen_))
    {
      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "[EGO] Ignore reference path republish: already near goal (%.2f m).",
          (requested_end - odom_pos_).head<2>().norm());
      if (exec_state_ == WAIT_TARGET || exec_state_ == INIT)
        return;
      if (have_target_ && exec_state_ == EXEC_TRAJ)
        return;
    }

    end_pt_ = requested_end;
    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        waypoints, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    if (success)
      success = adjustGlobalTargetIfOccupied();

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from reference path!");
      return;
    }

    const bool target_was_truncated = (end_pt_ - requested_end).norm() > 1e-6;
    bool reference_stored = false;
    if (target_was_truncated)
    {
      reference_stored = setTruncatedReferencePathFromGlobalTrajectory();
    }
    else
    {
      std::vector<Eigen::Vector3d> stored_reference = waypoints;
      if (stored_reference.size() == 1)
        stored_reference.insert(stored_reference.begin(), odom_pos_);
      reference_stored = setReferencePath(stored_reference);
    }
    if (!reference_stored)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[EGO] Reject malformed reference path: odometry projection failed.");
      return;
    }

    init_pt_ = odom_pos_;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_trigger_ = true;
    have_new_target_ = true;
    replan_failure_count_ = 0;
    stop_sent_in_wait_replan_ = false;
    if (exec_state_ == EXEC_TRAJ)
    {
      double execution_time = 0.0;
      if (getSynchronizedExecutionTime(execution_time))
        changeFSMExecState(REPLAN_TRAJ, "REF_PATH");
    }
    else if (exec_state_ == WAIT_TARGET || exec_state_ == INIT || exec_state_ == WAIT_REPLAN ||
        exec_state_ == WAIT_TRAJ_START ||
        exec_state_ == GEN_NEW_TRAJ || exec_state_ == REPLAN_TRAJ)
      changeFSMExecState(GEN_NEW_TRAJ, "REF_PATH");
    else if (exec_state_ == EMERGENCY_STOP)
      changeFSMExecState(GEN_NEW_TRAJ, "REF_PATH");

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
  }

  bool EGOReplanFSM::lookupTransformSafe(const std::string &target_frame, const std::string &source_frame,
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
          rclcpp::Duration::from_seconds(0.1));
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

  bool EGOReplanFSM::getOdomStateInWorld(const nav_msgs::msg::Odometry &odom_msg, Eigen::Vector3d &pos,
                                         Eigen::Quaterniond &orientation, Eigen::Vector3d &linear_vel,
                                         Eigen::Vector3d &angular_vel)
  {
    tf2::Transform pose_tf;
    pose_tf.setOrigin(tf2::Vector3(
        odom_msg.pose.pose.position.x,
        odom_msg.pose.pose.position.y,
        odom_msg.pose.pose.position.z));
    pose_tf.setRotation(tf2::Quaternion(
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
        odom_msg.pose.pose.orientation.w));

    tf2::Quaternion header_to_world_q(0.0, 0.0, 0.0, 1.0);
    if (!odom_msg.header.frame_id.empty() && odom_msg.header.frame_id != odom_world_frame_)
    {
      geometry_msgs::msg::TransformStamped pose_transform;
      if (!lookupTransformSafe(odom_world_frame_, odom_msg.header.frame_id,
                               rclcpp::Time(odom_msg.header.stamp), pose_transform))
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Failed to transform odom from %s to %s",
                             odom_msg.header.frame_id.c_str(), odom_world_frame_.c_str());
        return false;
      }

      tf2::Transform tf_target_source;
      tf_target_source.setOrigin(tf2::Vector3(
          pose_transform.transform.translation.x,
          pose_transform.transform.translation.y,
          pose_transform.transform.translation.z));
      tf_target_source.setRotation(tf2::Quaternion(
          pose_transform.transform.rotation.x,
          pose_transform.transform.rotation.y,
          pose_transform.transform.rotation.z,
          pose_transform.transform.rotation.w));
      header_to_world_q = tf_target_source.getRotation();
      pose_tf = tf_target_source * pose_tf;
    }

    const tf2::Vector3 origin = pose_tf.getOrigin();
    const tf2::Quaternion rotation = pose_tf.getRotation();
    pos = Eigen::Vector3d(origin.x(), origin.y(), origin.z() + odom_z_offset_);
    orientation = Eigen::Quaterniond(rotation.w(), rotation.x(), rotation.y(), rotation.z());

    tf2::Vector3 linear_body(
        odom_msg.twist.twist.linear.x,
        odom_msg.twist.twist.linear.y,
        odom_msg.twist.twist.linear.z);
    tf2::Vector3 angular_body(
        odom_msg.twist.twist.angular.x,
        odom_msg.twist.twist.angular.y,
        odom_msg.twist.twist.angular.z);

    tf2::Vector3 linear_world;
    tf2::Vector3 angular_world;
    if (odom_msg.child_frame_id.empty())
    {
      linear_world = tf2::quatRotate(header_to_world_q, linear_body);
      angular_world = tf2::quatRotate(header_to_world_q, angular_body);
    }
    else if (odom_msg.child_frame_id == odom_world_frame_)
    {
      linear_world = linear_body;
      angular_world = angular_body;
    }
    else
    {
      linear_world = tf2::quatRotate(rotation, linear_body);
      angular_world = tf2::quatRotate(rotation, angular_body);
    }

    linear_vel = Eigen::Vector3d(linear_world.x(), linear_world.y(), linear_world.z());
    angular_vel = Eigen::Vector3d(angular_world.x(), angular_world.y(), angular_world.z());
    return true;
  }

  void EGOReplanFSM::odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
  {
    Eigen::Vector3d angular_vel;
    if (!getOdomStateInWorld(*msg, odom_pos_, odom_orient_, odom_vel_, angular_vel))
      return;

    have_odom_ = true;
    if (pending_reference_path_)
    {
      auto pending_path = pending_reference_path_;
      pending_reference_path_.reset();
      pathCallback(pending_path);
    }
    if (target_type_ == TARGET_TYPE::REFENCE_PATH && have_target_ &&
        reference_progress_initialized_)
      projectReferenceProgress(odom_pos_, false);
  }

  void EGOReplanFSM::BroadcastBsplineCallback(const std::shared_ptr<const traj_utils::msg::Bspline> &msg)
  {
    size_t id = msg->drone_id;
    if ((int)id == planner_manager_->pp_.drone_id)
      return;
    if (msg->order != 3 || msg->pos_pts.size() < 3 ||
        msg->knots.size() < msg->pos_pts.size() + msg->order + 1)
    {
      RCLCPP_ERROR(node_->get_logger(), "Reject malformed broadcast B-spline.");
      return;
    }

    // if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    rclcpp::Clock clock(RCL_SYSTEM_TIME);  // ȷ��ʹ�õ�ǰ�ڵ��ʱ��Դ
    auto msg_time = rclcpp::Time(msg->start_time, clock.get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "Clock type: %d", rclcpp::Clock().now().get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "Start time clock type: %d", rclcpp::Time(msg->start_time).get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "msg_time: %d", msg_time.get_clock_type());
    if (abs((rclcpp::Clock().now() - msg_time).seconds()) > 0.25)
    {
      // ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs", msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      RCLCPP_ERROR(node_->get_logger(), "Time difference is too large! Local - Remote Agent %d = %fs",
                   msg->drone_id, (rclcpp::Clock().now() - msg_time).seconds());
      return;
    }

    // ·����������ʼ��
    if (planner_manager_->swarm_trajs_buf_.size() <= id)
    {
      for (size_t i = planner_manager_->swarm_trajs_buf_.size(); i <= id; i++)
      {
        OneTrajDataOfSwarm blank;
        blank.drone_id = -1;
        planner_manager_->swarm_trajs_buf_.push_back(blank);
      }
    }

    /* Test distance to the agent */
    Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
    Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
    Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z);
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
    {
      planner_manager_->swarm_trajs_buf_[id].drone_id = -1;
      return; // if the current drone is too far to the received agent.
    }

    /* Store data */
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t j = 0; j < msg->knots.size(); ++j)
    {
      knots(j) = msg->knots[j];
    }
    for (size_t j = 0; j < msg->pos_pts.size(); ++j)
    {
      pos_pts(0, j) = msg->pos_pts[j].x;
      pos_pts(1, j) = msg->pos_pts[j].y;
      pos_pts(2, j) = msg->pos_pts[j].z;
    }

    planner_manager_->swarm_trajs_buf_[id].drone_id = id;

    // ����·������ʱ��
    if (msg->order % 2)
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = msg->knots[msg->knots.size() - ceil(cutback)];
    }
    else
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = (msg->knots[msg->knots.size() - floor(cutback)] + msg->knots[msg->knots.size() - ceil(cutback)]) / 2;
    }

    // ����bspline���洢
    UniformBspline pos_traj(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    pos_traj.setKnot(knots);
    planner_manager_->swarm_trajs_buf_[id].position_traj_ = pos_traj;

    planner_manager_->swarm_trajs_buf_[id].start_pos_ = planner_manager_->swarm_trajs_buf_[id].position_traj_.evaluateDeBoorT(0);

    planner_manager_->swarm_trajs_buf_[id].start_time_ = msg->start_time;

    /* Check Collision */
    if (planner_manager_->checkCollision(id))
    {
      changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
    }
  }

  void EGOReplanFSM::swarmTrajsCallback(const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg)
  {

    multi_bspline_msgs_buf_.traj.clear();
    multi_bspline_msgs_buf_ = *msg;

    if (!have_odom_)
    {
      RCLCPP_ERROR(node_->get_logger(), "swarmTrajsCallback(): no odom!, return.");
      return;
    }

    if (msg->traj.empty() || (int)msg->traj.size() != msg->drone_id_from + 1) // drone_id must start from 0
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong trajectory size!msg->traj.size()=%d, msg->drone_id_from+1=%d", (int)msg->traj.size(), msg->drone_id_from + 1);
      return;
    }

    if (msg->traj[0].order != 3) // only support B-spline order equals 3.
    {
      RCLCPP_ERROR(node_->get_logger(), "Only support B-spline order equals 3.");
      return;
    }

    // Step 1. receive the trajectories
    planner_manager_->swarm_trajs_buf_.clear();
    planner_manager_->swarm_trajs_buf_.resize(msg->traj.size());

    // ����ÿ��·��
    for (size_t i = 0; i < msg->traj.size(); i++)
    {
      if (msg->traj[i].order != 3 || msg->traj[i].pos_pts.size() < 3 ||
          msg->traj[i].knots.size() < msg->traj[i].pos_pts.size() + msg->traj[i].order + 1)
      {
        RCLCPP_ERROR(node_->get_logger(), "Reject malformed swarm B-spline at index %zu.", i);
        return;
      }

      Eigen::Vector3d cp0(msg->traj[i].pos_pts[0].x, msg->traj[i].pos_pts[0].y, msg->traj[i].pos_pts[0].z);
      Eigen::Vector3d cp1(msg->traj[i].pos_pts[1].x, msg->traj[i].pos_pts[1].y, msg->traj[i].pos_pts[1].z);
      Eigen::Vector3d cp2(msg->traj[i].pos_pts[2].x, msg->traj[i].pos_pts[2].y, msg->traj[i].pos_pts[2].z);
      Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
      if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
      {
        planner_manager_->swarm_trajs_buf_[i].drone_id = -1;
        continue;
      }

      // �洢·�����Ƶ�ͽڵ�
      Eigen::MatrixXd pos_pts(3, msg->traj[i].pos_pts.size());
      Eigen::VectorXd knots(msg->traj[i].knots.size());
      for (size_t j = 0; j < msg->traj[i].knots.size(); ++j)
      {
        knots(j) = msg->traj[i].knots[j];
      }
      for (size_t j = 0; j < msg->traj[i].pos_pts.size(); ++j)
      {
        pos_pts(0, j) = msg->traj[i].pos_pts[j].x;
        pos_pts(1, j) = msg->traj[i].pos_pts[j].y;
        pos_pts(2, j) = msg->traj[i].pos_pts[j].z;
      }

      planner_manager_->swarm_trajs_buf_[i].drone_id = i;

      // ����·������ʱ��
      if (msg->traj[i].order % 2)
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)];
      }
      else
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = (msg->traj[i].knots[msg->traj[i].knots.size() - floor(cutback)] + msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)]) / 2;
      }

      // planner_manager_->swarm_trajs_buf_[i].position_traj_ =
      UniformBspline pos_traj(pos_pts, msg->traj[i].order, msg->traj[i].knots[1] - msg->traj[i].knots[0]);
      pos_traj.setKnot(knots);
      planner_manager_->swarm_trajs_buf_[i].position_traj_ = pos_traj;

      planner_manager_->swarm_trajs_buf_[i].start_pos_ = planner_manager_->swarm_trajs_buf_[i].position_traj_.evaluateDeBoorT(0);

      planner_manager_->swarm_trajs_buf_[i].start_time_ = msg->traj[i].start_time;
    }

    have_recv_pre_agent_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[9] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START", "WAIT_REPLAN", "WAIT_TRAJ_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    if (new_state == WAIT_REPLAN && pre_s != WAIT_REPLAN)
    {
      const auto map = planner_manager_ ? planner_manager_->grid_map_ : nullptr;
      blocked_map_revision_ = map ? map->getMapRevision() : 0;
      blocked_reference_path_id_ = reference_path_id_;
    }
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[9] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START", "WAIT_REPLAN", "WAIT_TRAJ_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  rclcpp::Time EGOReplanFSM::plannerNow() const
  {
    // Keep all FSM/planner time arithmetic on the same clock source as
    // planner_manager::local_data_.start_time_ to avoid runtime clock-type
    // mismatch when subtracting timestamps during replanning.
    return rclcpp::Clock(RCL_SYSTEM_TIME).now();
  }

  void EGOReplanFSM::executionStateCallback(
      const std::shared_ptr<const traj_utils::msg::TrajectoryExecutionState> &msg)
  {
    if (!msg || !std::isfinite(msg->exec_time) || msg->exec_time < -1e-6)
    {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "[EGO] Reject invalid trajectory execution state.");
      return;
    }

    execution_traj_id_ = msg->traj_id;
    trajectory_execution_state_ = msg->state;
    trajectory_execution_time_ = msg->exec_time;
    trajectory_execution_frozen_ = msg->frozen;
    execution_state_received_time_ = plannerNow();
    have_execution_state_ = true;

    if (planner_manager_ && execution_traj_id_ == planner_manager_->local_data_.traj_id_)
    {
      planner_manager_->local_data_.start_time_ =
          execution_state_received_time_ -
          rclcpp::Duration::from_seconds(trajectory_execution_time_);
    }

    if (msg->state == traj_utils::msg::TrajectoryExecutionState::FAULT_STOP)
    {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "[EGO] Trajectory controller fault for trajectory %ld: %s",
                            static_cast<long>(msg->traj_id), msg->fault_reason.c_str());
    }
  }

  bool EGOReplanFSM::getSynchronizedExecutionTime(double &execution_time,
                                                   uint8_t *execution_state) const
  {
    if (!have_execution_state_ || !planner_manager_)
      return false;

    const auto &local = planner_manager_->local_data_;
    if (execution_traj_id_ != local.traj_id_)
      return false;

    const double state_age = (plannerNow() - execution_state_received_time_).seconds();
    if (!std::isfinite(state_age) || state_age < 0.0 || state_age > execution_state_timeout_)
      return false;

    if (trajectory_execution_state_ != traj_utils::msg::TrajectoryExecutionState::ALIGN_HEADING &&
        trajectory_execution_state_ != traj_utils::msg::TrajectoryExecutionState::TRACK_TRAJECTORY &&
        trajectory_execution_state_ != traj_utils::msg::TrajectoryExecutionState::HOLD_FINAL)
      return false;

    if (!std::isfinite(trajectory_execution_time_) || trajectory_execution_time_ < -1e-6 ||
        trajectory_execution_time_ > local.duration_ + 1e-3)
      return false;

    execution_time = std::clamp(trajectory_execution_time_, 0.0, local.duration_);
    if (execution_state != nullptr)
      *execution_state = trajectory_execution_state_;
    return true;
  }

  void EGOReplanFSM::resetReplanTracking()
  {
    replan_failure_count_ = 0;
    stop_sent_in_wait_replan_ = false;
    planning_failure_exhausted_ = false;
    force_wait_replan_ = false;
    force_wait_replan_reason_.clear();
  }

  void EGOReplanFSM::finishCurrentTarget()
  {
    have_target_ = false;
    have_trigger_ = false;
    replan_failure_count_ = 0;
    stop_sent_in_wait_replan_ = false;
    force_wait_replan_ = false;
    force_wait_replan_reason_.clear();
    reference_path_points_.clear();
    reference_path_arc_lengths_.clear();
    reference_path_total_length_ = 0.0;
    reference_path_progress_s_ = 0.0;
    reference_progress_initialized_ = false;
  }

  bool EGOReplanFSM::setReferencePath(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.size() < 2)
      return false;

    std::vector<double> arc_lengths(waypoints.size(), 0.0);
    for (size_t i = 1; i < waypoints.size(); ++i)
    {
      const double segment_length = (waypoints[i] - waypoints[i - 1]).norm();
      if (!std::isfinite(segment_length) || segment_length <= 1e-6)
        return false;
      arc_lengths[i] = arc_lengths[i - 1] + segment_length;
    }

    reference_path_points_ = waypoints;
    reference_path_arc_lengths_ = std::move(arc_lengths);
    reference_path_total_length_ = reference_path_arc_lengths_.back();
    reference_path_progress_s_ = 0.0;
    reference_progress_initialized_ = false;
    if (!projectReferenceProgress(odom_pos_, true))
    {
      reference_path_points_.clear();
      reference_path_arc_lengths_.clear();
      reference_path_total_length_ = 0.0;
      return false;
    }
    ++reference_path_id_;
    return true;
  }

  bool EGOReplanFSM::setTruncatedReferencePathFromGlobalTrajectory()
  {
    auto &global = planner_manager_->global_data_;
    const double duration = global.global_duration_;
    if (!std::isfinite(duration) || duration <= 1e-6 || !end_pt_.allFinite())
      return false;

    constexpr double kSampleDt = 0.05;
    constexpr double kCornerAngle = 0.12;
    const int sample_count =
        std::max(1, static_cast<int>(std::ceil(duration / kSampleDt)));

    std::vector<Eigen::Vector3d> trajectory_points;
    trajectory_points.reserve(static_cast<size_t>(sample_count) + 1);
    for (int i = 0; i <= sample_count; ++i)
    {
      const double t = duration * static_cast<double>(i) / sample_count;
      const Eigen::Vector3d point = global.getPosition(t);
      if (!point.allFinite())
        return false;
      if (trajectory_points.empty() ||
          (point - trajectory_points.back()).norm() > 1e-6)
        trajectory_points.push_back(point);
    }

    if (trajectory_points.size() < 2 ||
        (trajectory_points.back() - end_pt_).norm() > 1e-6)
      return false;
    trajectory_points.back() = end_pt_;

    std::vector<Eigen::Vector3d> sparse_reference;
    sparse_reference.reserve(trajectory_points.size());
    sparse_reference.push_back(trajectory_points.front());
    for (size_t i = 1; i + 1 < trajectory_points.size(); ++i)
    {
      const Eigen::Vector3d incoming = trajectory_points[i] - trajectory_points[i - 1];
      const Eigen::Vector3d outgoing = trajectory_points[i + 1] - trajectory_points[i];
      double turn_angle = 0.0;
      if (incoming.norm() > 1e-6 && outgoing.norm() > 1e-6)
      {
        const double cosine = std::clamp(
            incoming.dot(outgoing) / (incoming.norm() * outgoing.norm()), -1.0, 1.0);
        turn_angle = std::acos(cosine);
      }
      if ((trajectory_points[i] - sparse_reference.back()).norm() >=
              reference_path_sample_spacing_ ||
          turn_angle >= kCornerAngle)
        sparse_reference.push_back(trajectory_points[i]);
    }
    if ((trajectory_points.back() - sparse_reference.back()).norm() > 1e-6)
      sparse_reference.push_back(trajectory_points.back());

    return setReferencePath(sparse_reference);
  }

  bool EGOReplanFSM::projectReferenceProgress(const Eigen::Vector3d &position, bool initialize)
  {
    if (!position.allFinite() || reference_path_points_.size() < 2 ||
        reference_path_arc_lengths_.size() != reference_path_points_.size() ||
        reference_path_total_length_ <= 1e-6)
      return false;

    const double search_start = initialize ? 0.0 : reference_path_progress_s_;
    const double search_end = initialize
                                  ? reference_path_total_length_
                                  : std::min(reference_path_total_length_,
                                             search_start + std::max(2.0 * planning_horizen_,
                                                                     2.0 * reference_path_sample_spacing_));
    double best_distance = std::numeric_limits<double>::infinity();
    double best_progress = -1.0;

    for (size_t i = 1; i < reference_path_points_.size(); ++i)
    {
      const double segment_start = reference_path_arc_lengths_[i - 1];
      const double segment_end = reference_path_arc_lengths_[i];
      if (segment_end < search_start || segment_start > search_end)
        continue;

      const Eigen::Vector3d &a = reference_path_points_[i - 1];
      const Eigen::Vector3d segment = reference_path_points_[i] - a;
      const double segment_length_sq = segment.squaredNorm();
      if (segment_length_sq <= 1e-12)
        return false;

      const double lower = std::clamp(
          (search_start - segment_start) / (segment_end - segment_start), 0.0, 1.0);
      const double upper = std::clamp(
          (search_end - segment_start) / (segment_end - segment_start), 0.0, 1.0);
      if (upper < lower)
        continue;

      const double projection = std::clamp((position - a).dot(segment) / segment_length_sq,
                                           lower, upper);
      const double distance = (position - (a + projection * segment)).norm();
      if (distance < best_distance)
      {
        best_distance = distance;
        best_progress = segment_start + projection * (segment_end - segment_start);
      }
    }

    if (!std::isfinite(best_distance) || best_progress < 0.0)
      return false;

    reference_path_progress_s_ = initialize
                                     ? best_progress
                                     : std::max(reference_path_progress_s_, best_progress);
    reference_progress_initialized_ = true;
    return true;
  }

  bool EGOReplanFSM::buildSparseRemainingGuide(std::vector<Eigen::Vector3d> &waypoints)
  {
    waypoints.clear();
    if (!projectReferenceProgress(odom_pos_, false))
      return false;

    const double forward_skip = std::max(reference_path_sample_spacing_,
                                         0.25 * planning_horizen_);
    const double anchor_progress = std::min(reference_path_total_length_,
                                            reference_path_progress_s_ + forward_skip);
    size_t anchor_index = reference_path_points_.size() - 1;
    for (size_t i = 1; i < reference_path_arc_lengths_.size(); ++i)
    {
      if (reference_path_arc_lengths_[i] >= anchor_progress)
      {
        anchor_index = i;
        break;
      }
    }

    constexpr double kCornerAngle = 0.12;
    for (size_t i = 1; i < anchor_index && i + 1 < reference_path_points_.size(); ++i)
    {
      if (reference_path_arc_lengths_[i] <= reference_path_progress_s_ + 1e-6)
        continue;
      const Eigen::Vector3d incoming = reference_path_points_[i] - reference_path_points_[i - 1];
      const Eigen::Vector3d outgoing = reference_path_points_[i + 1] - reference_path_points_[i];
      const double denominator = incoming.norm() * outgoing.norm();
      if (denominator <= 1e-9)
        return false;
      const double angle = std::acos(std::clamp(incoming.dot(outgoing) / denominator, -1.0, 1.0));
      if (angle >= kCornerAngle)
        waypoints.push_back(reference_path_points_[i]);
    }

    for (size_t i = anchor_index; i < reference_path_points_.size(); ++i)
    {
      if (waypoints.empty() || (reference_path_points_[i] - waypoints.back()).norm() > 1e-6)
        waypoints.push_back(reference_path_points_[i]);
    }
    return !waypoints.empty();
  }

  bool EGOReplanFSM::canRecoverFromMapRevision()
  {
    if (!have_target_ || reference_path_id_ != blocked_reference_path_id_)
      return false;

    const auto map = planner_manager_ ? planner_manager_->grid_map_ : nullptr;
    if (!map || !map->hasMapObservation() || map->getMapRevision() == blocked_map_revision_)
      return false;

    blocked_map_revision_ = map->getMapRevision();
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> candidates;
    getLocalTarget(&candidates);
    return candidates.size() == 1;
  }

  void EGOReplanFSM::execFSMCallback()
  {
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        if (have_odom_ && have_target_ && have_trigger_)
        {
          bool success = planFromGlobalTraj(1);
          if (success)
          {
            changeFSMExecState(WAIT_TRAJ_START, "FSM");
            resetReplanTracking();

            publishSwarmTrajs(true);
          }
          else
          {
            stop_sent_in_wait_replan_ = false;
            if (force_wait_replan_reason_.empty())
              force_wait_replan_reason_ = "deterministic initial planning failed";
            RCLCPP_ERROR(node_->get_logger(), "[EGO] %s", force_wait_replan_reason_.c_str());
            ++replan_failure_count_;
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          }
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(), "No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {
      const int attempt = replan_failure_count_ + 1;
      bool success = planFromGlobalTraj(attempt);
      if (success)
      {
        changeFSMExecState(WAIT_TRAJ_START, "FSM");
        flag_escape_emergency_ = true;
        resetReplanTracking();
        publishSwarmTrajs(false);
      }
      else
      {
        ++replan_failure_count_;
        if (force_wait_replan_reason_.empty())
          force_wait_replan_reason_ = "deterministic planning failed";
        RCLCPP_ERROR(node_->get_logger(), "[EGO] %s", force_wait_replan_reason_.c_str());
        if (replan_failure_count_ >= max_replan_failures_)
        {
          planning_failure_exhausted_ = true;
          flag_escape_emergency_ = true;
          changeFSMExecState(EMERGENCY_STOP, "FSM");
        }
        else
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      const bool success = have_new_target_
                               ? planFromGlobalTraj(1)
                               : planFromCurrentTraj(1);
      if (success)
      {
        changeFSMExecState(WAIT_TRAJ_START, "FSM");
        resetReplanTracking();
        publishSwarmTrajs(false);
      }
      else
      {
        ++replan_failure_count_;
        if (force_wait_replan_reason_.empty())
          force_wait_replan_reason_ = "deterministic replanning failed";
        RCLCPP_ERROR(node_->get_logger(), "[EGO] %s", force_wait_replan_reason_.c_str());
        if (replan_failure_count_ >= max_replan_failures_)
        {
          planning_failure_exhausted_ = true;
          flag_escape_emergency_ = true;
          changeFSMExecState(EMERGENCY_STOP, "FSM");
        }
        else
          changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case WAIT_TRAJ_START:
    {
      double controller_time = 0.0;
      uint8_t controller_state = traj_utils::msg::TrajectoryExecutionState::WAIT_TRAJECTORY;
      if (getSynchronizedExecutionTime(controller_time, &controller_state))
      {
        changeFSMExecState(EXEC_TRAJ, "EXEC_SYNC");
        break;
      }

      if (have_execution_state_ &&
          execution_traj_id_ == planner_manager_->local_data_.traj_id_ &&
          trajectory_execution_state_ == traj_utils::msg::TrajectoryExecutionState::FAULT_STOP)
      {
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[EGO] Controller rejected trajectory %d; stop and wait for a new planning attempt.",
            planner_manager_->local_data_.traj_id_);
        stop_sent_in_wait_replan_ = false;
        changeFSMExecState(WAIT_REPLAN, "EXEC_SYNC");
        break;
      }

      const double acknowledgement_age =
          (plannerNow() - planner_manager_->local_data_.start_time_).seconds();
      if (std::isfinite(acknowledgement_age) &&
          acknowledgement_age > execution_state_timeout_)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "[EGO] Controller acknowledgement timed out for trajectory %d; command an explicit stop.",
                     planner_manager_->local_data_.traj_id_);
        callEmergencyStop(odom_pos_);
        stop_sent_in_wait_replan_ = true;
        flag_escape_emergency_ = false;
        changeFSMExecState(EMERGENCY_STOP, "EXEC_SYNC");
        break;
      }

      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[EGO] Waiting for controller acknowledgement of trajectory %d.",
          planner_manager_->local_data_.traj_id_);
      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      double t_cur = 0.0;
      uint8_t controller_state = traj_utils::msg::TrajectoryExecutionState::WAIT_TRAJECTORY;
      if (!getSynchronizedExecutionTime(t_cur, &controller_state))
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "[EGO] Controller execution feedback timed out; command an explicit stop.");
        callEmergencyStop(odom_pos_);
        stop_sent_in_wait_replan_ = true;
        flag_escape_emergency_ = false;
        changeFSMExecState(EMERGENCY_STOP, "EXEC_SYNC");
        break;
      }

      if (controller_state == traj_utils::msg::TrajectoryExecutionState::ALIGN_HEADING ||
          trajectory_execution_frozen_)
      {
        break;
      }

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);
      const bool local_traj_finished =
          controller_state == traj_utils::msg::TrajectoryExecutionState::HOLD_FINAL ||
          t_cur > info->duration_ - 1e-2;

      /* && (end_pt_ - pos).norm() < 0.5 */
      if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (wp_id_ < waypoint_num_ - 1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        wp_id_++;
        planNextWaypoint(wps_[wp_id_]);
      }
      else if (local_traj_finished)
      {
        // Reference-path mode: a finished local segment is normal (planning
        // horizon). Keep chaining until the global end is reached; clearing the
        // target here used to freeze the robot far from the goal/obstacle.
        if (target_type_ == TARGET_TYPE::REFENCE_PATH)
        {
          if ((end_pt_ - odom_pos_).norm() > no_replan_thresh_)
          {
            changeFSMExecState(REPLAN_TRAJ, "FSM");
            break;
          }
          finishCurrentTarget();
          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
        }

        finishCurrentTarget();

        if (target_type_ == TARGET_TYPE::PRESET_TARGET)
        {
          wp_id_ = 0;
          planNextWaypoint(wps_[wp_id_]);
        }

        changeFSMExecState(WAIT_TARGET, "FSM");
        goto force_return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        break;
      }
      else
      {
        bool need_replan = false;
        if (have_new_target_)
        {
          need_replan = true;
        }
        else if (continuous_replan_)
        {
          need_replan = (info->start_pos_ - pos).norm() >= replan_distance_;
        }
        else if ((local_target_pt_ - end_pt_).norm() < 1e-3)
        {
          if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
            need_replan = true;
        }
        else if (t_cur > replan_thresh_)
        {
          need_replan = true;
        }

        if (need_replan)
          changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case WAIT_REPLAN:
    {
      if (!have_target_)
      {
        changeFSMExecState(WAIT_TARGET, "FSM");
        break;
      }

      if (!stop_sent_in_wait_replan_)
      {
        callEmergencyStop(odom_pos_);
        stop_sent_in_wait_replan_ = true;
      }

      if (canRecoverFromMapRevision())
      {
        stop_sent_in_wait_replan_ = false;
        changeFSMExecState(GEN_NEW_TRAJ, "MAP_REVISION");
        break;
      }

      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "[EGO] STOPPED: waiting for a new local-map revision before retrying planning.");

      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_)
      {
        callEmergencyStop(odom_pos_);
        stop_sent_in_wait_replan_ = true;
      }
      flag_escape_emergency_ = false;
      if (odom_vel_.norm() < 0.1)
      {
        if (planning_failure_exhausted_)
        {
          finishCurrentTarget();
          planning_failure_exhausted_ = false;
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_STOP");
        }
        else
        {
          changeFSMExecState(have_target_ ? GEN_NEW_TRAJ : WAIT_TARGET,
                             "EMERGENCY_STOP");
        }
      }
      break;
    }
    }

    data_disp_.header.stamp = node_->get_clock()->now();
    data_disp_pub_->publish(data_disp_);

  force_return:;
    return;
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) // zx-todo
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    double execution_time = 0.0;
    if (getSynchronizedExecutionTime(execution_time))
    {
      const LocalTrajData &local = planner_manager_->local_data_;
      start_vel_ = local.velocity_traj_.evaluateDeBoorT(execution_time);
      start_acc_ = local.acceleration_traj_.evaluateDeBoorT(execution_time);
    }

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
    return callReboundReplan(true, trial_times > 1);
  }

  bool EGOReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
  {
    (void)trial_times;

    LocalTrajData *info = &planner_manager_->local_data_;
    double t_cur = 0.0;
    if (!getSynchronizedExecutionTime(t_cur))
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "[EGO] Replan requested without a synchronized executable trajectory state.");
      return false;
    }

    const Eigen::Vector3d traj_start_vel = info->velocity_traj_.evaluateDeBoorT(t_cur);
    const Eigen::Vector3d traj_start_acc = info->acceleration_traj_.evaluateDeBoorT(t_cur);
    start_pt_ = odom_pos_;
    start_vel_ = traj_start_vel;
    start_acc_ = traj_start_acc;

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    bool global_guide_ready = false;
    if (target_type_ == TARGET_TYPE::REFENCE_PATH)
    {
      std::vector<Eigen::Vector3d> remaining_guide;
      if (!buildSparseRemainingGuide(remaining_guide))
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "[EGO] Cannot construct the remaining sparse global guide.");
        return false;
      }

      global_guide_ready = planner_manager_->planGlobalTrajWaypoints(
          start_pt_, start_vel_, start_acc_, remaining_guide,
          end_vel_, Eigen::Vector3d::Zero());
    }
    else
    {
      global_guide_ready = planner_manager_->planGlobalTraj(
          start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, Eigen::Vector3d::Zero());
    }

    if (!global_guide_ready)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[EGO] Cannot rebuild the global guide from the current odometry state.");
      return false;
    }
    const Eigen::Vector3d requested_end = end_pt_;
    if (!adjustGlobalTargetIfOccupied())
      return false;
    if (target_type_ == TARGET_TYPE::REFENCE_PATH &&
        (end_pt_ - requested_end).norm() > 1e-6 &&
        !setTruncatedReferencePathFromGlobalTrajectory())
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[EGO] Cannot persist the global guide truncated at the occupied target.");
      return false;
    }

    if (visualization_ != nullptr)
    {
      visualization_->displayGoalPoint(odom_pos_, Eigen::Vector4d(0.0, 1.0, 1.0, 1.0), 0.18, 9000);
      visualization_->displayGoalPoint(start_pt_, Eigen::Vector4d(1.0, 0.5, 0.0, 1.0), 0.18, 9002);
    }

    if (callReboundReplan(true, false))
      return true;
    return callReboundReplan(true, true);
  }

  bool EGOReplanFSM::adjustGlobalTargetIfOccupied()
  {
    const auto map = planner_manager_->grid_map_;
    auto &global = planner_manager_->global_data_;
    const double duration = global.global_duration_;
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05;
    const int sample_count =
        std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_point = global.getPosition(duration);
    const Eigen::Vector3d final_previous =
        global.getPosition(duration * (sample_count - 1) / sample_count);
    double final_yaw = quaternionYaw(odom_orient_);
    planarYaw(final_previous, final_point, quaternionYaw(odom_orient_), final_yaw);
    const auto final_collision = map->checkFootprint(final_point, final_yaw);
    if (final_collision.status == GridMap::FootprintStatus::FREE ||
        final_collision.status == GridMap::FootprintStatus::OUT_OF_MAP)
      return true;
    if (final_collision.status != GridMap::FootprintStatus::OCCUPIED)
    {
      RCLCPP_ERROR(node_->get_logger(), "Global target footprint query is invalid.");
      return false;
    }

    for (int i = sample_count; i >= 0; --i)
    {
      const double t = duration * i / sample_count;
      const double previous_t = duration * std::max(0, i - 1) / sample_count;
      const Eigen::Vector3d point = global.getPosition(t);
      const Eigen::Vector3d previous = global.getPosition(previous_t);
      double yaw = final_yaw;
      planarYaw(previous, point, quaternionYaw(odom_orient_), yaw);
      if (map->checkFootprint(point, yaw).status == GridMap::FootprintStatus::FREE)
      {
        const Eigen::Vector3d requested_end = end_pt_;
        end_pt_ = point;
        global.global_duration_ = t;
        global.last_progress_time_ = std::min(global.last_progress_time_, t);
        RCLCPP_WARN(
            node_->get_logger(),
            "Global target [%.2f, %.2f, %.2f] is occupied; truncate to [%.2f, %.2f, %.2f].",
            requested_end.x(), requested_end.y(), requested_end.z(),
            end_pt_.x(), end_pt_.y(), end_pt_.z());
        return true;
      }
    }

    RCLCPP_ERROR(node_->get_logger(),
                 "Global target is occupied and no collision-free point exists on the global guide.");
    return false;
  }

  void EGOReplanFSM::checkCollisionCallback()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;
    
    const bool executing_existing_trajectory =
        exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ ||
        exec_state_ == GEN_NEW_TRAJ || exec_state_ == WAIT_TRAJ_START;
    if (!executing_existing_trajectory)
      return;

    const auto now = plannerNow();

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = 0.0;
    if (!getSynchronizedExecutionTime(t_cur))
    {
      if (exec_state_ == REPLAN_TRAJ)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "[EGO] Controller execution feedback timed out during replanning; command an explicit stop.");
        callEmergencyStop(odom_pos_);
        stop_sent_in_wait_replan_ = true;
        flag_escape_emergency_ = false;
        changeFSMExecState(EMERGENCY_STOP, "SAFETY_SYNC");
      }
      return;
    }

    const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();
    double t_cur_global = now.seconds();

    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      bool occ = false;
      const Eigen::Vector3d footprint_position = info->position_traj_.evaluateDeBoorT(t);
      const double next_t = std::min(info->duration_, t + time_step);
      const Eigen::Vector3d footprint_next = info->position_traj_.evaluateDeBoorT(next_t);
      double footprint_yaw = quaternionYaw(odom_orient_);
      occ = !planarYaw(footprint_position, footprint_next,
                       quaternionYaw(odom_orient_), footprint_yaw) ||
            !map->isFootprintFree(footprint_position, footprint_yaw);

      for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) || (planner_manager_->swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        double t_X = t_cur_global + (t - t_cur) -
                     planner_manager_->swarm_trajs_buf_.at(id).start_time_.seconds();
        t_X = std::clamp(t_X, 0.0, planner_manager_->swarm_trajs_buf_.at(id).duration_);
        Eigen::Vector3d swarm_pridicted = planner_manager_->swarm_trajs_buf_.at(id).position_traj_.evaluateDeBoorT(t_X);
        double dist = (footprint_position - swarm_pridicted).norm();

        if (dist < CLEARANCE)
        {
          occ = true;
          break;
        }
      }

      if (occ)
      {
        const bool replan_success = planFromCurrentTraj();

        if (replan_success) // Make a chance
        {
          changeFSMExecState(WAIT_TRAJ_START, "SAFETY");
          resetReplanTracking();
          publishSwarmTrajs(false);
          return;
        }
        else
        {
          if (force_wait_replan_)
          {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "[EGO] Replanning prerequisite is invalid: %s Stop and wait for a valid update.",
                force_wait_replan_reason_.c_str());
            callEmergencyStop(odom_pos_);
            stop_sent_in_wait_replan_ = true;
            changeFSMExecState(WAIT_REPLAN, "SAFETY");
            return;
          }

          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);

            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            RCLCPP_WARN(node_->get_logger(), "current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool use_polynomial_seed,
                                       bool random_polynomial_seed)
  {
    force_wait_replan_ = false;
    force_wait_replan_reason_.clear();

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> target_candidates;
    getLocalTarget(&target_candidates);

    double diagnostic_yaw = quaternionYaw(odom_orient_);
    const Eigen::Vector3d diagnostic_target =
        target_candidates.empty() ? end_pt_ : target_candidates.front().first;
    planarYaw(start_pt_, diagnostic_target, quaternionYaw(odom_orient_), diagnostic_yaw);
    if (target_candidates.size() != 1)
    {
      have_new_target_ = false;
      RCLCPP_ERROR(
          node_->get_logger(),
          "[EGO] Deterministic local-target selection produced %zu candidates. start=%s | requested=%s",
          target_candidates.size(),
          formatPointState(planner_manager_->grid_map_, start_pt_, diagnostic_yaw).c_str(),
          formatPointState(planner_manager_->grid_map_, local_target_pt_, diagnostic_yaw).c_str());
      return false;
    }

    local_target_pt_ = target_candidates.front().first;
    local_target_vel_ = target_candidates.front().second;
    const bool plan_and_refine_success = planner_manager_->reboundReplan(
        start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_,
        use_polynomial_seed, random_polynomial_seed);

    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success << endl;

    if (plan_and_refine_success)
    {

      auto info = &planner_manager_->local_data_;

      traj_utils::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();

      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      /* 1. publish traj to traj_server */
      bspline_pub_->publish(bspline);

      /* 2. publish traj to the next drone of swarm */

      /* 3. publish traj for visualization */
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_and_refine_success;
  }

  void EGOReplanFSM::publishSwarmTrajs(bool startup_pub)
  {
    auto info = &planner_manager_->local_data_;

    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.drone_id = planner_manager_->pp_.drone_id;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();

    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    if (startup_pub)
    {
      multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id; // zx-todo
      if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id + 1)
      {
        multi_bspline_msgs_buf_.traj.back() = bspline;
      }
      else if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id)
      {
        multi_bspline_msgs_buf_.traj.push_back(bspline);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d", (int)multi_bspline_msgs_buf_.traj.size(), planner_manager_->pp_.drone_id);
        // return plan_and_refine_success;
      }
      // swarm_trajs_pub_.publish(multi_bspline_msgs_buf_);
      swarm_trajs_pub_->publish(multi_bspline_msgs_buf_);
    }

    broadcast_bspline_pub_->publish(bspline);
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);

    return true;
  }

  void EGOReplanFSM::getLocalTarget(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> *candidate_targets)
  {
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> generated_candidates;
    auto &candidates = (candidate_targets != nullptr) ? *candidate_targets : generated_candidates;
    candidates.clear();
    force_wait_replan_ = false;
    force_wait_replan_reason_.clear();

    const double duration = planner_manager_->global_data_.global_duration_;
    const double max_vel = planner_manager_->pp_.max_vel_;
    const double max_acc = planner_manager_->pp_.max_acc_;
    const auto map = planner_manager_->grid_map_;
    if (!std::isfinite(duration) || duration <= 1e-6 ||
        !std::isfinite(max_vel) || max_vel <= 0.0 ||
        !std::isfinite(max_acc) || max_acc <= 0.0 || !map)
    {
      force_wait_replan_ = true;
      force_wait_replan_reason_ = "Global guide, dynamic limit, or local map is invalid.";
      return;
    }

    const double time_step = planning_horizen_ / (20.0 * max_vel);
    double nearest_distance = std::numeric_limits<double>::infinity();
    double nearest_time = std::clamp(
        planner_manager_->global_data_.last_progress_time_, 0.0, duration);
    double target_time = duration;
    bool horizon_reached = false;

    for (double t = nearest_time; t < duration; t += time_step)
    {
      const Eigen::Vector3d position = planner_manager_->global_data_.getPosition(t);
      const double distance = (position - start_pt_).norm();
      if (distance < nearest_distance)
      {
        nearest_distance = distance;
        nearest_time = t;
      }
      if (distance >= planning_horizen_)
      {
        local_target_pt_ = position;
        target_time = t;
        planner_manager_->global_data_.last_progress_time_ = nearest_time;
        horizon_reached = true;
        break;
      }
    }

    if (!horizon_reached)
    {
      local_target_pt_ = end_pt_;
      target_time = duration;
    }

    auto targetIsFree = [&](const Eigen::Vector3d &target) {
      double yaw = quaternionYaw(odom_orient_);
      planarYaw(odom_pos_, target, quaternionYaw(odom_orient_), yaw);
      return map->isFootprintFree(target, yaw);
    };

    if (!targetIsFree(local_target_pt_))
    {
      bool found_free_target = false;
      for (double dt = 0.0; dt <= duration; dt += time_step)
      {
        const double forward_time = target_time + dt;
        if (forward_time <= duration)
        {
          const Eigen::Vector3d target = planner_manager_->global_data_.getPosition(forward_time);
          if (targetIsFree(target))
          {
            local_target_pt_ = target;
            target_time = forward_time;
            found_free_target = true;
            break;
          }
        }

        const double backward_time = target_time - dt;
        if (backward_time >= std::max(0.0, nearest_time))
        {
          const Eigen::Vector3d target = planner_manager_->global_data_.getPosition(backward_time);
          if (targetIsFree(target))
          {
            local_target_pt_ = target;
            target_time = backward_time;
            found_free_target = true;
            break;
          }
        }
      }

      if (!found_free_target)
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[EGO] Local target is occupied and no free point exists on the global guide; keep the SCAN target for optimization.");
    }

    const double braking_distance = max_vel * max_vel / (2.0 * max_acc);
    local_target_vel_ = (end_pt_ - local_target_pt_).norm() < braking_distance
                            ? Eigen::Vector3d::Zero()
                            : planner_manager_->global_data_.getVelocity(target_time);
    candidates.emplace_back(local_target_pt_, local_target_vel_);
  }

} // namespace ego_planner
