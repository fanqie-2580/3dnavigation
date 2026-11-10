#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <utility>
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include "visualization_msgs/msg/marker.hpp"

#include "bspline_opt/bspline_optimizer.h"
#include "plan_env/grid_map.h"
#include "traj_utils/msg/bspline.hpp"
#include "traj_utils/msg/multi_bsplines.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "traj_utils/msg/data_disp.hpp"
#include "traj_utils/msg/trajectory_execution_state.hpp"
#include "ego_planner/planner_manager.h"
#include "traj_utils/planning_visualization.h"

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START,
      WAIT_REPLAN,
      WAIT_TRAJ_START
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::msg::DataDisp data_disp_;
    traj_utils::msg::MultiBsplines multi_bspline_msgs_buf_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int waypoint_num_, wp_id_;
    double planning_horizen_, planning_horizen_time_;
    double emergency_time_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;
    bool continuous_replan_;
    std::string reference_path_topic_;
    double reference_path_z_offset_;
    double reference_path_start_skip_dist_;
    double reference_path_sample_spacing_;
    std::string odom_world_frame_;
    double odom_z_offset_;
    double replan_distance_;
    int exec_interval_ms_;
    int safety_interval_ms_;
    int max_replan_failures_;
    double execution_state_timeout_;

    /* planning data */
    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};
    int replan_failure_count_{0};
    bool stop_sent_in_wait_replan_{false};
    bool planning_failure_exhausted_{false};
    bool force_wait_replan_{false};
    bool reference_progress_initialized_{false};
    std::string force_wait_replan_reason_;
    std::vector<Eigen::Vector3d> reference_path_points_;
    std::vector<double> reference_path_arc_lengths_;
    double reference_path_total_length_{0.0};
    double reference_path_progress_s_{0.0};
    uint64_t reference_path_id_{0};
    uint64_t blocked_map_revision_{0};
    uint64_t blocked_reference_path_id_{0};
    bool have_execution_state_{false};
    int64_t execution_traj_id_{-1};
    uint8_t trajectory_execution_state_{traj_utils::msg::TrajectoryExecutionState::WAIT_TRAJECTORY};
    double trajectory_execution_time_{0.0};
    bool trajectory_execution_frozen_{false};
    rclcpp::Time execution_state_received_time_;

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> wps_;
    int current_wp_;

    bool flag_escape_emergency_;

    /* ROS utils */
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    nav_msgs::msg::Path::SharedPtr pending_reference_path_;
    rclcpp::Subscription<traj_utils::msg::MultiBsplines>::SharedPtr swarm_trajs_sub_;
    rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr broadcast_bspline_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;
    rclcpp::Subscription<traj_utils::msg::TrajectoryExecutionState>::SharedPtr execution_state_sub_;

    // rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_;
    // rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr new_pub_;
    rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr bspline_pub_;
    rclcpp::Publisher<traj_utils::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<traj_utils::msg::MultiBsplines>::SharedPtr swarm_trajs_pub_;
    rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr broadcast_bspline_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /* helper functions */
    bool callReboundReplan(bool use_polynomial_seed, bool random_polynomial_seed);
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromCurrentTraj(const int trial_times = 1);
    bool adjustGlobalTargetIfOccupied();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();
    bool lookupTransformSafe(const std::string &target_frame, const std::string &source_frame,
                             const rclcpp::Time &stamp, geometry_msgs::msg::TransformStamped &transform);
    bool getOdomStateInWorld(const nav_msgs::msg::Odometry &odom_msg, Eigen::Vector3d &pos,
                             Eigen::Quaterniond &orientation, Eigen::Vector3d &linear_vel,
                             Eigen::Vector3d &angular_vel);
    rclcpp::Time plannerNow() const;
    void resetReplanTracking();
    void finishCurrentTarget();
    bool getSynchronizedExecutionTime(double &execution_time, uint8_t *execution_state = nullptr) const;
    bool setReferencePath(const std::vector<Eigen::Vector3d> &waypoints);
    bool setTruncatedReferencePathFromGlobalTrajectory();
    bool projectReferenceProgress(const Eigen::Vector3d &position, bool initialize);
    bool buildSparseRemainingGuide(std::vector<Eigen::Vector3d> &waypoints);
    bool canRecoverFromMapRevision();

    void readGivenWps();
    void planNextWaypoint(const Eigen::Vector3d next_wp);
    void getLocalTarget(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> *candidate_targets = nullptr);

    /* ROS functions */
    void execFSMCallback();
    void checkCollisionCallback();
    void waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg);
    void pathCallback(const std::shared_ptr<const nav_msgs::msg::Path> &msg);
    void triggerCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg);
    void odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg);
    void swarmTrajsCallback(const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg);
    void BroadcastBsplineCallback(const std::shared_ptr<const traj_utils::msg::Bspline> &msg);
    void executionStateCallback(
        const std::shared_ptr<const traj_utils::msg::TrajectoryExecutionState> &msg);

    bool checkCollision();
    void publishSwarmTrajs(bool startup_pub);

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM()
    {
    }

    void init(const rclcpp::Node::SharedPtr &node);
    inline void init(rclcpp::Node::SharedPtr &node)
    {
      init(static_cast<const rclcpp::Node::SharedPtr &>(node));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif
