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

#ifndef DDDMR_LOCAL_PLANNER_H_
#define DDDMR_LOCAL_PLANNER_H_

/*For graph*/
#include <unordered_map>
#include <set>
#include <queue> 
#include <angles/angles.h>
#include <pcl/kdtree/kdtree_flann.h>
/*For perception plugin*/
#include <perception_3d/perception_3d_ros.h>

/*For critics plugin*/
#include <mpc_critics/mpc_critics_ros.h>

/*For trajectory generators plugin*/
#include <trajectory_generators/trajectory_generators_ros.h>

/*For edge markers*/
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

//@planner state
#include <dddmr_sys_core/dddmr_enum_states.h>

namespace local_planner {

class Local_Planner : public rclcpp::Node {

    public:
      Local_Planner(const std::string& name);
      
      void initial(
        const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d,
        const std::shared_ptr<mpc_critics::MPC_Critics_ROS>& mpc_critics,
        const std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS>& trajectory_generators);

      ~Local_Planner();

      void setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& orig_global_plan);
      dddmr_sys_core::PlannerState computeVelocityCommand(std::string traj_gen_name, base_trajectory::Trajectory& best_traj);
      void getBestTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& best_traj);

      //@ shared data for trajectory generator, we manage the variables by this way for future changed to ROS2
      std::shared_ptr<trajectory_generators::TrajectoryGeneratorSharedData> traj_shared_data_;
      
      bool isGoalReached();
      
      bool isInitialHeadingAligned();
      bool isGoalHeadingAligned();

      void updateGlobalPose();
      geometry_msgs::msg::TransformStamped getGlobalPose();
      
    private: 
      
      rclcpp::Clock::SharedPtr clock_;

      rclcpp::CallbackGroup::SharedPtr cbs_group_;
      rclcpp::CallbackGroup::SharedPtr tf_listener_group_;

      //@ for percertion
      std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d_ros_;
      //@ for critics
      std::shared_ptr<mpc_critics::MPC_Critics_ROS> mpc_critics_ros_;
      //@ for trajectory generator
      std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS> trajectory_generators_ros_;

      std::string global_frame_;
      std::string robot_frame_;
      std::string odom_topic_;
      std::string odom_topic_qos_;
      
      /*For cuboid visualization*/
      visualization_msgs::msg::MarkerArray robot_cuboid_;
      visualization_msgs::msg::Marker marker_edge_;

      /*Original point cloud*/
      pcl::PointCloud<pcl::PointNormal>::Ptr ground_with_normals_;
      
      /*global plan from global planner*/
      std::vector<geometry_msgs::msg::PoseStamped> global_plan_;
      pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_global_plan_; 
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_global_plan_;
      
      /*store global pose*/
      geometry_msgs::msg::TransformStamped trans_gbl2b_;
      
      /*Compute shortest between robot heading and given pose*/
      double getShortestAngleFromPose2RobotHeading(tf2::Transform m_pose);

      /*Pub*/
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_robot_cuboid_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_aggregate_observation_;
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_prune_plan_;
      rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_accepted_trajectory_pose_array_;
      rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_best_trajectory_pose_;
      rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_trajectory_pose_array_;
      //ros::Publisher pub_pc_normal_;
      //ros::Publisher pub_trajectory_cuboids_;
      

      /*Sub*/
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_ros_sub_;

      /*cb*/
      void cbOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

      void trajectory2posearray_cuboids(const base_trajectory::Trajectory& a_traj, 
                                      geometry_msgs::msg::PoseArray& pose_arr,
                                      pcl::PointCloud<pcl::PointXYZ>& cuboids);
      
      void parseCuboid();
      //void normal2quaternion();
      void prunePlan(double forward_distance, double backward_distance);
      int selectPruneAnchorIndex(
        int nearest_idx,
        const pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr& local_obs_kdtree) const;
      double getDistanceBTWPoseStamp(const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b);
      // [GO2SmoothBypassV2] Densify short global plans to avoid prune-plan starvation.
      std::vector<geometry_msgs::msg::PoseStamped> densifyPlanIfNeeded(
        const std::vector<geometry_msgs::msg::PoseStamped>& orig_global_plan) const;
      // [GO2ReactiveBypassV7] If all sampled trajectories are rejected, generate an immediate side-escape command
      // from local obstacle distribution instead of waiting for recovery.
      bool computeReactiveSideEscape(
        const std::string & traj_gen_name,
        base_trajectory::Trajectory & best_traj);
      // [FARStyleBypass] Proactively bias feasible trajectories away from denser obstacle side.
      bool applyProactiveSideBypassBias(
        const std::string & traj_gen_name,
        base_trajectory::Trajectory & best_traj);
      bool analyzeLocalBypassField(
        int & front_points,
        int & left_points,
        int & right_points,
        double & left_metric,
        double & right_metric) const;

      bool compute_best_trajectory_in_odomCb_;

      /* Variables for prune. I have no idea to put these variables now.
         This should be optimize in future. The variables affect the control behavior.
         The variable should be adapt to vehicle speed!!!
      */
      double forward_prune_, backward_prune_, heading_tracking_distance_, heading_align_angle_;

      /*Timer for robust system design*/
      double prune_plane_timeout_;
      rclcpp::Time last_valid_prune_plan_;
      bool got_odom_;

      double xy_goal_tolerance_, yaw_goal_tolerance_;
      double controller_frequency_;
      // [SmoothLaneChange] If true, do not force WAIT when a safe local trajectory exists.
      bool blocked_wait_requires_all_trajectories_fail_;
      // [SmoothLaneChange] If true, keep moving when blocked-replanning is raised but a safe trajectory exists.
      bool blocked_replanning_keep_moving_;
      // [GO2SmoothBypassV2] Robustness knobs for sparse/shifted global plans.
      double short_plan_interpolation_resolution_;
      double max_prune_start_deviation_;
      // [FARStyleAnchor] Re-select prune anchor into locally clear region along global path.
      bool enable_prune_anchor_search_;
      double prune_anchor_check_radius_;
      int prune_anchor_max_blocked_points_;
      int prune_anchor_search_points_;
      double prune_anchor_min_z_;
      double prune_anchor_max_z_;
      int prune_anchor_backtrack_tolerance_;
      int last_prune_anchor_index_;
      bool has_last_prune_anchor_;
      // [GO2ReactiveBypassV7] Immediate side-escape fallback params/state.
      bool enable_reactive_side_escape_;
      std::string reactive_escape_generator_name_;
      double reactive_escape_front_min_x_;
      double reactive_escape_front_max_x_;
      double reactive_escape_corridor_half_width_;
      int reactive_escape_min_front_points_;
      double reactive_escape_side_scan_min_x_;
      double reactive_escape_side_scan_max_x_;
      double reactive_escape_side_min_abs_y_;
      double reactive_escape_side_max_abs_y_;
      double reactive_escape_min_z_;
      double reactive_escape_max_z_;
      double reactive_escape_cmd_vx_;
      double reactive_escape_cmd_vy_;
      double reactive_escape_cmd_w_;
      double reactive_escape_side_hold_sec_;
      double reactive_escape_side_bias_;
      int reactive_escape_clear_front_points_;
      double reactive_escape_min_active_sec_;
      int reactive_escape_last_dir_;
      rclcpp::Time reactive_escape_last_switch_time_;
      bool reactive_escape_active_;
      int reactive_escape_active_dir_;
      rclcpp::Time reactive_escape_active_since_;
      // [FARStyleBypass] Proactive side-bias knobs for smoother "left blocked -> go right" behavior.
      bool enable_proactive_side_bypass_;
      int proactive_front_min_points_;
      double proactive_min_lateral_speed_;
      double proactive_min_forward_speed_;
      bool proactive_force_forward_progress_;
      double proactive_hard_min_forward_speed_;
      double proactive_low_forward_penalty_;
      double proactive_max_extra_cost_;
      double proactive_side_bias_;

      rclcpp::Time control_loop_time_;

      std::map<std::string, std::vector<base_trajectory::Trajectory>> rejected_trajectories_;

    protected:

      std::shared_ptr<tf2_ros::TransformListener> tfl_;
      std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;  ///< @brief Used for transforming point clouds
      nav_msgs::msg::Odometry robot_state_;
      std::shared_ptr<std::vector<base_trajectory::Trajectory>> trajectories_;
      nav_msgs::msg::Path prune_plan_;
      pcl::PointCloud<pcl::PointXYZI> pcl_prune_plan_; //@ will be copied to perception_ros, so do not use shared_ptr
      std::string name_;
      
};

} // end of name space

#endif  // DDDMR_LOCAL_PLANNER_H_
