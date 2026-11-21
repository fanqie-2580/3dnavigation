// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <atomic>
#include <chrono>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <pcl/io/pcd_io.h>
#include <sys/stat.h>
#include "preprocess.h"
#include "voxel_map_util.hpp"

using namespace std::chrono_literals;

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

// Helper: convert double seconds to builtin_interfaces::msg::Time
static inline builtin_interfaces::msg::Time sec_to_stamp(double t) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(t);
    stamp.nanosec = static_cast<uint32_t>((t - stamp.sec) * 1e9);
    return stamp;
}

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   time_sync_en = false, extrinsic_est_en = true, path_en = true;
double lidar_time_offset = 0.0;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string  lid_topic, imu_topic, lid_topic2, imu_topic2;
bool multi_lidar_en = false;
bool lidar2_en = false;
double lidar2_stamp_bias_sec = 0.0;  // measured mid-odin header offset ~0; 0.065 caused ~60ms end skew
double lidar2_max_sync_dt = 0.20;          // phase drift measured up to ~0.13s
double lidar2_soft_wait_sec = 0.18;        // >1 Odin period so late frames can arrive
double lidar2_bias_ema = 0.05;             // online stamp-bias adaptation rate
double lidar2_cov_scale = 4.0;
double bundle_center_dt = 0.0;             // last Mid↔Odin end/center residual (signed)
double bundle_pair_score = -1.0;           // |residual| when paired; <0 = Mid-only
double bundle_mid_push_wall = -1.0;
double bundle_end_dt = 0.0;                // mid_end - (odin_end + bias)
// Hold last dense Odin to bridge unpaired / sparse gaps (stops body flicker).
PointCloudXYZI::Ptr last_odin_fused(new PointCloudXYZI());
double last_odin_fused_wall = -1.0;
size_t last_odin_good_n = 0;
// Dual-tune knobs: defaults = 2e98c1d; override via config/dual_mid_odin_tune.py
// (launch → dual_tune.*). Edit the .py and restart — no rebuild.
int odin_min_pts = 4000;
double odin_min_frac = 0.35;
float odin_body_z_min = -2.0f;
float odin_body_z_max = 3.5f;
double odin_static_gyro = 0.05;
int odin_iekf_max_pts = 400;
double odin_iekf_max_end_dt = 0.05;
bool odin_s2_enable = true;
double odin_info_weight = 0.15;
double odin_range_ref = 6.0;
double odin_max_residual = 0.05;
double odin_s2_gyr_gate = 0.12;
double iekf_max_upd = 0.08;
double iekf_max_rot = 0.08;
double iekf_max_vel = 1.3;   // corridor return root: along-track |v| runaway
double iekf_s2_max_upd = 0.015;
double iekf_s2_max_rot = 0.025;
double map_jump_upd = 0.05;
double map_jump_frame = 0.12;
int consec_bad_freeze = 2;
float map_max_range = 25.0f;
bool map_require_still = true;        // skip Global densify when moving
double map_still_gyr = 0.12;          // rad/s
bool map_use_downsample = true;       // Global Mid downs (not dense undistort)
bool odin_global_insert_en = false;
bool odin_global_require_s2 = true;   // full Odin densify prefers accepted S2
// Fill ground without resurrecting double walls:
//  - still densify: full Odin when gyro low even if S2 did not run
//  - walk floor densify: while moving, insert only near-floor Odin points
bool odin_global_still_densify_en = true;
double odin_global_still_gyr = 0.18;
bool odin_global_walk_floor_densify_en = true;
float odin_global_walk_floor_z_min = -0.35f;
float odin_global_walk_floor_z_max = 0.45f;
int odin_global_walk_floor_stride = 2; // subsample while walking
double odin_s2_max_clip_ratio = 0.40; // reject S2 / densify if clip_r above
bool odin_body_pub_en = false;
// When end_dt+gyro look Bundle-safe, run a 1-iter Mid S1 then full joint S2
// (closer to official single update without dropping Mid lock on hard frames).
bool shallow_s1_en = true;
double shallow_s1_end_dt = 0.05;
double shallow_s1_gyr = 0.08;
int shallow_s1_iters = 1;
int iekf_shallow_s1 = 0;
double odin_wall_nz_thr = 0.40;      // |nz|<thr → wall-like Odin residual
double odin_wall_info_boost = 1.8;   // strengthen Odin walls in joint H
double odin_floor_info_scale = 0.35; // soften Odin floors in joint H
bool odin_voxel_insert_en = false;
bool odin_voxel_require_still = true; // official-like: false → insert while walking
double odin_voxel_max_clip = 0.35;
int odin_voxel_max_pts = 200;
float odin_voxel_body_z_min = -0.05f; // skip near-floor body points
double odin_voxel_cov_scale = 4.0;
int iekf_odin_voxel_ins = 0;
// Official MULTI Bundle: one joint Mid+Odin IEKF (no Mid-only S1 then S2).
bool odin_joint_only_en = false;
int iekf_joint_only_n = 0;
// Corridor Z root cause:
// - Mid floors bias Z; soft post-update that *fully follows Odin Δz* inherits
//   Odin drift (outbound ≈+12cm, return ≈−14cm). One-sided only made outbound
//   climb. Aggressive catch yanks broke Mid match (No Effective Points).
// Fix: target = flat_ref + clip(OdinΔz, ±band); mild floor soften + H_z damp;
//      small soft steps only (no catch open).
bool residual_z_en = true;
double residual_z_flat_z_alpha = 0.22;
double residual_z_flat_vz_alpha = 1.0;
double residual_z_flat_ba_z_alpha = 0.25;
double residual_z_flat_max_step = 0.015;
double residual_z_floor_nz_thr = 0.85;
double residual_z_floor_info_scale = 0.45; // mild always (Point-LIO-like)
double residual_z_h_dampen = 0.50;         // single-level H_z scale; multilevel → 1.0
double residual_z_dz_tol = 0.06;
double residual_z_floor_ratio_thr = 0.03;
// go2 residual_z_v6.3: after stairs, full H_z so each floor re-locks absolute Z.
// Without this, constant H_z*=0.12 leaves Z to IMU → upstairs sink / return rise.
double residual_z_multilevel_thr = 1.5;
double residual_z_vz_alpha = 0.75;
bool residual_z_multilevel = false;
bool residual_z_flat_ref_init = false;
double residual_z_flat_z_ref = 0.0;
double residual_z_hard_band = 0.0; // 0=off; clamp |z-flat_ref| after soft Z
int n_residual_z_hard = 0;
bool last_floor_valid = false;
double last_floor_z = 0.0;
int last_floor_n = 0;
double last_floor_ratio = 0.0;
bool odin_z_factor_en = true;
bool odin_z_replace_en = false; // true: pose Z := Odin odom Z (init-aligned), no algo Z
// Single-Odin / cam_z: also pin XY to Odin Δ (same anchors). Z-only leaves
// PV-LIO free to walk off by tens of meters on stairs while Z looks fine.
bool odin_xyz_replace_en = false;
// Soft XY pull toward Odin Δxy (init-aligned). Fixes Mid corridor scale shrink
// on return (~5m vs Odin onboard) without full pose hijack.
bool odin_xy_factor_en = false;
double odin_xy_factor_alpha = 0.25;
double odin_xy_factor_max_step = 0.05; // m per axis per frame
long long n_odin_xy_factor = 0;
// Stairs/open FOV: Mid match can die while Odin odom stays good. Freezing XY to
// last_good then still replacing Z → upper-floor XY + stair Z → permanent NEP.
// On bad frames, hold full XYZ from Odin (same init anchors) + zero vel.
bool odin_pose_hold_on_bad = true;
// Multifloor: follow Odin Z while |vz| >= thr (stairs); on flat, keep fusion Z
// and re-anchor offset so onboard slow sink is absorbed (not tracked).
bool odin_z_flat_hold_en = true;
double odin_z_flat_vz_thr = 0.03; // m/s; stairs ~0.1, upper-floor sink ~0.002
// Multifloor: Z-only Odin left Mid roll/pitch → tilted cloud.
// Full att replace also overwrites yaw → Mid rays miss Mid map → NEP/drift.
// Default: take Odin roll/pitch (init-aligned), keep Mid yaw.
bool odin_att_replace_en = true;
bool odin_att_rp_only = true;
std::string odin_odom_topic = "/odin1/odometry";
double odin_z_factor_alpha = 0.50;
double odin_z_factor_max_dt = 0.30;
double odin_z_factor_max_innov = 0.05;  // keep soft — large steps broke Mid/XY
double odin_z_factor_band = 0.04;     // |Odin Δz| allowed around flat_ref
double odin_z_factor_catch_err = 0.0; // 0 = disable catch open
double odin_z_factor_catch_frac = 1.0;
double odin_z_factor_catch_max = 0.05;
double odin_z_factor_catch_alpha = 0.50;
int odin_z_factor_catch_min_feat = 30;
bool odin_z_factor_one_sided = false; // two-sided soft (outbound-safe)
double odin_z_factor_floor_scale = 0.30;
std::mutex mtx_odin_odom;
double odin_odom_stamp = -1.0;
double odin_odom_x = 0.0;
double odin_odom_y = 0.0;
double odin_odom_z = 0.0;
double odin_odom_qw = 1.0;
double odin_odom_qx = 0.0;
double odin_odom_qy = 0.0;
double odin_odom_qz = 0.0;
bool odin_odom_have = false;
bool odin_att_align_valid = false;
Eigen::Matrix3d odin_R_align = Eigen::Matrix3d::Identity();
long long n_odin_att_replace = 0;
double last_pf_mid_z = 0.0;
double last_pf_odin_z = 0.0;
double last_pf_odin_stamp = -1.0;
bool last_pf_z_valid = false;
long long n_odin_z_flat_hold = 0;
double odin_z_anchor_mid = 0.0;   // fusion z at factor init
double odin_z_anchor_odin = 0.0;  // odin z at factor init
double odin_xy_anchor_mid_x = 0.0;
double odin_xy_anchor_mid_y = 0.0;
double odin_xy_anchor_odin_x = 0.0;
double odin_xy_anchor_odin_y = 0.0;
bool odin_xy_anchor_valid = false;
long long n_odin_z_factor = 0;
long long n_odin_pose_hold = 0;
int iekf_reject_s2 = 0;
int iekf_accept_s2 = 0;
int iekf_odin_res_clip = 0;
bool iekf_odin_stage = false;              // observation_model: clip Odin residuals
int last_mid_effct_feat_num = 0;           // Mid stage effective residuals
double frame_gyr_mean = 0.0;
// Global display-map insert gate (hold/jump left permanent ~1m colored ghosts).
V3D last_global_map_pos = Zero3d;
bool last_global_map_pose_ok = false;
int map_ins_ok = 0, map_skip_hold = 0, map_skip_jump = 0, map_skip_bad = 0;
int iekf_reject_jump = 0;
// After Mid match fails, IMU-only prop pollutes voxel map → reject_s1 cascade.
// Freeze to last good lidar pose (FAST_LIO-like: no map update without valid residual).
state_ikfom last_good_lidar_state;
esekfom::esekf<state_ikfom, 12, input_ikfom>::cov last_good_P;
bool have_last_good_lidar = false;
bool have_last_good_P = false;
int consec_iekf_bad = 0;
int iekf_reject_vel = 0;
// Per-frame Odin residual inflation (set when fusing).
double odin_frame_cov_scale = 4.0;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_lidar2 = -1.0, last_timestamp_imu = -1.0, last_timestamp_imu2 = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_surf_min = 0;
double total_distance = 0, lidar_end_time = 0, lidar_end_time2 = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;


vector<vector<int>>  pointSearchInd_surf;
vector<PointVector>  Nearest_Points;
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer;
deque<double> time_buffer2;
deque<PointCloudXYZI::Ptr> lidar_buffer2;
deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer2;
M3D LiDAR2_R_wrt_LiDAR1 = Eye3d;
V3D LiDAR2_T_wrt_LiDAR1 = Zero3d;
M3D Lidar2_R_wrt_IMU2 = Eye3d;
V3D Lidar2_T_wrt_IMU2 = Zero3d;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;
std::vector<M3D> var_down_body;

pcl::VoxelGrid<PointType> downSizeFilterSurf;

std::vector<float> nn_dist_in_feats;
std::vector<float> nn_plane_std;
PointCloudXYZI::Ptr feats_with_correspondence(new PointCloudXYZI());

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

// params for voxel mapping algorithm
double min_eigen_value = 0.003;
int max_layer = 0;

int max_cov_points_size = 50;
int max_points_size = 50;
double sigma_num = 2.0;
double max_voxel_size = 1.0;
std::vector<int> layer_size;

double ranging_cov = 0.0;
double angle_cov = 0.0;
std::vector<double> layer_point_size;

bool publish_voxel_map = false;
bool publish_global_cloud_map = false;
int publish_max_voxel_layer = 0;
double global_map_leaf_size = 0.1; // 0.08+huge span triggered PCL VoxelGrid overflow
// RViz-only Global publish (full-res map kept for /save_map).
double global_map_pub_leaf_size = 0.20;
double global_map_pub_period = 2.0;   // wall seconds between /Global_cloud_map pubs
int global_map_pub_max_pts = 350000;

bool has_save_opt = false;
bool pcd_save_en = false;
std::string pcd_save_path;
std::atomic<bool> save_map_busy{false};
std::atomic<bool> global_remap_busy{false};

std::unordered_map<VOXEL_LOC, OctoTree *> voxel_map;
std::mutex global_cloud_map_mtx;
PointCloudXYZI::Ptr global_cloud_map(new PointCloudXYZI());
// New world points since last 1Hz tick; timer voxelizes ONLY these (anti-lag).
PointCloudXYZI::Ptr global_cloud_map_pending(new PointCloudXYZI());
pcl::VoxelGrid<PointType> downSizeFilterGlobal;
/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<Preprocess> p_pre2(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig)
{
    flg_exit = true;
    RCLCPP_WARN(rclcpp::get_logger("pv_lio"), "catch sig %d", sig);
    sig_buffer.notify_all();
}

void odin_odom_cbk(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(mtx_odin_odom);
    odin_odom_stamp = rclcpp::Time(msg->header.stamp).seconds();
    odin_odom_x = msg->pose.pose.position.x;
    odin_odom_y = msg->pose.pose.position.y;
    odin_odom_z = msg->pose.pose.position.z;
    odin_odom_qw = msg->pose.pose.orientation.w;
    odin_odom_qx = msg->pose.pose.orientation.x;
    odin_odom_qy = msg->pose.pose.orientation.y;
    odin_odom_qz = msg->pose.pose.orientation.z;
    odin_odom_have = true;
}

Eigen::Matrix3d odin_odom_R_locked()
{
    Eigen::Quaterniond q(odin_odom_qw, odin_odom_qx, odin_odom_qy, odin_odom_qz);
    if (q.norm() < 1e-6)
        return Eigen::Matrix3d::Identity();
    q.normalize();
    return q.toRotationMatrix();
}

// Level fusion to Odin gravity while keeping Mid heading in the horizontal plane.
// (Euler roll/pitch+yaw mix was wrong: flat start printed pitch~50°.)
bool apply_odin_att_replace(state_ikfom &s)
{
    if (!odin_att_replace_en)
        return false;
    Eigen::Matrix3d R_o;
    {
        std::lock_guard<std::mutex> lk(mtx_odin_odom);
        if (!odin_odom_have)
            return false;
        R_o = odin_odom_R_locked();
    }
    if (!odin_att_align_valid)
    {
        odin_R_align = s.rot.toRotationMatrix() * R_o.transpose();
        odin_att_align_valid = true;
        std::cout << "[pv-z] odin_att_replace init align gravity_level rp_only="
                  << odin_att_rp_only << std::endl;
    }
    const Eigen::Matrix3d R_o_al = odin_R_align * R_o;
    Eigen::Matrix3d R_new = R_o_al;
    if (odin_att_rp_only)
    {
        // World +Z from Odin-aligned body; keep Mid forward projected to that plane.
        Eigen::Vector3d z_des = R_o_al.col(2);
        const double zn = z_des.norm();
        if (zn < 1e-6)
            return false;
        z_des /= zn;
        Eigen::Vector3d x_mid = s.rot.toRotationMatrix().col(0);
        Eigen::Vector3d x_new = x_mid - x_mid.dot(z_des) * z_des;
        if (x_new.norm() < 1e-6)
        {
            x_mid = s.rot.toRotationMatrix().col(1);
            x_new = x_mid - x_mid.dot(z_des) * z_des;
        }
        if (x_new.norm() < 1e-6)
            return false;
        x_new.normalize();
        Eigen::Vector3d y_new = z_des.cross(x_new);
        y_new.normalize();
        x_new = y_new.cross(z_des);
        x_new.normalize();
        R_new.col(0) = x_new;
        R_new.col(1) = y_new;
        R_new.col(2) = z_des;
    }
    Eigen::Quaterniond qm(R_new);
    qm.normalize();
    s.rot = SO3(qm);
    ++n_odin_att_replace;
    if ((n_odin_att_replace % 50) == 1)
    {
        const vect3 ypr = SO3ToEuler(s.rot);
        std::cout << "[pv-z] odin_att_replace ypr=(" << ypr[0] << ","
                  << ypr[1] << "," << ypr[2] << ") grav=" << odin_att_rp_only
                  << " n=" << n_odin_att_replace << std::endl;
    }
    return true;
}

// Signed (target - fusion_z): >0 means fusion below Odin (needs raise).
double odin_z_signed_err(const state_ikfom &s)
{
    if (!odin_z_factor_en || !last_pf_z_valid)
        return 0.0;
    double z_o = 0.0;
    {
        std::lock_guard<std::mutex> lk(mtx_odin_odom);
        if (!odin_odom_have)
            return 0.0;
        z_o = odin_odom_z;
    }
    const double target_z =
        odin_z_anchor_mid + (z_o - odin_z_anchor_odin);
    return target_z - s.pos(2);
}

// True when Odin vertical speed looks like flat-floor drift (not stairs).
bool odin_z_is_flat_vz(double z_o, double t_o, double *vz_out)
{
    if (vz_out)
        *vz_out = 0.0;
    if (!odin_z_flat_hold_en || last_pf_odin_stamp < 0.0)
        return false;
    const double dt = t_o - last_pf_odin_stamp;
    if (dt < 1e-3 || dt > 1.0)
        return false;
    const double vz = (z_o - last_pf_odin_z) / dt;
    if (vz_out)
        *vz_out = vz;
    return std::fabs(vz) < odin_z_flat_vz_thr;
}

// Absorb Odin slow Z drift into the init offset; leave fusion Z unchanged.
void absorb_odin_z_flat(double z_o, double t_o, double fusion_z)
{
    odin_z_anchor_odin = z_o - (fusion_z - odin_z_anchor_mid);
    last_pf_mid_z = fusion_z;
    last_pf_odin_z = z_o;
    last_pf_odin_stamp = t_o;
    ++n_odin_z_flat_hold;
}

// Dual XY/map unchanged. Z follows Odin Δz (keeps Mid map anchor; abs Z shears).
// z = z_mid0 + (z_odin - z_odin0)
void apply_odin_z_replace(double /*stamp*/)
{
    if (!odin_z_replace_en)
        return;
    double z_o = 0.0;
    {
        std::lock_guard<std::mutex> lk(mtx_odin_odom);
        if (!odin_odom_have)
            return;
        z_o = odin_odom_z;
    }
    state_ikfom s = kf.get_x();
    if (!last_pf_z_valid)
    {
        last_pf_z_valid = true;
        odin_z_anchor_mid = s.pos(2);
        odin_z_anchor_odin = z_o;
        std::cout << "[pv-z] odin_z_replace Δz init mid=" << odin_z_anchor_mid
                  << " odin=" << odin_z_anchor_odin << std::endl;
        return;
    }
    const double z_new = odin_z_anchor_mid + (z_o - odin_z_anchor_odin);
    s.pos(2) = z_new;
    s.vel(2) = 0.0;
    kf.change_x(s);
    ++n_odin_z_factor;
    last_pf_mid_z = z_new;
    last_pf_odin_z = z_o;
    if ((n_odin_z_factor % 100) == 1)
        std::cout << "[pv-z] odin_dz z=" << z_new << " odin=" << z_o
                  << " n=" << n_odin_z_factor << std::endl;
}

// Soft XY: pull Mid toward Odin init-aligned XY (scale/yaw assist on return).
void apply_odin_xy_factor()
{
    if (!odin_xy_factor_en)
        return;
    double x_o = 0.0, y_o = 0.0;
    {
        std::lock_guard<std::mutex> lk(mtx_odin_odom);
        if (!odin_odom_have)
            return;
        x_o = odin_odom_x;
        y_o = odin_odom_y;
    }
    state_ikfom s = kf.get_x();
    if (!odin_xy_anchor_valid)
    {
        odin_xy_anchor_mid_x = s.pos(0);
        odin_xy_anchor_mid_y = s.pos(1);
        odin_xy_anchor_odin_x = x_o;
        odin_xy_anchor_odin_y = y_o;
        odin_xy_anchor_valid = true;
        if (!last_pf_z_valid)
        {
            // Keep Z-replace anchors consistent if enabled later.
            odin_z_anchor_mid = s.pos(2);
            {
                std::lock_guard<std::mutex> lk(mtx_odin_odom);
                odin_z_anchor_odin = odin_odom_z;
            }
            last_pf_z_valid = true;
            last_pf_mid_z = s.pos(2);
            last_pf_odin_z = odin_z_anchor_odin;
        }
        std::cout << "[pv-xy] odin_xy_factor init mid=(" << odin_xy_anchor_mid_x
                  << "," << odin_xy_anchor_mid_y << ") odin=("
                  << odin_xy_anchor_odin_x << "," << odin_xy_anchor_odin_y
                  << ")" << std::endl;
        return;
    }
    const double tgt_x = odin_xy_anchor_mid_x + (x_o - odin_xy_anchor_odin_x);
    const double tgt_y = odin_xy_anchor_mid_y + (y_o - odin_xy_anchor_odin_y);
    double dx = tgt_x - s.pos(0);
    double dy = tgt_y - s.pos(1);
    const double lim = std::max(0.0, odin_xy_factor_max_step);
    if (lim > 0.0)
    {
        if (std::fabs(dx) > lim) dx = std::copysign(lim, dx);
        if (std::fabs(dy) > lim) dy = std::copysign(lim, dy);
    }
    const double a = std::max(0.0, std::min(1.0, odin_xy_factor_alpha));
    s.pos(0) += a * dx;
    s.pos(1) += a * dy;
    // Do not zero vel fully — keep Mid motion; lightly damp xy vel mismatch.
    s.vel(0) *= (1.0 - 0.5 * a);
    s.vel(1) *= (1.0 - 0.5 * a);
    kf.change_x(s);
    ++n_odin_xy_factor;
    if ((n_odin_xy_factor % 50) == 1)
        std::cout << "[pv-xy] odin_xy_factor dx=" << (a * dx) << " dy=" << (a * dy)
                  << " err=(" << (tgt_x - (s.pos(0) - a * dx)) << ","
                  << (tgt_y - (s.pos(1) - a * dy)) << ")"
                  << " n=" << n_odin_xy_factor << std::endl;
}

// Bad-frame hold: map Odin Δxyz into fusion frame (same anchors as z_replace).
// Keeps XY/Z self-consistent through stairs when Mid→voxel match is empty.
// Z uses the same flat-hold gate so upper-floor NEP does not import Odin sink.
bool apply_odin_pose_hold_on_bad()
{
    // Hold works with factor or replace; only needs Odin odom + flag.
    if (!odin_pose_hold_on_bad)
        return false;
    double x_o = 0.0, y_o = 0.0, z_o = 0.0, t_o = -1.0;
    {
        std::lock_guard<std::mutex> lk(mtx_odin_odom);
        if (!odin_odom_have)
            return false;
        x_o = odin_odom_x;
        y_o = odin_odom_y;
        z_o = odin_odom_z;
        t_o = odin_odom_stamp;
    }
    state_ikfom s = kf.get_x();
    if (!last_pf_z_valid)
    {
        odin_z_anchor_mid = s.pos(2);
        odin_z_anchor_odin = z_o;
        odin_xy_anchor_mid_x = s.pos(0);
        odin_xy_anchor_mid_y = s.pos(1);
        odin_xy_anchor_odin_x = x_o;
        odin_xy_anchor_odin_y = y_o;
        odin_xy_anchor_valid = true;
        last_pf_z_valid = true;
        last_pf_mid_z = s.pos(2);
        last_pf_odin_z = z_o;
        last_pf_odin_stamp = t_o;
        return false;
    }
    if (!odin_xy_anchor_valid)
    {
        odin_xy_anchor_mid_x = s.pos(0);
        odin_xy_anchor_mid_y = s.pos(1);
        odin_xy_anchor_odin_x = x_o;
        odin_xy_anchor_odin_y = y_o;
        odin_xy_anchor_valid = true;
    }
    s.pos(0) = odin_xy_anchor_mid_x + (x_o - odin_xy_anchor_odin_x);
    s.pos(1) = odin_xy_anchor_mid_y + (y_o - odin_xy_anchor_odin_y);
    double vz = 0.0;
    if (odin_z_is_flat_vz(z_o, t_o, &vz))
    {
        // Keep last good floor height; do not import Odin sink on bad frames.
        const double z_keep = last_pf_mid_z;
        absorb_odin_z_flat(z_o, t_o, z_keep);
        s.pos(2) = z_keep;
    }
    else
    {
        s.pos(2) = odin_z_anchor_mid + (z_o - odin_z_anchor_odin);
        last_pf_mid_z = s.pos(2);
        last_pf_odin_z = z_o;
        last_pf_odin_stamp = t_o;
    }
    s.vel.setZero();
    apply_odin_att_replace(s);
    kf.change_x(s);
    ++n_odin_pose_hold;
    if ((n_odin_pose_hold % 50) == 1)
        std::cout << "[pv-z] odin_pose_hold xyz=(" << s.pos(0) << ","
                  << s.pos(1) << "," << s.pos(2) << ")"
                  << " vz=" << vz
                  << " n=" << n_odin_pose_hold << std::endl;
    return true;
}

// Soft Z: flat_ref + band-limited Odin Δz (do not inherit full Odin drift).
void apply_odin_z_factor(double mid_stamp)
{
    if (!odin_z_factor_en)
        return;
    double t_o = -1.0, z_o = 0.0;
    {
        std::lock_guard<std::mutex> lk(mtx_odin_odom);
        if (!odin_odom_have)
            return;
        t_o = odin_odom_stamp;
        z_o = odin_odom_z;
    }
    if (odin_z_factor_max_dt > 0.0 &&
        std::fabs(t_o - mid_stamp) > odin_z_factor_max_dt)
        return;

    state_ikfom s = kf.get_x();
    if (!last_pf_z_valid)
    {
        last_pf_mid_z = s.pos(2);
        last_pf_odin_z = z_o;
        odin_z_anchor_mid = s.pos(2);
        odin_z_anchor_odin = z_o;
        last_pf_z_valid = true;
        return;
    }

    double odin_dz = z_o - odin_z_anchor_odin;
    if (odin_z_factor_band > 0.0)
    {
        if (odin_dz > odin_z_factor_band)
            odin_dz = odin_z_factor_band;
        else if (odin_dz < -odin_z_factor_band)
            odin_dz = -odin_z_factor_band;
    }
    // Prefer flat world height; allow only ±band of Odin Δz as Mid anti-bias.
    const double base_z =
        residual_z_flat_ref_init ? residual_z_flat_z_ref : odin_z_anchor_mid;
    const double target_z = base_z + odin_dz;
    double raw = target_z - s.pos(2);
    if (odin_z_factor_one_sided && raw < 0.0)
        raw = 0.0;
    const double abs_err = std::fabs(raw);
    if (abs_err < 1e-6)
        return;
    const bool catching =
        odin_z_factor_catch_err > 0.0 && abs_err > odin_z_factor_catch_err;
    const int min_feat = catching ? odin_z_factor_catch_min_feat : 30;
    if (last_mid_effct_feat_num < min_feat)
        return;

    double max_step = odin_z_factor_max_innov;
    double a = odin_z_factor_alpha;
    if (catching)
    {
        max_step = std::max(max_step, odin_z_factor_catch_max);
        a = std::max(a, odin_z_factor_catch_alpha);
    }
    double innov = raw;
    if (max_step > 0.0 && abs_err > max_step)
        innov = std::copysign(max_step, raw);
    a = std::max(0.0, std::min(1.0, a));
    s.pos(2) += a * innov;
    s.vel(2) = 0.0;
    kf.change_x(s);
    ++n_odin_z_factor;
    last_pf_mid_z = s.pos(2);
    last_pf_odin_z = z_o;
    if ((n_odin_z_factor % 50) == 1)
        std::cout << "[pv-z] odin_z_factor innov=" << innov
                  << " tgt=" << target_z << " z=" << s.pos(2)
                  << " raw=" << raw << " odin_dz=" << odin_dz
                  << " n=" << n_odin_z_factor << std::endl;
}

// go2 residual_z_v6.3 post-update:
//  single-level → flat z_ref + vz/ba damp
//  multilevel   → only soft vz/ba (H_z already full in h_share)
void apply_residual_z_flat()
{
    if (!residual_z_en)
        return;
    state_ikfom s = kf.get_x();
    if (!last_floor_valid)
    {
        // Still damp vz/ba lightly so IMU Z coast is limited when no floor.
        const double av = std::max(0.0, std::min(1.0, residual_z_flat_vz_alpha));
        const double ab = std::max(0.0, std::min(1.0, residual_z_flat_ba_z_alpha));
        s.vel(2) *= (1.0 - 0.5 * av);
        s.ba(2) *= (1.0 - 0.5 * ab);
        kf.change_x(s);
        return;
    }
    static double floor_z_ema = 0.0;
    static bool hold_init = false;
    if (!hold_init)
    {
        floor_z_ema = last_floor_z;
        hold_init = true;
        std::cout << "[pv-z] residual_z v6.3 post: multilevel_thr="
                  << residual_z_multilevel_thr << std::endl;
    }
    const double dz_floor = last_floor_z - floor_z_ema;
    floor_z_ema = 0.90 * floor_z_ema + 0.10 * last_floor_z;
    const double tol = std::max(1e-3, residual_z_dz_tol);
    const double move = std::fabs(dz_floor);
    const double stable = std::exp(-0.5 * (move / tol) * (move / tol));
    double w = 0.0;
    if (last_floor_n >= 12 || last_floor_ratio >= residual_z_floor_ratio_thr)
        w = stable * std::min(1.0, std::max(last_floor_ratio / 0.08,
                                            last_floor_n / 80.0));

    if (!residual_z_multilevel)
    {
        if (!residual_z_flat_ref_init)
        {
            residual_z_flat_z_ref = s.pos(2);
            residual_z_flat_ref_init = true;
            std::cout << "[pv-z] residual_z flat_z_ref=" << residual_z_flat_z_ref
                      << " (single-level)" << std::endl;
        }
        const double av = std::max(0.0, std::min(1.0, residual_z_flat_vz_alpha));
        const double az = std::max(0.0, std::min(1.0, residual_z_flat_z_alpha));
        const double ab = std::max(0.0, std::min(1.0, residual_z_flat_ba_z_alpha));
        s.vel(2) *= (1.0 - av);
        s.ba(2) *= (1.0 - ab);
        double step = az * (residual_z_flat_z_ref - s.pos(2));
        if (residual_z_flat_max_step > 0.0 &&
            std::fabs(step) > residual_z_flat_max_step)
            step = std::copysign(residual_z_flat_max_step, step);
        s.pos(2) += step;
    }
    else
    {
        // Multi-level: soft vz/ba only; absolute Z from full floor H_z.
        const double a = std::max(0.0, std::min(0.90, residual_z_vz_alpha));
        s.vel(2) *= (1.0 - w * a);
        s.ba(2) *= (1.0 - 0.35 * w * a);
    }
    kf.change_x(s);
}

// Hard ceiling on |z - flat_ref| after soft pulls (caps peak-to-peak drift).
void apply_residual_z_hard_clamp()
{
    if (!residual_z_en || !residual_z_flat_ref_init || residual_z_hard_band <= 0.0)
        return;
    state_ikfom s = kf.get_x();
    const double dz = s.pos(2) - residual_z_flat_z_ref;
    if (std::fabs(dz) <= residual_z_hard_band)
        return;
    s.pos(2) = residual_z_flat_z_ref + std::copysign(residual_z_hard_band, dz);
    s.vel(2) = 0.0;
    kf.change_x(s);
    ++n_residual_z_hard;
    if ((n_residual_z_hard % 50) == 1)
        std::cout << "[pv-z] hard_clamp dz_was=" << dz
                  << " band=" << residual_z_hard_band
                  << " z=" << s.pos(2)
                  << " n=" << n_residual_z_hard << std::endl;
}

const bool var_contrast(pointWithCov &x, pointWithCov &y) {
    return (x.cov.diagonal().norm() < y.cov.diagonal().norm());
};

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;

    po->curvature = pi->curvature;
    po->normal_x = pi->normal_x;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    auto time_offset = lidar_time_offset;
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    if (rclcpp::Time(msg->header.stamp).seconds() + time_offset < last_timestamp_lidar)
    {
        RCLCPP_ERROR(rclcpp::get_logger("pv_lio"), "lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(rclcpp::Time(msg->header.stamp).seconds() + time_offset);
    last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds() + time_offset;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void standard_pcl_cbk2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    mtx_buffer.lock();
    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    if (stamp < last_timestamp_lidar2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("pv_lio"), "lidar2 loop back, clear buffer");
        lidar_buffer2.clear(); time_buffer2.clear();
    }
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre2->process(msg, ptr);
    // Empty after confidence/range filter must not enter the pair buffer —
    // it would waste sync slots and force Mid-only (Odin flicker).
    if (ptr->empty())
    {
        mtx_buffer.unlock();
        return;
    }
    std::sort(ptr->points.begin(), ptr->points.end(), time_list);
    lidar_buffer2.push_back(ptr);
    time_buffer2.push_back(stamp);
    last_timestamp_lidar2 = stamp;
    while (lidar_buffer2.size() > 50)
    {
        lidar_buffer2.pop_front();
        time_buffer2.pop_front();
    }
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar)
    {
        RCLCPP_ERROR(rclcpp::get_logger("pv_lio"), "lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::SharedPtr msg_in)
{
    publish_count ++;
    auto msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);

    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = sec_to_stamp(timediff_lidar_wrt_imu + rclcpp::Time(msg_in->header.stamp).seconds());
    }

    double timestamp = rclcpp::Time(msg->header.stamp).seconds();

    if (timestamp < last_timestamp_imu)
    {
        RCLCPP_WARN(rclcpp::get_logger("pv_lio"), "imu loop back, ignoring!!!");
        RCLCPP_WARN(rclcpp::get_logger("pv_lio"), "current T: %f, last T: %f", timestamp, last_timestamp_imu);
        return;
    }
    // reject abnormal IMU data
    if (std::abs(msg->angular_velocity.x) > 10
        || std::abs(msg->angular_velocity.y) > 10
        || std::abs(msg->angular_velocity.z) > 10) {
        RCLCPP_WARN(rclcpp::get_logger("pv_lio"), "Large IMU measurement!!! Drop Data!!! %.3f  %.3f  %.3f",
                 msg->angular_velocity.x,
                 msg->angular_velocity.y,
                 msg->angular_velocity.z
        );
        return;
    }

    last_timestamp_imu = timestamp;

    mtx_buffer.lock();

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu2_cbk(const sensor_msgs::msg::Imu::SharedPtr msg_in)
{
    auto msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);
    const double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    std::lock_guard<std::mutex> lk(mtx_buffer);
    if (timestamp < last_timestamp_imu2)
    {
        RCLCPP_WARN(rclcpp::get_logger("pv_lio"), "imu2 loop back, clear");
        imu_buffer2.clear();
    }
    last_timestamp_imu2 = timestamp;
    imu_buffer2.push_back(msg);
    while (imu_buffer2.size() > 4000) imu_buffer2.pop_front();
    sig_buffer.notify_all();
}

static double cloud_offset_median_sec(const PointCloudXYZI::Ptr &cloud)
{
    if (!cloud || cloud->empty()) return 0.0;
    std::vector<double> t; t.reserve(cloud->size());
    for (const auto &p: cloud->points) t.push_back(p.curvature/1000.0);
    auto mid=t.begin()+t.size()/2; std::nth_element(t.begin(),mid,t.end());
    return *mid;
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    std::lock_guard<std::mutex> lk(mtx_buffer);
    if (lidar_buffer.empty() || imu_buffer.empty()) return false;
    // If Mid queue lags, Odin buffer (~50 frames/~10s) is drained by the
    // beg-time prune → perpetual unpaired and Odin vanishes after hold.
    // Drop stale Mid (+IMU) so pairing stays near realtime. (Polka/aggregator
    // pattern: drop stale sources; never paint old geometry at new pose.)
    {
        const double max_mid_lag_sec = 0.50;
        int dropped = 0;
        while (lidar_buffer.size() > 1 &&
               (last_timestamp_lidar - time_buffer.front()) > max_mid_lag_sec)
        {
            const double drop_end = time_buffer.front() + 0.15;
            lidar_buffer.pop_front();
            time_buffer.pop_front();
            while (!imu_buffer.empty() &&
                   rclcpp::Time(imu_buffer.front()->header.stamp).seconds() < drop_end)
                imu_buffer.pop_front();
            lidar_pushed = false;
            ++dropped;
        }
        if (dropped > 0)
        {
            static int lag_drop_log = 0;
            if ((++lag_drop_log % 20) == 1)
                std::cout << "[pv-dual] drop_stale_mid n=" << dropped
                          << " mid_buf=" << lidar_buffer.size()
                          << " odin_buf=" << lidar_buffer2.size()
                          << " lag_ms="
                          << (last_timestamp_lidar - time_buffer.front()) * 1000.0
                          << std::endl;
        }
    }
    static bool odin_selected = false;
    static size_t odin_selected_i = 0;
    const double wall_sec = omp_get_wtime();

    auto fill_odin = [&](size_t i, double score, double signed_dt, double end_dt) {
        auto cloud = lidar_buffer2[i];
        meas.lidar2 = cloud;
        meas.lidar_beg_time2 = time_buffer2[i];
        double span2 = 0.0;
        for (const auto &pt : cloud->points)
            span2 = std::max(span2, double(pt.curvature) / 1000.0);
        lidar_end_time2 = meas.lidar_beg_time2 + span2;
        meas.lidar_end_time2 = lidar_end_time2;
        meas.lidar2_fresh = true;
        odin_selected = true;
        odin_selected_i = i;
        bundle_pair_score = score;
        bundle_center_dt = signed_dt;
        bundle_end_dt = end_dt;
        // Do NOT adapt bias from end_dt. end residual mixes clock offset with
        // Mid/Odin span difference (~6ms) and phase; EMA on end_dt walked bias
        // from 0→100ms+ in real runs and reintroduced the smear skew.
        // Clock offset was measured ~0 on this machine; keep configured bias.
        (void)lidar2_bias_ema;
    };
    auto fill_mid_only = [&]() {
        meas.lidar2.reset(new PointCloudXYZI());
        meas.imu2.clear();
        meas.lidar2_fresh = false;
        odin_selected = false;
        odin_selected_i = 0;
        bundle_pair_score = -1.0;
        bundle_center_dt = 0.0;
        bundle_end_dt = 0.0;
        meas.lidar_beg_time2 = meas.lidar_beg_time;
        lidar_end_time2 = lidar_end_time;
        meas.lidar_end_time2 = lidar_end_time2;
    };
    // Pair by END time (same quantity deskew targets), not center+median.
    // Root empty-frame cause: center metric + 30ms soft-wait dropped Odin when
    // phase drifted or Odin was slightly "too new", committing Mid-only → flicker.
    auto try_attach_odin = [&](bool &found, size_t &best_i, double &best_score,
                               double &signed_dt, double &end_dt) {
        found = false;
        best_i = 0;
        best_score = 1e9;
        signed_dt = 0.0;
        end_dt = 0.0;
        if (!multi_lidar_en) return;
        while (time_buffer2.size() > 1 &&
               time_buffer2.front() + lidar2_stamp_bias_sec <
                   meas.lidar_beg_time - lidar2_max_sync_dt)
        {
            lidar_buffer2.pop_front();
            time_buffer2.pop_front();
        }
        for (size_t i = 0; i < time_buffer2.size(); ++i)
        {
            if (!lidar_buffer2[i] || lidar_buffer2[i]->empty()) continue;
            double span2 = 0.0;
            for (const auto &pt : lidar_buffer2[i]->points)
                span2 = std::max(span2, double(pt.curvature) / 1000.0);
            if (span2 < 1e-4)
                span2 = cloud_offset_median_sec(lidar_buffer2[i]) * 2.0;
            const double odin_end_on_mid =
                time_buffer2[i] + lidar2_stamp_bias_sec + span2;
            const double score = std::fabs(odin_end_on_mid - lidar_end_time);
            if (score < best_score && score <= lidar2_max_sync_dt)
            {
                best_score = score;
                best_i = i;
                end_dt = lidar_end_time - odin_end_on_mid;
                const double med = cloud_offset_median_sec(lidar_buffer2[i]);
                const double mid_span =
                    std::max(0.0, lidar_end_time - meas.lidar_beg_time);
                const double mid_center = meas.lidar_beg_time + 0.5 * mid_span;
                signed_dt = mid_center -
                            (time_buffer2[i] + lidar2_stamp_bias_sec + med);
                found = true;
            }
        }
    };

    if (!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        fill_mid_only();
        if (meas.lidar->points.size() <= 1)
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        else if (meas.lidar->points.back().curvature / 1000.0 < 0.5 * lidar_mean_scantime)
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        else
        {
            scan_num++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / 1000.0;
            lidar_mean_scantime +=
                (meas.lidar->points.back().curvature / 1000.0 - lidar_mean_scantime) / scan_num;
        }
        meas.lidar_end_time = lidar_end_time;

        bool found = false;
        size_t best_i = 0;
        double best_score = 1e9, signed_dt = 0.0, end_dt = 0.0;
        try_attach_odin(found, best_i, best_score, signed_dt, end_dt);
        if (found)
            fill_odin(best_i, best_score, signed_dt, end_dt);
        bundle_mid_push_wall = wall_sec;
        lidar_pushed = true;
    }

    // Soft-wait: ALWAYS hold Mid until Odin pairs or timeout.
    // Previous logic skipped waiting when buffer had a too-new Odin → Mid-only
    // flicker even though a pairable Odin was about to arrive / already present
    // just outside the old 55–80ms window.
    if (multi_lidar_en && !meas.lidar2_fresh)
    {
        bool found = false;
        size_t best_i = 0;
        double best_score = 1e9, signed_dt = 0.0, end_dt = 0.0;
        try_attach_odin(found, best_i, best_score, signed_dt, end_dt);
        if (found)
        {
            fill_odin(best_i, best_score, signed_dt, end_dt);
        }
        else
        {
            // Always soft-wait the full window. Aborting on Mid backlog (old
            // size>2 check) forced Mid-only while a pairable Odin was still
            // in flight — measured as clustered skip=unpaired bursts.
            if (lidar2_soft_wait_sec > 0.0 &&
                (wall_sec - bundle_mid_push_wall) < lidar2_soft_wait_sec)
                return false; // keep Mid unconsumed
            // Last chance: accept a slightly late/early Odin rather than Mid-only
            // flicker (realtime body centroid jumped ~1.4m on composition flips).
            double best_any = 1e9;
            size_t best_any_i = 0;
            double signed_any = 0.0, end_any = 0.0;
            for (size_t i = 0; i < time_buffer2.size(); ++i)
            {
                if (!lidar_buffer2[i] || lidar_buffer2[i]->empty()) continue;
                double span2 = 0.0;
                for (const auto &pt : lidar_buffer2[i]->points)
                    span2 = std::max(span2, double(pt.curvature) / 1000.0);
                if (span2 < 1e-4)
                    span2 = cloud_offset_median_sec(lidar_buffer2[i]) * 2.0;
                const double odin_end_on_mid =
                    time_buffer2[i] + lidar2_stamp_bias_sec + span2;
                const double score = std::fabs(odin_end_on_mid - lidar_end_time);
                if (score < best_any)
                {
                    best_any = score;
                    best_any_i = i;
                    end_any = lidar_end_time - odin_end_on_mid;
                    const double med = cloud_offset_median_sec(lidar_buffer2[i]);
                    const double mid_span =
                        std::max(0.0, lidar_end_time - meas.lidar_beg_time);
                    signed_any = meas.lidar_beg_time + 0.5 * mid_span -
                                 (time_buffer2[i] + lidar2_stamp_bias_sec + med);
                }
            }
            if (best_any <= std::max(lidar2_max_sync_dt * 1.5, 0.30))
                fill_odin(best_any_i, best_any, signed_any, end_any);
            else
                fill_mid_only();
        }
    }

    if (last_timestamp_imu < lidar_end_time) return false;
    if (meas.lidar2_fresh)
    {
        // Need imu2 through Mid-end on Odin clock so native deskew can reach
        // the same end pose Mid uses (not only Odin's own scan end).
        const double imu2_horizon = std::max(
            meas.lidar_end_time2,
            meas.lidar_end_time - lidar2_stamp_bias_sec);
        if (last_timestamp_imu2 > 0.0 && last_timestamp_imu2 < imu2_horizon)
            return false;
        while (imu_buffer2.size() > 2 &&
               rclcpp::Time(imu_buffer2.front()->header.stamp).seconds() <
                   meas.lidar_beg_time2 - 0.15)
            imu_buffer2.pop_front();
        meas.imu2.clear();
        for (const auto &im : imu_buffer2)
        {
            const double t = rclcpp::Time(im->header.stamp).seconds();
            if (t < meas.lidar_beg_time2 - 0.02) continue;
            if (t > imu2_horizon + 1e-4) break;
            meas.imu2.push_back(im);
        }
        // Drop only samples well before this Odin scan; keep overlap for next.
        while (!imu_buffer2.empty() &&
               rclcpp::Time(imu_buffer2.front()->header.stamp).seconds() <
                   meas.lidar_beg_time2 - 0.02)
            imu_buffer2.pop_front();
    }

    double imu_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();
    meas.imu.clear();
    while (!imu_buffer.empty() && imu_time < lidar_end_time)
    {
        imu_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();
        if (imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    if (odin_selected)
    {
        for (size_t i = 0; i <= odin_selected_i && !lidar_buffer2.empty(); ++i)
        {
            lidar_buffer2.pop_front();
            time_buffer2.pop_front();
        }
    }
    odin_selected = false;
    lidar_pushed = false;
    return true;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull,
                         const PointCloudXYZI::Ptr &cloud_override = PointCloudXYZI::Ptr())
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes =
            cloud_override ? cloud_override : (dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI laserCloudWorld;
        for (int i = 0; i < size; i++)
        {
            PointType const * const p = &laserCloudFullRes->points[i];
            if(p->intensity < 5){
                continue;
            }
            PointType p_world;

            RGBpointBodyToWorld(p, &p_world);
            laserCloudWorld.push_back(p_world);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = sec_to_stamp(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

}

void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull_body,
                        const PointCloudXYZI::Ptr &cloud_override = PointCloudXYZI::Ptr())
{
    PointCloudXYZI::Ptr laserCloudFullRes =
        cloud_override ? cloud_override : (dense_pub_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&laserCloudFullRes->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = sec_to_stamp(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudMap)
{
    sensor_msgs::msg::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = sec_to_stamp(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap->publish(laserCloudMap);
}

void publish_global_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubGlobalCloudMap)
{
    sensor_msgs::msg::PointCloud2 laserCloudMap;
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloudMap
                             (new pcl::PointCloud<pcl::PointXYZ>);
    for(const auto& ceil_map : voxel_map) {
        pcl::PointXYZ pt;
        for(size_t i = 0; i < ceil_map.second->temp_points_.size(); i++) {
            const auto& temp_point = ceil_map.second->temp_points_[i];
            pt.x = temp_point.point_world.x();
            pt.y = temp_point.point_world.y();
            pt.z = temp_point.point_world.z();
            globalCloudMap->push_back(pt);
        }
    }

}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;

}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pubOdomAftMapped,
                      std::shared_ptr<tf2_ros::TransformBroadcaster> & tf_br,
                      const rclcpp::Node::SharedPtr & node)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = sec_to_stamp(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped->publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    // TF: camera_init -> body
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = odomAftMapped.header.stamp;
    t.header.frame_id = "camera_init";
    t.child_frame_id = "body";
    t.transform.translation.x = odomAftMapped.pose.pose.position.x;
    t.transform.translation.y = odomAftMapped.pose.pose.position.y;
    t.transform.translation.z = odomAftMapped.pose.pose.position.z;
    t.transform.rotation = odomAftMapped.pose.pose.orientation;
    tf_br->sendTransform(t);

    // TF: world -> camera_init
    geometry_msgs::msg::TransformStamped t_world;
    t_world.header.stamp = odomAftMapped.header.stamp;
    t_world.header.frame_id = "world";
    t_world.child_frame_id = "camera_init";
    t_world.transform.translation.x = 0;
    t_world.transform.translation.y = 0;
    t_world.transform.translation.z = 0;
    t_world.transform.rotation.x = p_imu->Initial_R_wrt_G.x();
    t_world.transform.rotation.y = p_imu->Initial_R_wrt_G.y();
    t_world.transform.rotation.z = p_imu->Initial_R_wrt_G.z();
    t_world.transform.rotation.w = p_imu->Initial_R_wrt_G.w();
    tf_br->sendTransform(t_world);
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = sec_to_stamp(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 1 == 0)
    {
        path.header.stamp = msg_body_pose.header.stamp;
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

void publish_pose(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &pubpose)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = sec_to_stamp(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";
    pubpose->publish(msg_body_pose);

}



void transformLidar(const state_ikfom &state_point, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
    trans_cloud->clear();
    for (size_t i = 0; i < input_cloud->size(); i++) {
        pcl::PointXYZINormal p_c = input_cloud->points[i];
        Eigen::Vector3d p_lidar(p_c.x, p_c.y, p_c.z);
        V3D p_body = state_point.rot * (state_point.offset_R_L_I * p_lidar + state_point.offset_T_L_I) + state_point.pos;
        PointType pi;
        pi.x = p_body(0);
        pi.y = p_body(1);
        pi.z = p_body(2);
        pi.intensity = p_c.intensity;
        trans_cloud->points.push_back(pi);
    }
}

M3D transformLiDARCovToWorld(Eigen::Vector3d &p_lidar, const esekfom::esekf<state_ikfom, 12, input_ikfom>& kf, const Eigen::Matrix3d& COV_lidar)
{
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(p_lidar);
    auto state = kf.get_x();

    M3D il_rot_var = kf.get_P().block<3, 3>(6, 6);
    M3D il_t_var = kf.get_P().block<3, 3>(9, 9);

    M3D COV_body =
            state.offset_R_L_I * COV_lidar * state.offset_R_L_I.conjugate()
            + state.offset_R_L_I * (-point_crossmat) * il_rot_var * (-point_crossmat).transpose() * state.offset_R_L_I.conjugate()
            + il_t_var;

    V3D p_body = state.offset_R_L_I * p_lidar + state.offset_T_L_I;

    point_crossmat << SKEW_SYM_MATRX(p_body);
    M3D rot_var = kf.get_P().block<3, 3>(3, 3);
    M3D t_var = kf.get_P().block<3, 3>(0, 0);

    M3D COV_world =
        state.rot * COV_body * state.rot.conjugate()
        + state.rot * (-point_crossmat) * rot_var * (-point_crossmat).transpose()  * state.rot.conjugate()
        + t_var;

    return COV_world;
}

void observation_model_share(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    feats_with_correspondence->clear();
    total_residual = 0.0;

    vector<pointWithCov> pv_list;
    PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI);
    transformLidar(s, feats_down_body, world_lidar);
    pv_list.resize(feats_down_body->size());
    for (size_t i = 0; i < feats_down_body->size(); i++) {
        pointWithCov pv;
        pv.point << feats_down_body->points[i].x, feats_down_body->points[i].y, feats_down_body->points[i].z;
        pv.point_world << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
        M3D cov_lidar = var_down_body[i];
        M3D cov_world = transformLiDARCovToWorld(pv.point, kf, cov_lidar);
        pv.cov = cov_world;
        pv.cov_lidar = cov_lidar;
        // Odin body points are tagged intensity=200 in commit_odin_body.
        pv.lidar_src = (feats_down_body->points[i].intensity >= 150.0f) ? 1 : 0;
        pv_list[i] = pv;
    }

    double match_start = omp_get_wtime();
    std::vector<ptpl> ptpl_list;
    std::vector<V3D> non_match_list;
    BuildResidualListOMP(voxel_map, max_voxel_size, 3.0, max_layer, pv_list,
                         ptpl_list, non_match_list);
    double match_end = omp_get_wtime();

    effct_feat_num = ptpl_list.size();
    {
        int mid_eff = 0;
        for (const auto &p : ptpl_list)
            if (p.lidar_src == 0) ++mid_eff;
        // Bundle/joint: Mid match count (not total, which includes soft Odin).
        last_mid_effct_feat_num = mid_eff > 0 ? mid_eff : (!iekf_odin_stage ? effct_feat_num : 0);
    }
    if (effct_feat_num < 1){
        ekfom_data.valid = false;
        RCLCPP_WARN(rclcpp::get_logger("pv_lio"), "No Effective Points!");
        return;
    }
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);
    ekfom_data.h.resize(effct_feat_num);
    ekfom_data.R.resize(effct_feat_num, 1);

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < effct_feat_num; i++)
    {
        V3D point_this_be(ptpl_list[i].point);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        V3D norm_vec(ptpl_list[i].normal);
        const bool is_odin_pt = (ptpl_list[i].lidar_src == 1);
        // Mid floors: H_z damp applied after residual build (go2 multilevel).
        const bool is_floor =
            residual_z_en && !is_odin_pt &&
            std::fabs(norm_vec.z()) > residual_z_floor_nz_thr;
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_vec.x(), norm_vec.y(), norm_vec.z(), VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_vec.x(), norm_vec.y(), norm_vec.z(), VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        float pd2 = norm_vec.x() * ptpl_list[i].point_world.x()
                + norm_vec.y() * ptpl_list[i].point_world.y()
                + norm_vec.z() * ptpl_list[i].point_world.z()
                + ptpl_list[i].d;

        // Joint S2: clip large residuals on Odin rows only (never Mid).
        if (iekf_odin_stage && is_odin_pt && std::fabs(pd2) > odin_max_residual)
        {
            ekfom_data.h(i) = 0.0;
            ekfom_data.h_x.row(i).setZero();
            ekfom_data.R(i) = 1e-12;  // R is information weight
#ifdef MP_EN
#pragma omp atomic
#endif
            iekf_odin_res_clip++;
            continue;
        }

        ekfom_data.h(i) = -pd2;
        total_residual += std::fabs(pd2);

        V3D point_world = ptpl_list[i].point_world;
        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq.block<1, 3>(0, 0) = point_world - ptpl_list[i].center;
        J_nq.block<1, 3>(0, 3) = -ptpl_list[i].normal;
        double sigma_l = J_nq * ptpl_list[i].plane_cov * J_nq.transpose();

        M3D cov_lidar = ptpl_list[i].cov_lidar;
        M3D R_cov_Rt = s.rot * s.offset_R_L_I * cov_lidar * s.offset_R_L_I.conjugate() * s.rot.conjugate();
        double R_inv = 1.0 / (sigma_l + norm_vec.transpose() * R_cov_Rt * norm_vec);

        // Always mildly soften Mid floors (do not fully trust biased Z planes).
        if (is_floor)
        {
            const double sc =
                std::max(0.05, std::min(1.0, residual_z_floor_info_scale));
            R_inv *= sc;
        }
        // Joint S2: boost Odin near-vertical walls; keep Odin floors soft so
        // wall normals carry height/lateral info without floor Z bite.
        if (iekf_odin_stage && is_odin_pt)
        {
            const double nz = std::fabs(norm_vec.z());
            if (nz < odin_wall_nz_thr)
                R_inv *= std::max(1.0, odin_wall_info_boost);
            else if (nz > residual_z_floor_nz_thr)
                R_inv *= std::max(0.05, std::min(1.0, odin_floor_info_scale));
        }

        ekfom_data.R(i) = R_inv;
    }

    // go2 residual_z_v6: floor plane under robot → multilevel H_z policy.
    last_floor_valid = false;
    last_floor_n = 0;
    last_floor_ratio = 0.0;
    if (residual_z_en && effct_feat_num > 0)
    {
        double z_sum = 0.0;
        int n_floor = 0;
        for (int i = 0; i < effct_feat_num; ++i)
        {
            if (ptpl_list[i].lidar_src == 1) continue;
            const double nz = ptpl_list[i].normal.z();
            if (std::fabs(nz) <= residual_z_floor_nz_thr) continue;
            const double z_map =
                -(ptpl_list[i].d + ptpl_list[i].normal.x() * s.pos(0) +
                  ptpl_list[i].normal.y() * s.pos(1)) /
                nz;
            if (z_map > s.pos(2) - 0.15) continue;
            if (z_map < s.pos(2) - 2.5) continue;
            z_sum += z_map;
            ++n_floor;
        }
        last_floor_n = n_floor;
        last_floor_ratio =
            static_cast<double>(n_floor) / static_cast<double>(effct_feat_num);
        last_floor_valid = (n_floor >= 12);
        if (last_floor_valid)
            last_floor_z = z_sum / static_cast<double>(n_floor);

        if (last_floor_valid && ekfom_data.h_x.cols() > 2)
        {
            static double floor_z_ema_h = 0.0;
            static bool floor_z_ema_h_init = false;
            static double floor_z_min_seen_h = 0.0;
            static double floor_z_max_seen_h = 0.0;
            static bool floor_span_init_h = false;
            if (!floor_z_ema_h_init)
            {
                floor_z_ema_h = last_floor_z;
                floor_z_ema_h_init = true;
            }
            if (!floor_span_init_h)
            {
                floor_z_min_seen_h = last_floor_z;
                floor_z_max_seen_h = last_floor_z;
                floor_span_init_h = true;
            }
            const double dz_floor = last_floor_z - floor_z_ema_h;
            floor_z_ema_h = 0.90 * floor_z_ema_h + 0.10 * last_floor_z;
            floor_z_min_seen_h = std::min(floor_z_min_seen_h, last_floor_z);
            floor_z_max_seen_h = std::max(floor_z_max_seen_h, last_floor_z);
            if ((floor_z_max_seen_h - floor_z_min_seen_h) >=
                residual_z_multilevel_thr)
            {
                if (!residual_z_multilevel)
                {
                    residual_z_multilevel = true;
                    std::cout << "[pv-z] multi-level floor span="
                              << (floor_z_max_seen_h - floor_z_min_seen_h)
                              << " → full H_z, flat z_ref OFF" << std::endl;
                }
            }
            const double tol = std::max(1e-3, residual_z_dz_tol);
            const double move = std::fabs(dz_floor);
            const double stable =
                std::exp(-0.5 * (move / tol) * (move / tol));
            // After stairs: full H_z so floor re-locks absolute Z (stops sink).
            const double zmin = residual_z_multilevel
                                    ? 1.0
                                    : std::max(0.0, std::min(1.0, residual_z_h_dampen));
            const double zdamp = zmin + (1.0 - zmin) * (1.0 - stable);
            if (zdamp < 0.999)
                ekfom_data.h_x.col(2) *= zdamp;
        }
    }

    res_mean_last = (effct_feat_num > 0) ? (total_residual / effct_feat_num) : 0.0;
}

bool saveMapToFile(const std::string &dir) {
    mkdir(dir.c_str(), 0775);
    time_t now = time(nullptr);
    struct tm *t = localtime(&now);
    char filename[256];
    snprintf(filename, sizeof(filename), "%s/map_%04d%02d%02d_%02d%02d%02d.pcd",
             dir.c_str(), t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
             t->tm_hour, t->tm_min, t->tm_sec);

    PointCloudXYZI::Ptr map_copy(new PointCloudXYZI());
    {
        std::lock_guard<std::mutex> lck(global_cloud_map_mtx);
        *map_copy = *global_cloud_map;
    }

    if (map_copy->empty()) {
        RCLCPP_WARN(rclcpp::get_logger("pv_lio"), "Map is empty, nothing to save.");
        return false;
    }

    int ret = pcl::io::savePCDFileBinary(std::string(filename), *map_copy);
    if (ret == 0) {
        RCLCPP_INFO(rclcpp::get_logger("pv_lio"), "Map saved to %s (%zu points)", filename, map_copy->size());
        return true;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("pv_lio"), "Failed to save map to %s", filename);
        return false;
    }
}

// Async save (FAST_LIO-like): snapshot under lock, write PCD off the mapping thread
// so SingleThreadedExecutor is not blocked by ~90MB disk IO → IMU backlog/divergence.
void saveMapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    has_save_opt = true;
    if (save_map_busy.exchange(true)) {
        res->success = false;
        res->message = "Save already in progress.";
        return;
    }

    mkdir(pcd_save_path.c_str(), 0775);
    time_t now = time(nullptr);
    struct tm *t = localtime(&now);
    char filename[256];
    snprintf(filename, sizeof(filename), "%s/map_%04d%02d%02d_%02d%02d%02d.pcd",
             pcd_save_path.c_str(), t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
             t->tm_hour, t->tm_min, t->tm_sec);
    const std::string path(filename);

    PointCloudXYZI::Ptr map_copy(new PointCloudXYZI());
    {
        std::lock_guard<std::mutex> lck(global_cloud_map_mtx);
        *map_copy = *global_cloud_map;
    }
    if (map_copy->empty()) {
        save_map_busy.store(false);
        res->success = false;
        res->message = "Map is empty, nothing to save.";
        return;
    }

    std::thread([map_copy, path]() {
        const int ret = pcl::io::savePCDFileBinary(path, *map_copy);
        if (ret == 0) {
            RCLCPP_INFO(rclcpp::get_logger("pv_lio"),
                        "Map saved (async) to %s (%zu points)", path.c_str(), map_copy->size());
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("pv_lio"),
                         "Async map save failed: %s", path.c_str());
        }
        save_map_busy.store(false);
    }).detach();

    res->success = true;
    res->message = std::string("Async map save started: ") + path;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("laserMapping");

    node->declare_parameter<double>("time_offset", 0.0);
    node->get_parameter("time_offset", lidar_time_offset);

    node->declare_parameter<bool>("publish.path_en", true);
    node->declare_parameter<bool>("publish.scan_publish_en", true);
    node->declare_parameter<bool>("publish.dense_publish_en", true);
    node->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
    node->get_parameter("publish.path_en", path_en);
    node->get_parameter("publish.scan_publish_en", scan_pub_en);
    node->get_parameter("publish.dense_publish_en", dense_pub_en);
    node->get_parameter("publish.scan_bodyframe_pub_en", scan_body_pub_en);

    node->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
    node->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
    node->declare_parameter<bool>("common.time_sync_en", false);
    node->get_parameter("common.lid_topic", lid_topic);
    node->get_parameter("common.imu_topic", imu_topic);
    node->get_parameter("common.time_sync_en", time_sync_en);
    node->declare_parameter<bool>("common.multi_lidar_en", false);
    node->declare_parameter<std::string>("common.lid_topic2", "/odin1/cloud_raw");
    node->declare_parameter<std::string>("common.imu_topic2", "/odin1/imu");
    node->declare_parameter<double>("common.lidar2_stamp_bias_sec", 0.0);
    node->declare_parameter<double>("common.lidar2_max_sync_dt", 0.20);
    node->declare_parameter<double>("common.lidar2_soft_wait_sec", 0.18);
    node->declare_parameter<double>("common.lidar2_bias_ema", 0.05);
    node->get_parameter("common.multi_lidar_en", multi_lidar_en);
    node->get_parameter("common.lid_topic2", lid_topic2);
    node->get_parameter("common.imu_topic2", imu_topic2);
    node->get_parameter("common.lidar2_stamp_bias_sec", lidar2_stamp_bias_sec);
    node->get_parameter("common.lidar2_max_sync_dt", lidar2_max_sync_dt);
    node->get_parameter("common.lidar2_soft_wait_sec", lidar2_soft_wait_sec);
    node->get_parameter("common.lidar2_bias_ema", lidar2_bias_ema);

    // mapping algorithm params
    node->declare_parameter<double>("mapping.det_range", 300.0);
    node->declare_parameter<int>("mapping.max_iteration", 4);
    node->declare_parameter<int>("mapping.max_points_size", 100);
    node->declare_parameter<int>("mapping.max_cov_points_size", 100);
    node->declare_parameter<std::vector<double>>("mapping.layer_point_size", std::vector<double>());
    node->declare_parameter<int>("mapping.max_layer", 2);
    node->declare_parameter<double>("mapping.voxel_size", 1.0);
    node->declare_parameter<double>("mapping.down_sample_size", 0.5);
    node->declare_parameter<double>("mapping.plannar_threshold", 0.01);
    node->declare_parameter<bool>("mapping.extrinsic_est_en", true);
    node->declare_parameter<std::vector<double>>("mapping.extrinsic_T", std::vector<double>());
    node->declare_parameter<std::vector<double>>("mapping.extrinsic_R", std::vector<double>());

    double det_range_d;
    node->get_parameter("mapping.det_range", det_range_d);
    DET_RANGE = static_cast<float>(det_range_d);
    node->get_parameter("mapping.max_iteration", NUM_MAX_ITERATIONS);
    node->get_parameter("mapping.max_points_size", max_points_size);
    node->get_parameter("mapping.max_cov_points_size", max_cov_points_size);
    node->get_parameter("mapping.layer_point_size", layer_point_size);
    node->get_parameter("mapping.max_layer", max_layer);
    node->get_parameter("mapping.voxel_size", max_voxel_size);
    node->get_parameter("mapping.down_sample_size", filter_size_surf_min);
    std::cout << "filter_size_surf_min:" << filter_size_surf_min << std::endl;
    node->get_parameter("mapping.plannar_threshold", min_eigen_value);
    node->get_parameter("mapping.extrinsic_est_en", extrinsic_est_en);
    node->get_parameter("mapping.extrinsic_T", extrinT);
    node->get_parameter("mapping.extrinsic_R", extrinR);
    node->declare_parameter<std::vector<double>>("mapping.extrinsic_T_L2_wrt_L1", {0.1324518,-0.0103959,0.1587});
    node->declare_parameter<std::vector<double>>("mapping.extrinsic_R_L2_wrt_L1", {0.9743698,0.0007368,-0.2249510,-0.0007179,0.9999997,0.0001657,0.22495105,0.0,0.97437006});
    node->declare_parameter<std::vector<double>>("mapping.extrinsic_T_L2_wrt_odin_imu", {-0.02663,0.03447,0.02174});
    node->declare_parameter<std::vector<double>>("mapping.extrinsic_R_L2_wrt_odin_imu", {1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0});
    node->declare_parameter<double>("mapping.lidar2_cov_scale", 4.0);
    std::vector<double> extrinT_l2, extrinR_l2, extrinT_oi, extrinR_oi;
    node->get_parameter("mapping.extrinsic_T_L2_wrt_L1", extrinT_l2);
    node->get_parameter("mapping.extrinsic_R_L2_wrt_L1", extrinR_l2);
    node->get_parameter("mapping.extrinsic_T_L2_wrt_odin_imu", extrinT_oi);
    node->get_parameter("mapping.extrinsic_R_L2_wrt_odin_imu", extrinR_oi);
    node->get_parameter("mapping.lidar2_cov_scale", lidar2_cov_scale);

    // dual_tune.* from config/dual_mid_odin_tune.py (edit py → restart, no rebuild)
    node->declare_parameter<int>("dual_tune.odin_min_pts", odin_min_pts);
    node->declare_parameter<double>("dual_tune.odin_min_frac", odin_min_frac);
    node->declare_parameter<double>("dual_tune.odin_body_z_min", odin_body_z_min);
    node->declare_parameter<double>("dual_tune.odin_body_z_max", odin_body_z_max);
    node->declare_parameter<double>("dual_tune.odin_static_gyro", odin_static_gyro);
    node->declare_parameter<bool>("dual_tune.odin_s2_enable", odin_s2_enable);
    node->declare_parameter<int>("dual_tune.odin_iekf_max_pts", odin_iekf_max_pts);
    node->declare_parameter<double>("dual_tune.odin_iekf_max_end_dt", odin_iekf_max_end_dt);
    node->declare_parameter<double>("dual_tune.odin_info_weight", odin_info_weight);
    node->declare_parameter<double>("dual_tune.odin_range_ref", odin_range_ref);
    node->declare_parameter<double>("dual_tune.odin_max_residual", odin_max_residual);
    node->declare_parameter<double>("dual_tune.odin_s2_gyr_gate", odin_s2_gyr_gate);
    node->declare_parameter<double>("dual_tune.iekf_max_upd", iekf_max_upd);
    node->declare_parameter<double>("dual_tune.iekf_max_rot", iekf_max_rot);
    node->declare_parameter<double>("dual_tune.iekf_max_vel", iekf_max_vel);
    node->declare_parameter<double>("dual_tune.iekf_s2_max_upd", iekf_s2_max_upd);
    node->declare_parameter<double>("dual_tune.iekf_s2_max_rot", iekf_s2_max_rot);
    node->declare_parameter<int>("dual_tune.consec_bad_freeze", consec_bad_freeze);
    node->declare_parameter<double>("dual_tune.map_jump_upd", map_jump_upd);
    node->declare_parameter<double>("dual_tune.map_jump_frame", map_jump_frame);
    node->declare_parameter<double>("dual_tune.map_max_range", map_max_range);
    node->declare_parameter<bool>("dual_tune.map_require_still", map_require_still);
    node->declare_parameter<double>("dual_tune.map_still_gyr", map_still_gyr);
    node->declare_parameter<bool>("dual_tune.map_use_downsample", map_use_downsample);
    node->declare_parameter<bool>("dual_tune.odin_global_insert_en", odin_global_insert_en);
    node->declare_parameter<bool>("dual_tune.odin_global_require_s2", odin_global_require_s2);
    node->declare_parameter<bool>("dual_tune.odin_global_still_densify_en", odin_global_still_densify_en);
    node->declare_parameter<double>("dual_tune.odin_global_still_gyr", odin_global_still_gyr);
    node->declare_parameter<bool>("dual_tune.odin_global_walk_floor_densify_en",
                                  odin_global_walk_floor_densify_en);
    node->declare_parameter<double>("dual_tune.odin_global_walk_floor_z_min",
                                    odin_global_walk_floor_z_min);
    node->declare_parameter<double>("dual_tune.odin_global_walk_floor_z_max",
                                    odin_global_walk_floor_z_max);
    node->declare_parameter<int>("dual_tune.odin_global_walk_floor_stride",
                                 odin_global_walk_floor_stride);
    node->declare_parameter<double>("dual_tune.odin_s2_max_clip_ratio", odin_s2_max_clip_ratio);
    node->declare_parameter<bool>("dual_tune.odin_body_pub_en", odin_body_pub_en);
    node->declare_parameter<bool>("dual_tune.shallow_s1_en", shallow_s1_en);
    node->declare_parameter<double>("dual_tune.shallow_s1_end_dt", shallow_s1_end_dt);
    node->declare_parameter<double>("dual_tune.shallow_s1_gyr", shallow_s1_gyr);
    node->declare_parameter<int>("dual_tune.shallow_s1_iters", shallow_s1_iters);
    node->declare_parameter<double>("dual_tune.odin_wall_nz_thr", odin_wall_nz_thr);
    node->declare_parameter<double>("dual_tune.odin_wall_info_boost", odin_wall_info_boost);
    node->declare_parameter<double>("dual_tune.odin_floor_info_scale", odin_floor_info_scale);
    node->declare_parameter<bool>("dual_tune.odin_voxel_insert_en", odin_voxel_insert_en);
    node->declare_parameter<bool>("dual_tune.odin_voxel_require_still", odin_voxel_require_still);
    node->declare_parameter<double>("dual_tune.odin_voxel_max_clip", odin_voxel_max_clip);
    node->declare_parameter<int>("dual_tune.odin_voxel_max_pts", odin_voxel_max_pts);
    node->declare_parameter<double>("dual_tune.odin_voxel_body_z_min", odin_voxel_body_z_min);
    node->declare_parameter<double>("dual_tune.odin_voxel_cov_scale", odin_voxel_cov_scale);
    node->declare_parameter<bool>("dual_tune.odin_joint_only_en", odin_joint_only_en);
    node->declare_parameter<bool>("dual_tune.residual_z_en", residual_z_en);
    node->declare_parameter<double>("dual_tune.residual_z_flat_z_alpha", residual_z_flat_z_alpha);
    node->declare_parameter<double>("dual_tune.residual_z_flat_vz_alpha", residual_z_flat_vz_alpha);
    node->declare_parameter<double>("dual_tune.residual_z_flat_ba_z_alpha", residual_z_flat_ba_z_alpha);
    node->declare_parameter<double>("dual_tune.residual_z_flat_max_step", residual_z_flat_max_step);
    node->declare_parameter<double>("dual_tune.residual_z_hard_band", residual_z_hard_band);
    node->declare_parameter<double>("dual_tune.residual_z_floor_nz_thr", residual_z_floor_nz_thr);
    node->declare_parameter<double>("dual_tune.residual_z_floor_info_scale", residual_z_floor_info_scale);
    node->declare_parameter<double>("dual_tune.residual_z_h_dampen", residual_z_h_dampen);
    node->declare_parameter<double>("dual_tune.residual_z_dz_tol", residual_z_dz_tol);
    node->declare_parameter<double>("dual_tune.residual_z_floor_ratio_thr", residual_z_floor_ratio_thr);
    node->declare_parameter<double>("dual_tune.residual_z_multilevel_thr", residual_z_multilevel_thr);
    node->declare_parameter<double>("dual_tune.residual_z_vz_alpha", residual_z_vz_alpha);
    node->declare_parameter<bool>("dual_tune.odin_z_factor_en", odin_z_factor_en);
    node->declare_parameter<bool>("dual_tune.odin_z_replace_en", odin_z_replace_en);
    node->declare_parameter<bool>("dual_tune.odin_xyz_replace_en", odin_xyz_replace_en);
    node->declare_parameter<bool>("dual_tune.odin_xy_factor_en", odin_xy_factor_en);
    node->declare_parameter<double>("dual_tune.odin_xy_factor_alpha", odin_xy_factor_alpha);
    node->declare_parameter<double>("dual_tune.odin_xy_factor_max_step", odin_xy_factor_max_step);
    node->declare_parameter<bool>("dual_tune.odin_pose_hold_on_bad", odin_pose_hold_on_bad);
    node->declare_parameter<bool>("dual_tune.odin_z_flat_hold_en", odin_z_flat_hold_en);
    node->declare_parameter<double>("dual_tune.odin_z_flat_vz_thr", odin_z_flat_vz_thr);
    node->declare_parameter<bool>("dual_tune.odin_att_replace_en", odin_att_replace_en);
    node->declare_parameter<bool>("dual_tune.odin_att_rp_only", odin_att_rp_only);
    node->declare_parameter<std::string>("dual_tune.odin_odom_topic", odin_odom_topic);
    node->declare_parameter<double>("dual_tune.odin_z_factor_alpha", odin_z_factor_alpha);
    node->declare_parameter<double>("dual_tune.odin_z_factor_max_dt", odin_z_factor_max_dt);
    node->declare_parameter<double>("dual_tune.odin_z_factor_max_innov", odin_z_factor_max_innov);
    node->declare_parameter<double>("dual_tune.odin_z_factor_band", odin_z_factor_band);
    node->declare_parameter<double>("dual_tune.odin_z_factor_catch_err", odin_z_factor_catch_err);
    node->declare_parameter<double>("dual_tune.odin_z_factor_catch_frac", odin_z_factor_catch_frac);
    node->declare_parameter<double>("dual_tune.odin_z_factor_catch_max", odin_z_factor_catch_max);
    node->declare_parameter<double>("dual_tune.odin_z_factor_catch_alpha", odin_z_factor_catch_alpha);
    node->declare_parameter<int>("dual_tune.odin_z_factor_catch_min_feat", odin_z_factor_catch_min_feat);
    node->declare_parameter<bool>("dual_tune.odin_z_factor_one_sided", odin_z_factor_one_sided);
    node->declare_parameter<double>("dual_tune.odin_z_factor_floor_scale", odin_z_factor_floor_scale);
    node->get_parameter("dual_tune.odin_min_pts", odin_min_pts);
    node->get_parameter("dual_tune.odin_min_frac", odin_min_frac);
    {
        double z0 = odin_body_z_min, z1 = odin_body_z_max;
        node->get_parameter("dual_tune.odin_body_z_min", z0);
        node->get_parameter("dual_tune.odin_body_z_max", z1);
        odin_body_z_min = static_cast<float>(z0);
        odin_body_z_max = static_cast<float>(z1);
    }
    node->get_parameter("dual_tune.odin_static_gyro", odin_static_gyro);
    node->get_parameter("dual_tune.odin_s2_enable", odin_s2_enable);
    node->get_parameter("dual_tune.odin_iekf_max_pts", odin_iekf_max_pts);
    node->get_parameter("dual_tune.odin_iekf_max_end_dt", odin_iekf_max_end_dt);
    node->get_parameter("dual_tune.odin_info_weight", odin_info_weight);
    node->get_parameter("dual_tune.odin_range_ref", odin_range_ref);
    node->get_parameter("dual_tune.odin_max_residual", odin_max_residual);
    node->get_parameter("dual_tune.odin_s2_gyr_gate", odin_s2_gyr_gate);
    node->get_parameter("dual_tune.iekf_max_upd", iekf_max_upd);
    node->get_parameter("dual_tune.iekf_max_rot", iekf_max_rot);
    node->get_parameter("dual_tune.iekf_max_vel", iekf_max_vel);
    node->get_parameter("dual_tune.iekf_s2_max_upd", iekf_s2_max_upd);
    node->get_parameter("dual_tune.iekf_s2_max_rot", iekf_s2_max_rot);
    node->get_parameter("dual_tune.consec_bad_freeze", consec_bad_freeze);
    node->get_parameter("dual_tune.map_jump_upd", map_jump_upd);
    node->get_parameter("dual_tune.map_jump_frame", map_jump_frame);
    {
        double mr = map_max_range;
        node->get_parameter("dual_tune.map_max_range", mr);
        map_max_range = static_cast<float>(mr);
    }
    node->get_parameter("dual_tune.map_require_still", map_require_still);
    node->get_parameter("dual_tune.map_still_gyr", map_still_gyr);
    node->get_parameter("dual_tune.map_use_downsample", map_use_downsample);
    node->get_parameter("dual_tune.odin_global_insert_en", odin_global_insert_en);
    node->get_parameter("dual_tune.odin_global_require_s2", odin_global_require_s2);
    node->get_parameter("dual_tune.odin_global_still_densify_en", odin_global_still_densify_en);
    node->get_parameter("dual_tune.odin_global_still_gyr", odin_global_still_gyr);
    node->get_parameter("dual_tune.odin_global_walk_floor_densify_en",
                        odin_global_walk_floor_densify_en);
    {
        double z0 = odin_global_walk_floor_z_min, z1 = odin_global_walk_floor_z_max;
        node->get_parameter("dual_tune.odin_global_walk_floor_z_min", z0);
        node->get_parameter("dual_tune.odin_global_walk_floor_z_max", z1);
        odin_global_walk_floor_z_min = static_cast<float>(z0);
        odin_global_walk_floor_z_max = static_cast<float>(z1);
    }
    node->get_parameter("dual_tune.odin_global_walk_floor_stride",
                        odin_global_walk_floor_stride);
    if (odin_global_walk_floor_stride < 1) odin_global_walk_floor_stride = 1;
    node->get_parameter("dual_tune.odin_s2_max_clip_ratio", odin_s2_max_clip_ratio);
    node->get_parameter("dual_tune.odin_body_pub_en", odin_body_pub_en);
    node->get_parameter("dual_tune.shallow_s1_en", shallow_s1_en);
    node->get_parameter("dual_tune.shallow_s1_end_dt", shallow_s1_end_dt);
    node->get_parameter("dual_tune.shallow_s1_gyr", shallow_s1_gyr);
    node->get_parameter("dual_tune.shallow_s1_iters", shallow_s1_iters);
    if (shallow_s1_iters < 1) shallow_s1_iters = 1;
    node->get_parameter("dual_tune.odin_wall_nz_thr", odin_wall_nz_thr);
    node->get_parameter("dual_tune.odin_wall_info_boost", odin_wall_info_boost);
    node->get_parameter("dual_tune.odin_floor_info_scale", odin_floor_info_scale);
    node->get_parameter("dual_tune.odin_voxel_insert_en", odin_voxel_insert_en);
    node->get_parameter("dual_tune.odin_voxel_require_still", odin_voxel_require_still);
    node->get_parameter("dual_tune.odin_voxel_max_clip", odin_voxel_max_clip);
    node->get_parameter("dual_tune.odin_voxel_max_pts", odin_voxel_max_pts);
    {
        double zmin = odin_voxel_body_z_min;
        node->get_parameter("dual_tune.odin_voxel_body_z_min", zmin);
        odin_voxel_body_z_min = static_cast<float>(zmin);
    }
    node->get_parameter("dual_tune.odin_voxel_cov_scale", odin_voxel_cov_scale);
    if (odin_voxel_max_pts < 1) odin_voxel_max_pts = 1;
    node->get_parameter("dual_tune.odin_joint_only_en", odin_joint_only_en);
    node->get_parameter("dual_tune.residual_z_en", residual_z_en);
    node->get_parameter("dual_tune.residual_z_flat_z_alpha", residual_z_flat_z_alpha);
    node->get_parameter("dual_tune.residual_z_flat_vz_alpha", residual_z_flat_vz_alpha);
    node->get_parameter("dual_tune.residual_z_flat_ba_z_alpha", residual_z_flat_ba_z_alpha);
    node->get_parameter("dual_tune.residual_z_flat_max_step", residual_z_flat_max_step);
    node->get_parameter("dual_tune.residual_z_hard_band", residual_z_hard_band);
    node->get_parameter("dual_tune.residual_z_floor_nz_thr", residual_z_floor_nz_thr);
    node->get_parameter("dual_tune.residual_z_floor_info_scale", residual_z_floor_info_scale);
    node->get_parameter("dual_tune.residual_z_h_dampen", residual_z_h_dampen);
    node->get_parameter("dual_tune.residual_z_dz_tol", residual_z_dz_tol);
    node->get_parameter("dual_tune.residual_z_floor_ratio_thr", residual_z_floor_ratio_thr);
    node->get_parameter("dual_tune.residual_z_multilevel_thr", residual_z_multilevel_thr);
    node->get_parameter("dual_tune.residual_z_vz_alpha", residual_z_vz_alpha);
    node->get_parameter("dual_tune.odin_z_factor_en", odin_z_factor_en);
    node->get_parameter("dual_tune.odin_z_replace_en", odin_z_replace_en);
    node->get_parameter("dual_tune.odin_xyz_replace_en", odin_xyz_replace_en);
    node->get_parameter("dual_tune.odin_xy_factor_en", odin_xy_factor_en);
    node->get_parameter("dual_tune.odin_xy_factor_alpha", odin_xy_factor_alpha);
    node->get_parameter("dual_tune.odin_xy_factor_max_step", odin_xy_factor_max_step);
    node->get_parameter("dual_tune.odin_pose_hold_on_bad", odin_pose_hold_on_bad);
    node->get_parameter("dual_tune.odin_z_flat_hold_en", odin_z_flat_hold_en);
    node->get_parameter("dual_tune.odin_z_flat_vz_thr", odin_z_flat_vz_thr);
    node->get_parameter("dual_tune.odin_att_replace_en", odin_att_replace_en);
    node->get_parameter("dual_tune.odin_att_rp_only", odin_att_rp_only);
    node->get_parameter("dual_tune.odin_odom_topic", odin_odom_topic);
    node->get_parameter("dual_tune.odin_z_factor_alpha", odin_z_factor_alpha);
    node->get_parameter("dual_tune.odin_z_factor_max_dt", odin_z_factor_max_dt);
    node->get_parameter("dual_tune.odin_z_factor_max_innov", odin_z_factor_max_innov);
    node->get_parameter("dual_tune.odin_z_factor_band", odin_z_factor_band);
    node->get_parameter("dual_tune.odin_z_factor_catch_err", odin_z_factor_catch_err);
    node->get_parameter("dual_tune.odin_z_factor_catch_frac", odin_z_factor_catch_frac);
    node->get_parameter("dual_tune.odin_z_factor_catch_max", odin_z_factor_catch_max);
    node->get_parameter("dual_tune.odin_z_factor_catch_alpha", odin_z_factor_catch_alpha);
    node->get_parameter("dual_tune.odin_z_factor_catch_min_feat", odin_z_factor_catch_min_feat);
    node->get_parameter("dual_tune.odin_z_factor_one_sided", odin_z_factor_one_sided);
    node->get_parameter("dual_tune.odin_z_factor_floor_scale", odin_z_factor_floor_scale);
    std::cout << "[dual_tune] s2=" << odin_s2_enable
              << " end_dt=" << odin_iekf_max_end_dt
              << " gyr=" << odin_s2_gyr_gate
              << " shallow_s1=" << shallow_s1_en
              << "/" << shallow_s1_iters
              << " sh_edt=" << shallow_s1_end_dt
              << " sh_gyr=" << shallow_s1_gyr
              << " wall_boost=" << odin_wall_info_boost
              << " odin_vox=" << odin_voxel_insert_en
              << " vox_still=" << odin_voxel_require_still
              << " joint_only=" << odin_joint_only_en
              << " w=" << odin_info_weight
              << " res=" << odin_max_residual
              << " clip_max=" << odin_s2_max_clip_ratio
              << " still=" << map_require_still
              << " downs=" << map_use_downsample
              << " odin_global=" << odin_global_insert_en
              << " req_s2=" << odin_global_require_s2
              << " still_den=" << odin_global_still_densify_en
              << "@" << odin_global_still_gyr
              << " walk_floor=" << odin_global_walk_floor_densify_en
              << " odin_body=" << odin_body_pub_en
              << " residual_z=" << residual_z_en
              << " flat_z_a=" << residual_z_flat_z_alpha
              << " flat_step=" << residual_z_flat_max_step
              << " hard_z=" << residual_z_hard_band
              << " floor_info=" << residual_z_floor_info_scale
              << " h_damp=" << residual_z_h_dampen
              << " ml_thr=" << residual_z_multilevel_thr
              << " odin_z_factor=" << odin_z_factor_en
              << " odin_z_replace=" << odin_z_replace_en
              << " odin_xyz_replace=" << odin_xyz_replace_en
              << " odin_xy_factor=" << odin_xy_factor_en
              << " odin_pose_hold_on_bad=" << odin_pose_hold_on_bad
              << " flat_hold=" << odin_z_flat_hold_en
              << " flat_vz_thr=" << odin_z_flat_vz_thr
              << " odin_att_replace=" << odin_att_replace_en
              << " att_rp_only=" << odin_att_rp_only
              << " band=" << odin_z_factor_band
              << " one_sided=" << odin_z_factor_one_sided
              << " catch_err=" << odin_z_factor_catch_err
              << std::endl;

    if(extrinT_l2.size()==3) LiDAR2_T_wrt_LiDAR1<<VEC_FROM_ARRAY(extrinT_l2);
    // Yaml lists R row-major. Eigen comma-init is column-major; MAT_FROM_ARRAY
    // transposed Ry(±13°) → Odin tilted ~13° vs Mid. Load RowMajor explicitly.
    if(extrinR_l2.size()==9)
      LiDAR2_R_wrt_LiDAR1 = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(extrinR_l2.data());
    if(extrinT_oi.size()==3) Lidar2_T_wrt_IMU2<<VEC_FROM_ARRAY(extrinT_oi);
    if(extrinR_oi.size()==9)
      Lidar2_R_wrt_IMU2 = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(extrinR_oi.data());
    std::cout << "LiDAR2_R_wrt_LiDAR1 (row-major yaml):\n" << LiDAR2_R_wrt_LiDAR1 << std::endl;
    std::cout << "LiDAR2_T_wrt_LiDAR1: " << LiDAR2_T_wrt_LiDAR1.transpose() << std::endl;

    // noise model params
    node->declare_parameter<double>("noise_model.ranging_cov", 0.02);
    node->declare_parameter<double>("noise_model.angle_cov", 0.05);
    node->declare_parameter<double>("noise_model.gyr_cov", 0.1);
    node->declare_parameter<double>("noise_model.acc_cov", 0.1);
    node->declare_parameter<double>("noise_model.b_gyr_cov", 0.0001);
    node->declare_parameter<double>("noise_model.b_acc_cov", 0.0001);
    node->get_parameter("noise_model.ranging_cov", ranging_cov);
    node->get_parameter("noise_model.angle_cov", angle_cov);
    node->get_parameter("noise_model.gyr_cov", gyr_cov);
    node->get_parameter("noise_model.acc_cov", acc_cov);
    node->get_parameter("noise_model.b_gyr_cov", b_gyr_cov);
    node->get_parameter("noise_model.b_acc_cov", b_acc_cov);

    // visualization params
    node->declare_parameter<bool>("publish.pub_voxel_map", false);
    node->declare_parameter<bool>("publish.publish_global_cloud_map", false);
    node->declare_parameter<int>("publish.publish_max_voxel_layer", 0);
    node->declare_parameter<double>("publish.global_map_leaf_size", 0.1);
    node->declare_parameter<double>("publish.global_map_pub_leaf_size", global_map_pub_leaf_size);
    node->declare_parameter<double>("publish.global_map_pub_period", global_map_pub_period);
    node->declare_parameter<int>("publish.global_map_pub_max_pts", global_map_pub_max_pts);
    node->get_parameter("publish.pub_voxel_map", publish_voxel_map);
    node->get_parameter("publish.publish_global_cloud_map", publish_global_cloud_map);
    node->get_parameter("publish.publish_max_voxel_layer", publish_max_voxel_layer);
    node->get_parameter("publish.global_map_leaf_size", global_map_leaf_size);
    node->get_parameter("publish.global_map_pub_leaf_size", global_map_pub_leaf_size);
    node->get_parameter("publish.global_map_pub_period", global_map_pub_period);
    node->get_parameter("publish.global_map_pub_max_pts", global_map_pub_max_pts);
    // Allow 0.05 for denser far ground; clamp extremes for PCL VoxelGrid safety.
    if (global_map_leaf_size < 0.03) global_map_leaf_size = 0.03;
    if (global_map_leaf_size > 1.0) global_map_leaf_size = 1.0;
    if (global_map_pub_leaf_size < global_map_leaf_size)
        global_map_pub_leaf_size = global_map_leaf_size;
    if (global_map_pub_leaf_size > 1.0) global_map_pub_leaf_size = 1.0;
    if (global_map_pub_period < 0.5) global_map_pub_period = 0.5;
    if (global_map_pub_max_pts < 50000) global_map_pub_max_pts = 50000;
    std::cout << "global_map_pub: leaf=" << global_map_pub_leaf_size
              << " period=" << global_map_pub_period
              << "s max_pts=" << global_map_pub_max_pts << std::endl;

    node->declare_parameter<double>("preprocess.blind", 0.01);
    node->declare_parameter<int>("preprocess.lidar_type", AVIA);
    node->declare_parameter<int>("preprocess.scan_line", 16);
    node->declare_parameter<int>("preprocess.scan_rate", 10);
    node->declare_parameter<int>("preprocess.point_filter_num", 1);
    node->declare_parameter<double>("preprocess.min_range", 0.3);
    node->declare_parameter<double>("preprocess.max_range", 25.0);
    node->declare_parameter<bool>("preprocess.feature_extract_enable", false);
    node->get_parameter("preprocess.blind", p_pre->blind);
    node->get_parameter("preprocess.lidar_type", p_pre->lidar_type);
    node->get_parameter("preprocess.scan_line", p_pre->N_SCANS);
    node->get_parameter("preprocess.scan_rate", p_pre->SCAN_RATE);
    node->get_parameter("preprocess.point_filter_num", p_pre->point_filter_num);
    node->get_parameter("preprocess.min_range", p_pre->min_range);
    node->get_parameter("preprocess.max_range", p_pre->max_range);
    node->get_parameter("preprocess.feature_extract_enable", p_pre->feature_enabled);
    node->declare_parameter<int>("preprocess.lidar_type2", ODIN1);
    node->declare_parameter<int>("preprocess.point_filter_num2", 1);
    node->declare_parameter<double>("preprocess.blind2", 0.3);
    node->declare_parameter<double>("preprocess.min_range2", 0.3);
    node->declare_parameter<double>("preprocess.max_range2", 25.0);
    node->declare_parameter<double>("preprocess.min_confidence2", 30.0);
    int lidar_type2=ODIN1, filter2=1; double blind2=0.3,minr2=25.0,maxr2=25.0,conf2=30.0;
    node->get_parameter("preprocess.lidar_type2", lidar_type2);
    node->get_parameter("preprocess.point_filter_num2", filter2);
    node->get_parameter("preprocess.blind2", blind2);
    node->get_parameter("preprocess.min_range2", minr2);
    node->get_parameter("preprocess.max_range2", maxr2);
    node->get_parameter("preprocess.min_confidence2", conf2);
    p_pre2->set(false,lidar_type2,blind2,filter2);
    p_pre2->min_range=minr2; p_pre2->max_range=maxr2; p_pre2->min_confidence=static_cast<float>(conf2);
    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;

    node->declare_parameter<bool>("pcd_save.pcd_save_en", false);
    node->declare_parameter<std::string>("pcd_save.save_path", "./map_output");
    node->get_parameter("pcd_save.pcd_save_en", pcd_save_en);
    node->get_parameter("pcd_save.save_path", pcd_save_path);

    for (size_t i = 0; i < layer_point_size.size(); i++) {
        layer_size.push_back(static_cast<int>(layer_point_size[i]));
    }

    path.header.stamp    = node->now();
    path.header.frame_id ="camera_init";

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    // leaf 0.1 for denser Global (official-like). If PCL VoxelGrid overflows on
    // huge extents, timer falls back to the unfiltered copy.
    downSizeFilterGlobal.setLeafSize(global_map_leaf_size, global_map_leaf_size, global_map_leaf_size);
    std::cout << "global_map_leaf_size: " << global_map_leaf_size << std::endl;
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_odin_imu_extrinsic(Lidar2_T_wrt_IMU2, Lidar2_R_wrt_IMU2);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, observation_model_share, NUM_MAX_ITERATIONS, epsi);

    /*** ROS2 subscribe / publish initialization ***/
    // auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(200)).best_effort().durability_volatile();
    auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(500)).best_effort().durability_volatile();
    auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(2000)).best_effort().durability_volatile();

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_livox;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl;
    if (p_pre->lidar_type == AVIA) {
        sub_livox = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lid_topic, lidar_qos, livox_pcl_cbk);
    } else {
        sub_pcl = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            lid_topic, lidar_qos, standard_pcl_cbk);
    }
    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, imu_qos, imu_cbk);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl2;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu2;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odin_odom;
    if (multi_lidar_en)
    {
        auto odin_cloud_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
        auto odin_imu_qos = rclcpp::QoS(rclcpp::KeepLast(4000)).reliable();
        sub_pcl2 = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            lid_topic2, odin_cloud_qos, standard_pcl_cbk2);
        sub_imu2 = node->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic2, odin_imu_qos, imu2_cbk);
    }
    // Need Odin odom for Z/XY factors, replace, or pose-hold.
    if (odin_z_factor_en || odin_z_replace_en || odin_xy_factor_en ||
        odin_pose_hold_on_bad)
    {
        auto odin_odom_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
        sub_odin_odom = node->create_subscription<nav_msgs::msg::Odometry>(
            odin_odom_topic, odin_odom_qos, odin_odom_cbk);
        std::cout << "[pv-z] subscribe " << odin_odom_topic
                  << " replace=" << odin_z_replace_en
                  << " z_factor=" << odin_z_factor_en
                  << " xy_factor=" << odin_xy_factor_en
                  << " multi=" << multi_lidar_en << std::endl;
    }

    auto pubLaserCloudFull = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100);
    auto pubLaserCloudFull_body = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 100);
    auto pubLaserCloudEffect = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100);
    auto pubLaserCloudMap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100);
    auto pubOdomAftMapped = node->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 100);
    auto pubExtrinsic = node->create_publisher<nav_msgs::msg::Odometry>("/Extrinsic", 100);
    auto pubPath = node->create_publisher<nav_msgs::msg::Path>("/path", 100000);
    auto voxel_map_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("/planes", 10000);
    auto pubGlobalCloudMap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/Global_cloud_map", 10);

    auto pubFreqPose = node->create_publisher<geometry_msgs::msg::PoseStamped>("/freq_pose", 10);

    auto save_map_srv = node->create_service<std_srvs::srv::Trigger>("/save_map", saveMapCallback);

    auto tf_br = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    PointCloudXYZI::Ptr global_pending_t(new PointCloudXYZI());
    PointCloudXYZI::Ptr global_chunk_t(new PointCloudXYZI());
    PointCloudXYZI::Ptr global_pub_t(new PointCloudXYZI());

    // Timer: voxelize new points each 1s. Periodically re-VoxelGrid the FULL
    // Global map async — pending-only left Mid360 non-repetitive scans as
    // independent centroid lattices ("一团一团") and wavy floors.
    // RViz gets a coarser, rate-limited copy so Jetson CPU is not stolen from IEKF.
    auto timer = node->create_wall_timer(1s,
        [&]()
        {
            static int global_remap_tick = 0;
            static auto last_global_pub_tp =
                std::chrono::steady_clock::now() -
                std::chrono::milliseconds(static_cast<int>(global_map_pub_period * 2000));
            {
                std::lock_guard<std::mutex> lck(global_cloud_map_mtx);
                if (global_cloud_map_pending->empty() && global_cloud_map->empty())
                    return;
                global_pending_t.swap(global_cloud_map_pending);
                global_cloud_map_pending.reset(new PointCloudXYZI());
                *global_pub_t = *global_cloud_map;
            }
            if (!global_pending_t->empty())
            {
                downSizeFilterGlobal.setInputCloud(global_pending_t);
                downSizeFilterGlobal.filter(*global_chunk_t);
                if (global_chunk_t->empty() && !global_pending_t->empty())
                    *global_chunk_t = *global_pending_t;
                {
                    std::lock_guard<std::mutex> lck(global_cloud_map_mtx);
                    *global_cloud_map += *global_chunk_t;
                    *global_pub_t = *global_cloud_map;
                }
            }
            // Every 5s: unify leaf lattice across history (async, non-blocking).
            if ((++global_remap_tick % 5) == 0 &&
                !global_pub_t->empty() &&
                !global_remap_busy.exchange(true))
            {
                PointCloudXYZI::Ptr remap_in(new PointCloudXYZI(*global_pub_t));
                const float leaf = static_cast<float>(global_map_leaf_size);
                std::thread([remap_in, leaf]() {
                    pcl::VoxelGrid<PointType> vg;
                    vg.setLeafSize(leaf, leaf, leaf);
                    PointCloudXYZI::Ptr remap_out(new PointCloudXYZI());
                    vg.setInputCloud(remap_in);
                    vg.filter(*remap_out);
                    if (!remap_out->empty())
                    {
                        std::lock_guard<std::mutex> lck(global_cloud_map_mtx);
                        // Replace history only; leave pending for the 1s path
                        // (pending was not in remap_in snapshot).
                        global_cloud_map.swap(remap_out);
                    }
                    global_remap_busy.store(false);
                }).detach();
            }
            if (!publish_global_cloud_map || global_pub_t->empty())
                return;
            const auto now_tp = std::chrono::steady_clock::now();
            const double since_pub =
                std::chrono::duration<double>(now_tp - last_global_pub_tp).count();
            if (since_pub < global_map_pub_period)
                return;

            PointCloudXYZI::Ptr viz(new PointCloudXYZI());
            if (global_map_pub_leaf_size > global_map_leaf_size + 1e-6)
            {
                pcl::VoxelGrid<PointType> vg_pub;
                const float leaf = static_cast<float>(global_map_pub_leaf_size);
                vg_pub.setLeafSize(leaf, leaf, leaf);
                vg_pub.setInputCloud(global_pub_t);
                vg_pub.filter(*viz);
                if (viz->empty())
                    *viz = *global_pub_t;
            }
            else
            {
                *viz = *global_pub_t;
            }
            if (static_cast<int>(viz->size()) > global_map_pub_max_pts &&
                global_map_pub_max_pts > 0)
            {
                const int stride =
                    (static_cast<int>(viz->size()) + global_map_pub_max_pts - 1) /
                    global_map_pub_max_pts;
                PointCloudXYZI::Ptr capped(new PointCloudXYZI());
                capped->points.reserve(static_cast<size_t>(global_map_pub_max_pts));
                for (size_t i = 0; i < viz->size(); i += static_cast<size_t>(stride))
                    capped->points.push_back(viz->points[i]);
                capped->width = capped->points.size();
                capped->height = 1;
                viz.swap(capped);
            }

            sensor_msgs::msg::PointCloud2 laserCloudmsg;
            pcl::toROSMsg(*viz, laserCloudmsg);
            laserCloudmsg.header.stamp = sec_to_stamp(lidar_end_time);
            laserCloudmsg.header.frame_id = "camera_init";
            pubGlobalCloudMap->publish(laserCloudmsg);
            last_global_pub_tp = now_tp;
        }
    );
//------------------------------------------------------------------------------------------------------
    // for Plane Map
    bool init_map = false;

    double sum_optimize_time = 0, sum_update_time = 0;
    int scan_index = 0;

    signal(SIGINT, SigHandle);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    rclcpp::Rate rate(5000);
    bool status = rclcpp::ok();

    while (status)
    {
        if (flg_exit) break;
        executor.spin_some(std::chrono::milliseconds(10));

        if(sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            p_imu->Process(Measures, kf, feats_undistort);
            size_t odin_used_now = 0;
            size_t odin_iekf_n = 0;
            const char *odin_skip = "none";
            bool fuse_is_hold = false;
            bool frame_quasi_static = false;
            frame_gyr_mean = 0.0;
            odin_frame_cov_scale = lidar2_cov_scale;
            iekf_odin_stage = false;
            const double now_wall = omp_get_wtime();
            // Display-only Odin (never into IEKF): hold of old body cloud at a
            // new Mid pose is the classic smear. Only allow when still.
            PointCloudXYZI::Ptr odin_display_only(new PointCloudXYZI());
            // Full gated Odin for Global/publish; stage-2 IEKF uses a subsample.
            PointCloudXYZI::Ptr odin_map_full(new PointCloudXYZI());
            PointCloudXYZI::Ptr odin_iekf_body(new PointCloudXYZI());
            auto subsample_odin_iekf = [&](const PointCloudXYZI &src) -> PointCloudXYZI {
                PointCloudXYZI out;
                if (src.empty()) return out;
                const size_t max_pts = static_cast<size_t>(std::max(1, odin_iekf_max_pts));
                if (src.size() <= max_pts)
                {
                    out = src;
                    return out;
                }
                const size_t stride = (src.size() + max_pts - 1) / max_pts;
                out.points.reserve((src.size() / stride) + 1);
                for (size_t i = 0; i < src.size(); i += stride)
                    out.points.push_back(src.points[i]);
                out.width = out.points.size();
                out.height = 1;
                return out;
            };
            auto hold_odin_display = [&](const char *tag) -> bool {
                if (!frame_quasi_static) return false;
                if (last_odin_fused && !last_odin_fused->empty() &&
                    last_odin_fused_wall > 0.0 &&
                    (now_wall - last_odin_fused_wall) <= 1.0)
                {
                    *odin_display_only = *last_odin_fused;
                    odin_used_now = odin_display_only->size();
                    odin_skip = tag;
                    fuse_is_hold = true;
                    return true;
                }
                return false;
            };
            auto odin_dense_enough = [&](size_t n) -> bool {
                if (n < odin_min_pts) return false;
                if (last_odin_good_n > 0 &&
                    n < static_cast<size_t>(odin_min_frac * static_cast<double>(last_odin_good_n)))
                    return false;
                return true;
            };
            auto imu2_quasi_static = [&]() -> bool {
                if (Measures.imu2.size() < 2) return true;
                double sum = 0.0;
                int n = 0;
                for (const auto &im : Measures.imu2)
                {
                    const double wx = im->angular_velocity.x;
                    const double wy = im->angular_velocity.y;
                    const double wz = im->angular_velocity.z;
                    sum += std::sqrt(wx * wx + wy * wy + wz * wz);
                    ++n;
                }
                if (n > 0) frame_gyr_mean = sum / n;
                return n > 0 && frame_gyr_mean < odin_static_gyro;
            };
            auto commit_odin_body = [&](PointCloudXYZI &odin_cloud, const char *tag) -> bool {
                Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
                T.block<3,3>(0,0) = LiDAR2_R_wrt_LiDAR1;
                T.block<3,1>(0,3) = LiDAR2_T_wrt_LiDAR1;
                pcl::transformPointCloud(odin_cloud, odin_cloud, T);
                PointCloudXYZI gated;
                gated.points.reserve(odin_cloud.points.size());
                for (const auto &pt : odin_cloud.points)
                {
                    if (pt.z < odin_body_z_min || pt.z > odin_body_z_max) continue;
                    PointType q = pt;
                    q.intensity = 200.0f;
                    gated.points.push_back(q);
                }
                gated.width = gated.points.size();
                gated.height = 1;
                if (!odin_dense_enough(gated.size()))
                    return false;
                *last_odin_fused = gated;
                last_odin_fused_wall = now_wall;
                last_odin_good_n = gated.size();
                *odin_map_full = gated;
                odin_used_now = gated.size();
                const double edt = std::abs(bundle_end_dt);
                {
                    // Time/motion soft only — do NOT multiply an extra ×2 on top of
                    // yaml lidar2_cov_scale (that made Odin ~200× Mid and fake-soft).
                    const double ratio = edt / 0.025;
                    double soft = std::max(1.0, ratio * ratio);
                    if (!frame_quasi_static) soft *= 2.0;
                    odin_frame_cov_scale = std::min(32.0, lidar2_cov_scale * soft);
                }
                const std::string tag_s = tag ? tag : "ok";
                const bool nodeskew = tag_s.find("nodeskew") != std::string::npos;
                // Never merge Odin into Mid feats_undistort (that was single-shot yank).
                // Gyro gate (FAST_LIO_MULTI): moving → map densify only.
                const bool s2_time_ok = edt <= odin_iekf_max_end_dt;
                const bool s2_motion_ok = frame_gyr_mean < odin_s2_gyr_gate;
                if (odin_s2_enable && s2_time_ok && s2_motion_ok)
                {
                    *odin_iekf_body = subsample_odin_iekf(gated);
                    odin_iekf_n = odin_iekf_body->size();
                    odin_skip = nodeskew ? "ok_nodeskew_s2" : "ok_s2";
                }
                else
                {
                    odin_iekf_body->clear();
                    odin_iekf_n = 0;
                    if (!s2_motion_ok)
                        odin_skip = "ok_map_gyr";
                    else
                        odin_skip = nodeskew ? "ok_nodeskew_map" : "ok_map";
                }
                return true;
            };

            // Prefer Odin IMU for still-gate; fall back to Mid IMU when unpaired.
            {
                const auto &imu_src = !Measures.imu2.empty() ? Measures.imu2 : Measures.imu;
                if (imu_src.size() >= 2)
                {
                    double sum = 0.0; int n = 0;
                    for (const auto &im : imu_src)
                    {
                        const double wx = im->angular_velocity.x;
                        const double wy = im->angular_velocity.y;
                        const double wz = im->angular_velocity.z;
                        sum += std::sqrt(wx * wx + wy * wy + wz * wz);
                        ++n;
                    }
                    frame_gyr_mean = (n > 0) ? (sum / n) : 0.0;
                    frame_quasi_static = frame_gyr_mean < odin_static_gyro;
                }
            }

            if (!multi_lidar_en)
                odin_skip = "multi_off";
            else if (feats_undistort->empty())
                odin_skip = "mid_empty";
            else if (!Measures.lidar2_fresh || !Measures.lidar2 || Measures.lidar2->empty()
                     || Measures.imu2.size() < 2)
            {
                if (!hold_odin_display("hold_1"))
                {
                    if (!Measures.lidar2_fresh)
                        odin_skip = "unpaired";
                    else if (!Measures.lidar2 || Measures.lidar2->empty())
                        odin_skip = "empty_l2";
                    else
                        odin_skip = "imu2_short";
                }
            }
            else if (!odin_dense_enough(Measures.lidar2->size()))
            {
                if (!hold_odin_display("hold_sparse"))
                    odin_skip = "sparse_l2";
            }
            else
            {
                PointCloudXYZI odin_cloud = *Measures.lidar2;
                bool deskew_ok = true;
                const char *tag = "ok";
                // Stationary: skip Odin deskew (az noise) ONLY if Mid-end is
                // close. |end_dt|>30ms + nodeskew left ground double layers.
                frame_quasi_static = imu2_quasi_static();
                const double mid_end_on_odin =
                    Measures.lidar_end_time - lidar2_stamp_bias_sec;
                const bool nodeskew_ok =
                    frame_quasi_static && (std::abs(bundle_end_dt) <= 0.03);
                if (nodeskew_ok)
                {
                    tag = "ok_nodeskew";
                }
                else
                {
                    deskew_ok = p_imu->UndistortOdinNative(odin_cloud, Measures,
                                                           V3D::Zero(), V3D::Zero(),
                                                           mid_end_on_odin);
                    tag = "ok";
                }
                if (deskew_ok)
                {
                    if (!commit_odin_body(odin_cloud, tag))
                    {
                        if (!hold_odin_display("hold_sparse_z"))
                            odin_skip = "sparse_z";
                    }
                }
                else if (!hold_odin_display("hold_deskew"))
                    odin_skip = "deskew_fail";
            }

            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            static int dual_frame_log = 0;
            if ((++dual_frame_log % 5) == 0 || odin_used_now == 0)
                std::cout << "[pv-dual] mid=" << Measures.lidar->size()
                          << " odin_in=" << (Measures.lidar2 ? Measures.lidar2->size() : 0)
                          << " odin_used=" << odin_used_now
                          << " odin_iekf=" << odin_iekf_n
                          << " odin_cov=" << odin_frame_cov_scale
                          << " imu2=" << Measures.imu2.size()
                          << " fused=" << feats_undistort->size()
                          << " pair_dt_ms="
                          << (bundle_pair_score < 0.0 ? -1.0 : bundle_pair_score * 1000.0)
                          << " end_dt_ms=" << bundle_end_dt * 1000.0
                          << " bias_ms=" << lidar2_stamp_bias_sec * 1000.0
                          << " skip=" << odin_skip
                          << std::endl;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(node->get_logger(), "No point, skip this scan!");
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;

            if (flg_EKF_inited && !init_map) {
                // Bootstrap Global with Mid-only: full fused first frame at a
                // still-settling pose left a permanent colored ghost sheet.
                PointCloudXYZI::Ptr boot_body(new PointCloudXYZI);
                boot_body->points.reserve(feats_undistort->size());
                for (const auto &pt : feats_undistort->points)
                {
                    if (pt.intensity >= 150.0f) continue;
                    boot_body->points.push_back(pt);
                }
                if (boot_body->empty())
                    *boot_body = *feats_undistort;
                boot_body->width = boot_body->points.size();
                boot_body->height = 1;
                PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI);
                transformLidar(state_point, boot_body, world_lidar);

                {
                    std::lock_guard<std::mutex> lck(global_cloud_map_mtx);
                    *global_cloud_map_pending += *world_lidar;
                }
                last_global_map_pos = state_point.pos;
                last_global_map_pose_ok = true;

                std::vector<pointWithCov> pv_list;

                std::cout << kf.get_P() << std::endl;

                for (size_t i = 0; i < world_lidar->size(); i++) {
                    pointWithCov pv;
                    pv.point << world_lidar->points[i].x, world_lidar->points[i].y,
                            world_lidar->points[i].z;
                    V3D point_this(feats_undistort->points[i].x,
                                   feats_undistort->points[i].y,
                                   feats_undistort->points[i].z);
                    if (point_this[2] == 0) {
                        point_this[2] = 0.001;
                    }
                    M3D cov_lidar = calcBodyCov(point_this, ranging_cov, angle_cov);
                    M3D cov_world = transformLiDARCovToWorld(point_this, kf, cov_lidar);

                    pv.cov = cov_world;
                    pv_list.push_back(pv);
                    Eigen::Vector3d sigma_pv = pv.cov.diagonal();
                    sigma_pv[0] = sqrt(sigma_pv[0]);
                    sigma_pv[1] = sqrt(sigma_pv[1]);
                    sigma_pv[2] = sqrt(sigma_pv[2]);
                }

                buildVoxelMap(pv_list, max_voxel_size, max_layer, layer_size,
                              max_points_size, max_points_size, min_eigen_value,
                              voxel_map);
                std::cout << "build voxel map" << std::endl;

                if (publish_voxel_map) {
                    pubVoxelMap(voxel_map, publish_max_voxel_layer, voxel_map_pub);
                    publish_frame_world(pubLaserCloudFull);
                    publish_frame_body(pubLaserCloudFull_body);
                }
                init_map = true;
                std::cout << "finish init_map" << std::endl;
                continue;
            }

            /*** downsample Mid features (Odin stays out of feats_undistort) ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);

            sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);

            feats_down_size = feats_down_body->points.size();

            var_down_body.clear();
            for (auto & pt:feats_down_body->points) {
                V3D point_this(pt.x, pt.y, pt.z);
                M3D cov_lidar = calcBodyCov(point_this, ranging_cov, angle_cov);
                var_down_body.push_back(cov_lidar);
            }

            if (feats_down_size < 5)
            {
                RCLCPP_WARN(node->get_logger(), "No point, skip this scan!");
                continue;
            }

            auto iekf_jump_metrics = [&](const state_ikfom &a, const state_ikfom &b,
                                         double &d_upd, double &d_rot, bool &bad) {
                bad = !std::isfinite(b.pos(0)) || !std::isfinite(b.pos(1)) || !std::isfinite(b.pos(2));
                const double pos_norm = b.pos.norm();
                if (!std::isfinite(pos_norm) || pos_norm > 1e4) bad = true;
                auto rc = b.rot.coeffs();
                double rn = 0;
                for (int i = 0; i < 4; i++) rn += rc[i] * rc[i];
                if (std::isnan(rn) || std::fabs(rn - 1.0) > 0.1) bad = true;
                d_upd = (b.pos - a.pos).norm();
                Eigen::Quaterniond qa(a.rot.coeffs()[3], a.rot.coeffs()[0],
                                      a.rot.coeffs()[1], a.rot.coeffs()[2]);
                Eigen::Quaterniond qb(b.rot.coeffs()[3], b.rot.coeffs()[0],
                                      b.rot.coeffs()[1], b.rot.coeffs()[2]);
                d_rot = qa.angularDistance(qb);
            };

            /*** IEKF: Mid-lock+S2 (default) or official-like joint-only Bundle ***/
            double t_update_start = omp_get_wtime();
            double t_update_end = t_update_start;
            double solve_H_time = 0;
            (void)solve_H_time;
            state_ikfom state_before_update = kf.get_x();
            esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P_before = kf.get_P();
            PointCloudXYZI::Ptr mid_down_body(new PointCloudXYZI(*feats_down_body));
            std::vector<M3D> mid_var_down = var_down_body;
            last_mid_effct_feat_num = 0;
            iekf_odin_stage = false;
            bool state_bad = false;
            bool odin_s2_accepted = false;
            double odin_s2_clip_ratio = 1.0;
            const size_t joint_mid_n = mid_down_body ? mid_down_body->size() : 0;

            auto build_joint_mid_odin = [&]() -> bool {
                if (!odin_iekf_body || odin_iekf_body->empty() ||
                    !mid_down_body || mid_down_body->empty() ||
                    mid_var_down.size() != joint_mid_n)
                    return false;
                feats_down_body->points.clear();
                feats_down_body->points.reserve(joint_mid_n + odin_iekf_body->size());
                feats_down_body->points.insert(feats_down_body->points.end(),
                                              mid_down_body->points.begin(),
                                              mid_down_body->points.end());
                feats_down_body->points.insert(feats_down_body->points.end(),
                                              odin_iekf_body->points.begin(),
                                              odin_iekf_body->points.end());
                feats_down_body->width = feats_down_body->points.size();
                feats_down_body->height = 1;
                feats_down_size = feats_down_body->points.size();
                var_down_body.clear();
                var_down_body.reserve(feats_down_size);
                var_down_body.insert(var_down_body.end(),
                                     mid_var_down.begin(), mid_var_down.end());
                for (const auto &pt : odin_iekf_body->points)
                {
                    V3D point_this(pt.x, pt.y, pt.z);
                    M3D cov_lidar = calcBodyCov(point_this, ranging_cov, angle_cov);
                    double s = odin_frame_cov_scale / std::max(1e-3, odin_info_weight);
                    const double r = point_this.norm();
                    const double rr = r / odin_range_ref;
                    s *= (1.0 + rr * rr);
                    const double dt_s =
                        std::fabs(static_cast<double>(pt.curvature)) / 1000.0;
                    const double lever =
                        std::max(0.15, static_cast<double>(std::hypot(pt.x, pt.y)));
                    const double tang = frame_gyr_mean * dt_s * lever;
                    s *= (1.0 + (tang / 0.05) * (tang / 0.05));
                    if (s > 128.0) s = 128.0;
                    cov_lidar *= s;
                    var_down_body.push_back(cov_lidar);
                }
                return feats_down_size >= 5;
            };

            const bool do_joint_only =
                odin_joint_only_en && odin_s2_enable && odin_iekf_n > 0 &&
                build_joint_mid_odin();

            if (do_joint_only)
            {
                // Official MULTI-like: one joint Mid+Odin update (no Mid-only S1).
                ++iekf_joint_only_n;
                iekf_odin_stage = true;
                const int clip_before = iekf_odin_res_clip;
                kf.set_maximum_iter(NUM_MAX_ITERATIONS);
                kf.update_iterated_dyn_share_diagonal();
                t_update_end = omp_get_wtime();
                sum_optimize_time += t_update_end - t_update_start;
                iekf_odin_stage = false;
                const int clipped = iekf_odin_res_clip - clip_before;
                odin_s2_clip_ratio =
                    (odin_iekf_n > 0)
                        ? (static_cast<double>(clipped) /
                           static_cast<double>(odin_iekf_n))
                        : 1.0;
                state_point = kf.get_x();
                double d_upd_chk = 0.0, d_rot_chk = 0.0;
                iekf_jump_metrics(state_before_update, state_point, d_upd_chk, d_rot_chk, state_bad);
                const bool mid_no_eff = (last_mid_effct_feat_num < 1);
                bool jump_reject = (!state_bad) && !mid_no_eff &&
                    (d_upd_chk > iekf_max_upd || d_rot_chk > iekf_max_rot);
                const bool clip_reject = odin_s2_clip_ratio > odin_s2_max_clip_ratio;
                if (state_bad || mid_no_eff || jump_reject || clip_reject)
                {
                    if (jump_reject || clip_reject)
                    {
                        ++iekf_reject_s2;
                        if ((iekf_reject_s2 % 10) == 1)
                            std::cout << "[pv-iekf] reject_joint d_upd=" << d_upd_chk
                                      << " d_rot=" << d_rot_chk
                                      << " clip_r=" << odin_s2_clip_ratio
                                      << " n=" << iekf_reject_s2 << std::endl;
                    }
                    kf.change_x(state_before_update);
                    kf.change_P(P_before);
                    state_point = kf.get_x();
                    state_bad = true;
                }
                else
                {
                    ++iekf_accept_s2;
                    odin_s2_accepted = true;
                    if (iekf_max_vel > 0.0)
                    {
                        const double vel_n = state_point.vel.norm();
                        if (vel_n > iekf_max_vel)
                        {
                            ++iekf_reject_vel;
                            state_ikfom s = state_point;
                            s.vel *= (iekf_max_vel / vel_n);
                            kf.change_x(s);
                            state_point = s;
                        }
                    }
                    if ((iekf_joint_only_n % 50) == 1)
                        std::cout << "[pv-iekf] joint_only mid=" << joint_mid_n
                                  << " odin=" << odin_iekf_n
                                  << " clip_r=" << odin_s2_clip_ratio
                                  << " n=" << iekf_joint_only_n << std::endl;
                }
                // Restore Mid-only cloud for voxel Mid insert path below.
                *feats_down_body = *mid_down_body;
                var_down_body = mid_var_down;
                feats_down_size = feats_down_body->points.size();
            }
            else
            {
            // Mid-only S1 lock, then optional joint S2 (legacy dual path).
            const bool shallow_s1 =
                shallow_s1_en && odin_s2_enable && odin_iekf_n > 0 &&
                (std::abs(bundle_end_dt) <= shallow_s1_end_dt) &&
                (frame_gyr_mean < shallow_s1_gyr);
            const int s1_iters =
                shallow_s1 ? shallow_s1_iters : NUM_MAX_ITERATIONS;
            kf.set_maximum_iter(s1_iters);
            if (shallow_s1)
            {
                ++iekf_shallow_s1;
                if ((iekf_shallow_s1 % 50) == 1)
                    std::cout << "[pv-iekf] shallow_s1 iters=" << s1_iters
                              << " end_dt_ms=" << bundle_end_dt * 1000.0
                              << " gyr=" << frame_gyr_mean
                              << " n=" << iekf_shallow_s1 << std::endl;
            }
            kf.update_iterated_dyn_share_diagonal();
            kf.set_maximum_iter(NUM_MAX_ITERATIONS);  // full iters for S2
            t_update_end = omp_get_wtime();
            sum_optimize_time += t_update_end - t_update_start;

            state_point = kf.get_x();
            double d_upd_chk = 0.0, d_rot_chk = 0.0;
            iekf_jump_metrics(state_before_update, state_point, d_upd_chk, d_rot_chk, state_bad);
            const bool mid_no_eff = (last_mid_effct_feat_num < 1);
            bool jump_reject = (!state_bad) && !mid_no_eff &&
                (d_upd_chk > iekf_max_upd || d_rot_chk > iekf_max_rot);
            if (state_bad || mid_no_eff || jump_reject)
            {
                if (jump_reject)
                {
                    ++iekf_reject_jump;
                    if ((iekf_reject_jump % 10) == 1)
                        std::cout << "[pv-iekf] reject_s1 d_upd=" << d_upd_chk
                                  << " d_rot=" << d_rot_chk
                                  << " n=" << iekf_reject_jump << std::endl;
                }
                else if (state_bad)
                {
                    RCLCPP_ERROR(node->get_logger(),
                        "Divergence detected (pos_norm=%.1f)! Restoring previous state.",
                        state_point.pos.norm());
                }
                kf.change_x(state_before_update);
                kf.change_P(P_before);
                state_point = kf.get_x();
                state_bad = true;
            }
            else if (iekf_max_vel > 0.0)
            {
                const double vel_n = state_point.vel.norm();
                if (vel_n > iekf_max_vel)
                {
                    ++iekf_reject_vel;
                    state_ikfom s = state_point;
                    s.vel *= (iekf_max_vel / vel_n);
                    kf.change_x(s);
                    state_point = s;
                    if ((iekf_reject_vel % 10) == 1)
                        std::cout << "[pv-iekf] clamp_vel |v|=" << vel_n
                                  << " -> " << iekf_max_vel
                                  << " d_upd=" << d_upd_chk
                                  << " n=" << iekf_reject_vel << std::endl;
                }
            }

            /*** Stage-2: joint Mid+Odin IEKF (same H as official Bundle) ***/
            state_ikfom state_after_mid = kf.get_x();
            esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P_after_mid = kf.get_P();
            if (!state_bad && odin_s2_enable && odin_iekf_body &&
                !odin_iekf_body->empty() && mid_down_body && !mid_down_body->empty() &&
                mid_var_down.size() == joint_mid_n && build_joint_mid_odin())
            {
                {
                    iekf_odin_stage = true;
                    const int clip_before = iekf_odin_res_clip;
                    t_update_start = omp_get_wtime();
                    kf.update_iterated_dyn_share_diagonal();
                    t_update_end = omp_get_wtime();
                    sum_optimize_time += t_update_end - t_update_start;
                    iekf_odin_stage = false;
                    const int clipped = iekf_odin_res_clip - clip_before;
                    odin_s2_clip_ratio =
                        (odin_iekf_n > 0)
                            ? (static_cast<double>(clipped) /
                               static_cast<double>(odin_iekf_n))
                            : 1.0;

                    state_ikfom state_s2 = kf.get_x();
                    bool s2_bad = false;
                    double s2_du = 0.0, s2_dr = 0.0;
                    iekf_jump_metrics(state_after_mid, state_s2, s2_du, s2_dr, s2_bad);
                    const bool s2_clip_reject =
                        odin_s2_clip_ratio > odin_s2_max_clip_ratio;
                    const bool s2_reject = s2_bad || s2_clip_reject ||
                        (s2_du > iekf_s2_max_upd) || (s2_dr > iekf_s2_max_rot);
                    if (s2_reject)
                    {
                        ++iekf_reject_s2;
                        if ((iekf_reject_s2 % 10) == 1)
                            std::cout << "[pv-iekf] reject_s2 d_upd=" << s2_du
                                      << " d_rot=" << s2_dr
                                      << " joint_mid=" << joint_mid_n
                                      << " odin=" << odin_iekf_n
                                      << " clip=" << clipped
                                      << " clip_r=" << odin_s2_clip_ratio
                                      << " n=" << iekf_reject_s2 << std::endl;
                        // Restore both pose and covariance (was x-only).
                        kf.change_x(state_after_mid);
                        kf.change_P(P_after_mid);
                        state_point = state_after_mid;
                    }
                    else
                    {
                        ++iekf_accept_s2;
                        odin_s2_accepted = true;
                        state_point = state_s2;
                        if ((iekf_accept_s2 % 50) == 1)
                            std::cout << "[pv-iekf] accept_s2 d_upd=" << s2_du
                                      << " d_rot=" << s2_dr
                                      << " joint_mid=" << joint_mid_n
                                      << " odin=" << odin_iekf_n
                                      << " clip=" << clipped
                                      << " clip_r=" << odin_s2_clip_ratio
                                      << " cov=" << odin_frame_cov_scale
                                      << " acc=" << iekf_accept_s2
                                      << " rej=" << iekf_reject_s2 << std::endl;
                    }
                }
                // Mid cloud for voxel Mid insert (Odin via odin_voxel_insert).
                *feats_down_body = *mid_down_body;
                var_down_body = mid_var_down;
                feats_down_size = feats_down_body->points.size();
            }
            } // end Mid-lock+S2 path (not joint_only)

            // Mid/S2 failed: do not keep integrating IMU into a wrong pose —
            // that wrote corrupt Mid planes into voxel_map and caused drift cascades.
            if (!state_bad)
            {
                if (!odin_z_replace_en)
                {
                    // Corridor Z: go2 flat residual_z, then soft Odin onboard Δz.
                    apply_residual_z_flat();
                    apply_odin_z_factor(lidar_end_time);
                    apply_residual_z_hard_clamp();
                }
                // Soft Odin XY (scale assist) after Mid IEKF + Z residual.
                if (!odin_xyz_replace_en)
                    apply_odin_xy_factor();
                state_point = kf.get_x();
                if (iekf_max_vel > 0.0)
                {
                    const double vel_n = state_point.vel.norm();
                    if (vel_n > iekf_max_vel)
                    {
                        ++iekf_reject_vel;
                        state_ikfom s = state_point;
                        s.vel *= (iekf_max_vel / vel_n);
                        kf.change_x(s);
                        state_point = s;
                        if ((iekf_reject_vel % 20) == 1)
                            std::cout << "[pv-iekf] clamp_vel_post |v|=" << vel_n
                                      << " -> " << iekf_max_vel
                                      << " n=" << iekf_reject_vel << std::endl;
                    }
                }
                last_good_lidar_state = state_point;
                last_good_P = kf.get_P();
                have_last_good_lidar = true;
                have_last_good_P = true;
                consec_iekf_bad = 0;
            }
            else
            {
                ++consec_iekf_bad;
                // Kill IMU coast immediately (was ~10s at freeze=100 → d_frame ~50m/s).
                {
                    state_ikfom s = kf.get_x();
                    s.vel.setZero();
                    kf.change_x(s);
                    state_point = s;
                }
                // Prefer Odin XYZ hold over freezing stale last_good (stairs).
                const bool held = apply_odin_pose_hold_on_bad();
                if (!held && have_last_good_lidar &&
                    consec_iekf_bad >= consec_bad_freeze)
                {
                    kf.change_x(last_good_lidar_state);
                    if (have_last_good_P) kf.change_P(last_good_P);
                    state_point = last_good_lidar_state;
                    if ((consec_iekf_bad % 20) == 0)
                        std::cout << "[pv-iekf] freeze_pose consec_bad="
                                  << consec_iekf_bad << std::endl;
                }
                else if (held)
                {
                    state_point = kf.get_x();
                    if ((consec_iekf_bad % 20) == 0)
                        std::cout << "[pv-iekf] odin_hold consec_bad="
                                  << consec_iekf_bad << std::endl;
                }
            }

            // Final Z from Odin odom (good or bad frame).
            if (odin_z_replace_en)
            {
                apply_odin_z_replace(lidar_end_time);
                state_point = kf.get_x();
                if (!state_bad)
                {
                    last_good_lidar_state = state_point;
                    last_good_P = kf.get_P();
                    have_last_good_lidar = true;
                    have_last_good_P = true;
                }
            }

            // Voxel/map insert must use Mid downs, not stage-2 Odin cloud.
            *feats_down_body = *mid_down_body;
            var_down_body = mid_var_down;
            feats_down_size = feats_down_body->points.size();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            /*** add the points to the voxel map ***/
            std::vector<pointWithCov> pv_list;
            PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI);
            transformLidar(state_point, feats_down_body, world_lidar);
            // Display map: denser undistorted cloud, but drop body-range outliers
            // (Mid preprocess historically lacked max_range → floating scatter).
            // Gate: hold frames + sudden pose jumps wrote permanent ~1m ghosts.
            {
                const double d_upd = (state_point.pos - state_before_update.pos).norm();
                const double d_frame = last_global_map_pose_ok
                    ? (state_point.pos - last_global_map_pos).norm() : 0.0;
                // Only gate on a large single EKF step while still. Do NOT use
                // sticky d_frame>threshold alone — after init a ~0.15m settle
                // permanently blocked all later inserts (ins_ok stuck at 0).
                const bool jump_gate =
                    frame_quasi_static && (d_upd > map_jump_upd);
                const bool motion_gate =
                    map_require_still && (frame_gyr_mean >= map_still_gyr);
                // When pose XYZ is pinned to Odin, Mid match failure must not
                // starve Global densify — otherwise Laser_map stays empty while
                // /Odometry tracks (looks like total drift in RViz).
                const bool trust_odin_pose = odin_xyz_replace_en;
                const char *skip_why = nullptr;
                if (state_bad && !trust_odin_pose) { skip_why = "bad"; ++map_skip_bad; }
                else if (fuse_is_hold && !trust_odin_pose) { skip_why = "hold"; ++map_skip_hold; }
                else if (jump_gate && !trust_odin_pose) { skip_why = "jump"; ++map_skip_jump; }
                else if (motion_gate) { skip_why = "motion"; ++map_skip_jump; }
                (void)d_frame; (void)map_jump_frame;

                if (skip_why == nullptr)
                {
                    // Global densify: still→dense fill; moving→downs (anti ground
                    // ripple / Mid360 smear clumps). map_use_downsample forces downs.
                    PointCloudXYZI::Ptr body_src =
                        map_use_downsample
                            ? feats_down_body
                            : ((dense_pub_en && frame_quasi_static)
                                   ? feats_undistort
                                   : feats_down_body);
                    PointCloudXYZI::Ptr body_f(new PointCloudXYZI);
                    // Odin→Global densify policy (anti double-wall + fill ground):
                    //  full cloud: S2 accepted, or still densify, or legacy !req_s2 map path
                    //  walk floor: while moving without S2, only near-floor Odin points
                    const bool s2_attempted = odin_iekf_n > 0;
                    const bool s2_rejected = s2_attempted && !odin_s2_accepted;
                    const bool still_for_odin =
                        odin_global_still_densify_en &&
                        (frame_gyr_mean < odin_global_still_gyr);
                    const bool full_odin_ok =
                        odin_global_insert_en && odin_map_full &&
                        !odin_map_full->empty() && !s2_rejected &&
                        (odin_s2_accepted ||
                         still_for_odin ||
                         (!odin_global_require_s2 && !s2_attempted));
                    // Floor-only is OK even after S2 reject: Mid pose restored,
                    // near-floor points fill holes without stacking corridor walls.
                    const bool floor_odin_ok =
                        odin_global_insert_en &&
                        odin_global_walk_floor_densify_en && odin_map_full &&
                        !odin_map_full->empty() && !full_odin_ok;
                    if (!full_odin_ok && !floor_odin_ok && odin_map_full &&
                        !odin_map_full->empty() && ((map_ins_ok % 50) == 1))
                    {
                        std::cout << "[pv-map] odin_skip_global s2="
                                  << odin_s2_accepted
                                  << " clip_r=" << odin_s2_clip_ratio
                                  << " req_s2=" << odin_global_require_s2
                                  << " gyr=" << frame_gyr_mean
                                  << std::endl;
                    }
                    const size_t odin_budget =
                        (full_odin_ok || floor_odin_ok) ? odin_map_full->size() : 0;
                    body_f->points.reserve(body_src->size() + odin_budget);
                    const float r2max = map_max_range * map_max_range;
                    for (const auto &pt : body_src->points)
                    {
                        // Drop IEKF-subsampled Odin; re-add full cloud below.
                        if (pt.intensity >= 150.0f) continue;
                        const float r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
                        if (r2 > r2max) continue;
                        body_f->points.push_back(pt);
                    }
                    if (full_odin_ok)
                    {
                        for (const auto &pt : odin_map_full->points)
                        {
                            const float r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
                            if (r2 > r2max) continue;
                            body_f->points.push_back(pt);
                        }
                    }
                    else if (floor_odin_ok)
                    {
                        const int stride = odin_global_walk_floor_stride;
                        for (size_t i = 0; i < odin_map_full->points.size(); i += stride)
                        {
                            const auto &pt = odin_map_full->points[i];
                            if (pt.z < odin_global_walk_floor_z_min ||
                                pt.z > odin_global_walk_floor_z_max)
                                continue;
                            const float r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
                            if (r2 > r2max) continue;
                            body_f->points.push_back(pt);
                        }
                        if ((map_ins_ok % 100) == 1)
                            std::cout << "[pv-map] odin_walk_floor gyr="
                                      << frame_gyr_mean
                                      << " n_src=" << odin_map_full->size()
                                      << std::endl;
                    }
                    body_f->width = body_f->points.size();
                    body_f->height = 1;
                    PointCloudXYZI::Ptr world_dense(new PointCloudXYZI);
                    transformLidar(state_point, body_f, world_dense);
                    std::lock_guard<std::mutex> lck(global_cloud_map_mtx);
                    *global_cloud_map_pending += *world_dense;
                    last_global_map_pos = state_point.pos;
                    last_global_map_pose_ok = true;
                    ++map_ins_ok;
                }
                else if (((map_skip_hold + map_skip_jump + map_skip_bad) % 10) == 1)
                {
                    std::cout << "[pv-map] skip_global why=" << skip_why
                              << " tag=" << odin_skip
                              << " d_upd=" << d_upd
                              << " d_frame=" << d_frame
                              << " ins_ok=" << map_ins_ok
                              << " skip_hold=" << map_skip_hold
                              << " skip_jump=" << map_skip_jump
                              << std::endl;
                }
            }

            Eigen::Vector3d ext_t = state_point.offset_T_L_I;
            auto ext_q_coeffs = state_point.offset_R_L_I.coeffs(); // [x, y, z, w]
            Eigen::Quaterniond ext_q(ext_q_coeffs[3], ext_q_coeffs[0], ext_q_coeffs[1], ext_q_coeffs[2]);
            Eigen::Matrix3d ext_R = ext_q.toRotationMatrix();
            (void)ext_t; (void)ext_R;
            // Match voxels: Mid always; Odin only when S2 accepted + low clip +
            // quasi-static (inflated cov, non-floor subsample) — closer to
            // official MULTI map insert without permanent floor smear.
            // CRITICAL: never insert when Mid match failed — wrong pose pollutes
            // planes → reject_s1 storm → visible drift / Eigen crash.
            if (!state_bad)
            {
                for (size_t i = 0; i < feats_down_body->size(); i++) {
                    if (feats_down_body->points[i].intensity >= 199.5f) continue;
                    pointWithCov pv;
                    pv.point << feats_down_body->points[i].x, feats_down_body->points[i].y, feats_down_body->points[i].z;
                    M3D cov_lidar = var_down_body[i];
                    M3D cov_world = transformLiDARCovToWorld(pv.point, kf, cov_lidar);

                    pv.cov = cov_world;
                    pv.point << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
                    pv_list.push_back(pv);
                }
                const bool odin_vox_ok =
                    odin_voxel_insert_en && odin_s2_accepted &&
                    odin_s2_clip_ratio <= odin_voxel_max_clip &&
                    (!odin_voxel_require_still || frame_quasi_static) &&
                    odin_iekf_body && !odin_iekf_body->empty();
                if (odin_vox_ok)
                {
                    PointCloudXYZI::Ptr odin_vox_body(new PointCloudXYZI);
                    const size_t n_src = odin_iekf_body->size();
                    const size_t max_n =
                        static_cast<size_t>(std::max(1, odin_voxel_max_pts));
                    const size_t stride =
                        std::max<size_t>(1, (n_src + max_n - 1) / max_n);
                    odin_vox_body->points.reserve(std::min(n_src, max_n));
                    for (size_t i = 0; i < n_src; i += stride)
                    {
                        const auto &pt = odin_iekf_body->points[i];
                        if (pt.z < odin_voxel_body_z_min) continue;
                        odin_vox_body->points.push_back(pt);
                        if (odin_vox_body->points.size() >= max_n) break;
                    }
                    odin_vox_body->width = odin_vox_body->points.size();
                    odin_vox_body->height = 1;
                    if (!odin_vox_body->empty())
                    {
                        PointCloudXYZI::Ptr odin_vox_world(new PointCloudXYZI);
                        transformLidar(state_point, odin_vox_body, odin_vox_world);
                        const double csc =
                            std::max(1.0, odin_voxel_cov_scale);
                        for (size_t i = 0; i < odin_vox_body->size(); ++i)
                        {
                            const auto &pt = odin_vox_body->points[i];
                            pointWithCov pv;
                            pv.point << pt.x, pt.y, pt.z;
                            M3D cov_lidar =
                                calcBodyCov(pv.point, ranging_cov, angle_cov);
                            cov_lidar *= csc;
                            M3D cov_world =
                                transformLiDARCovToWorld(pv.point, kf, cov_lidar);
                            pv.cov = cov_world;
                            pv.point << odin_vox_world->points[i].x,
                                odin_vox_world->points[i].y,
                                odin_vox_world->points[i].z;
                            pv_list.push_back(pv);
                        }
                        ++iekf_odin_voxel_ins;
                        if ((iekf_odin_voxel_ins % 50) == 1)
                            std::cout << "[pv-map] odin_voxel_ins n="
                                      << odin_vox_body->size()
                                      << " clip_r=" << odin_s2_clip_ratio
                                      << " total=" << iekf_odin_voxel_ins
                                      << std::endl;
                    }
                }

                t_update_start = omp_get_wtime();
                std::sort(pv_list.begin(), pv_list.end(), var_contrast);
                updateVoxelMapOMP(pv_list, max_voxel_size, max_layer, layer_size,
                               max_points_size, max_points_size, min_eigen_value,
                               voxel_map);
                t_update_end = omp_get_wtime();
                sum_update_time += t_update_end - t_update_start;
            }

            scan_index++;

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped, tf_br, node);
            publish_pose(pubFreqPose);

            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);

            // Publish Mid + optional Odin (odin_body_pub_en from dual_mid_odin_tune.py).
            PointCloudXYZI::Ptr pub_cloud(new PointCloudXYZI());
            {
                const bool odin_pub_have = odin_body_pub_en &&
                    (((odin_map_full && !odin_map_full->empty()) ||
                      (odin_display_only && !odin_display_only->empty())));
                const auto &src = dense_pub_en ? *feats_undistort : *feats_down_body;
                pub_cloud->points.reserve(
                    src.size() +
                    (odin_pub_have && odin_map_full ? odin_map_full->size() : 0) +
                    (odin_pub_have && odin_display_only ? odin_display_only->size() : 0));
                for (const auto &pt : src.points)
                {
                    if (pt.intensity >= 150.0f) continue;  // drop IEKF subsample
                    pub_cloud->points.push_back(pt);
                }
                if (odin_pub_have && odin_map_full && !odin_map_full->empty())
                    pub_cloud->points.insert(pub_cloud->points.end(),
                                            odin_map_full->points.begin(),
                                            odin_map_full->points.end());
                else if (odin_pub_have && odin_display_only &&
                         !odin_display_only->empty())
                    pub_cloud->points.insert(pub_cloud->points.end(),
                                            odin_display_only->points.begin(),
                                            odin_display_only->points.end());
                pub_cloud->width = pub_cloud->size();
                pub_cloud->height = 1;
            }
            if (scan_pub_en)
                publish_frame_world(pubLaserCloudFull, pub_cloud);
            if (scan_pub_en && scan_body_pub_en)
                publish_frame_body(pubLaserCloudFull_body, pub_cloud);
            if (publish_voxel_map) {
                pubVoxelMap(voxel_map, publish_max_voxel_layer, voxel_map_pub);
            }
            // if (publish_global_cloud_map) {

            // }
        }

        status = rclcpp::ok();
        rate.sleep();
    }

    if(!has_save_opt && pcd_save_en) {
        saveMapToFile(pcd_save_path);
    }

    for(auto& vp: voxel_map) {
        if(vp.second != nullptr) {
            delete vp.second;
            vp.second = nullptr;
        }
    }

    rclcpp::shutdown();
    return 0;
}
