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
// INTERRUPTION) HOWEVER CA)USED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/// c++ headers
#include <algorithm>
#include <mutex>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <condition_variable>
/// module headers
#include <omp.h>
/// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
/// ros
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <livox_ros_driver2/msg/custom_msg.hpp>
/// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h> //transformPointCloud
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
/// this package
#include "so3_math.h"
#include "IMU_Processing.hpp"
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include <chrono>
#include <std_msgs/msg/float32.hpp>
using namespace std::chrono;

#define LASER_POINT_COV     (0.001)

/**************************/
bool pcd_save_en = false, extrinsic_est_en = true, path_en = true;
float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

string lid_topic, lid_topic2, imu_topic, map_frame = "map";
bool multi_lidar = false, async_debug = false, publish_tf_results = false, publish_odom_tf = true, bundle_enabled = false;
// Smooth map→body TF for RViz only; /Odometry stays raw IEKF pose.
double tf_pose_ema = 0.15;
bool tf_ema_inited = false;
Eigen::Vector3d tf_ema_pos = Eigen::Vector3d::Zero();
Eigen::Quaterniond tf_ema_q = Eigen::Quaterniond::Identity();
bool extrinsic_imu_to_lidars = false;

int voxelized_pt_num_thres = 100, bundle_enabled_tic = 10000, bundle_enabled_tic_thres = 10;
size_t bundle_pair_index_ = 0;
double bundle_pair_dt_ = 0.0;
double effect_pt_num_ratio_thres = 0.4;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_lidar2 = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_surf = 0;
double cube_len = 0, lidar_end_time = 0, lidar_end_time2 = 0, first_lidar_time = 0.0;
double publish_lidar_time = 0.0;
int    effect_feat_num = 0;
int    feats_down_size = 0, NUM_MAX_ITERATIONS = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed = false, flg_exit = false;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points;
// temporal variables
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
vector<double> extrinT2(3, 0.0);
vector<double> extrinR2(9, 0.0);
vector<double> extrinT3(3, 0.0);
vector<double> extrinR3(9, 0.0);
vector<double> extrinT4(3, 0.0);
vector<double> extrinR4(9, 0.0);
deque<double>                     time_buffer;
deque<double>                     time_buffer2;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer2;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;
mutex mtx_buffer;
mutex mtx_state; // Mid pose snapshot for Odin callback (timer vs sub threads)
condition_variable sig_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

pcl::VoxelGrid<PointType> downSizeFilterSurf;
KD_TREE<PointType> ikdtree;
shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

Eigen::Matrix4d LiDAR2_wrt_LiDAR1 = Eigen::Matrix4d::Identity();
// Yaml level-frame R (Odin // gravity). Mid mount pitch is cancelled per-frame, not baked.
M3D LiDAR2_R_yaml = Eye3d;
Eigen::Matrix4d LiDAR1_wrt_drone = Eigen::Matrix4d::Identity();

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
bool first_lidar_scan_check = false;
bool lidar1_ikd_init = false, lidar2_ikd_init = false;
bool odin_gravity_ext_comp = false; // log once when level-Odin publish path is active
bool extrinsic_L2_gravity_frame = false; // true: pose Odin with Mid yaw+pos only (level cloud)
// false: Odin only densifies publish cloud; Mid360 alone drives IEKF + ikdtree.
// Bad L2 extrinsic otherwise corrupts pose/map as soon as the robot moves.
bool lidar2_update_en = false;
// Point residuals from Odin into IEKF (can smear Mid). Prefer pose-factor instead.
bool lidar2_point_update_en = false;
// Soft relative-pose tight couple from /odin1/odometry (scheme ②).
bool lidar2_pose_factor_en = true;
double lidar2_pose_alpha = 0.12;   // blend weight on relative innov [0,1]
double lidar2_pose_max_dt = 0.20;  // s vs Mid scan time
std::string odin_odom_topic = "/odin1/odometry";
std::mutex mtx_odin_odom;
double odin_odom_stamp = -1.0;
Eigen::Vector3d odin_odom_pos = Eigen::Vector3d::Zero();
Eigen::Matrix3d odin_odom_rot = Eigen::Matrix3d::Identity();
bool odin_odom_have = false;
bool odin_pose_align_init = false;
double odin_yaw_align = 0.0; // Mid_yaw - Odin_yaw at lock
Eigen::Vector3d last_pf_mid_pos = Eigen::Vector3d::Zero();
Eigen::Vector3d last_pf_odin_pos = Eigen::Vector3d::Zero();
double last_pf_mid_yaw = 0.0;
double last_pf_odin_yaw = 0.0;
bool last_pf_valid = false;
long long n_odin_pose_factor = 0;
// During fast yaw, cloud_slam L2 residuals pull Mid pose → /cloud_registered
// smears for a few seconds. Gate Odin IEKF/map by IMU |gyro|.
double last_imu_gyr_norm = 0.0;
double last_imu_gyr_pitch = 0.0; // |ωy| — stairs pitch rate (yaw-norm alone misses this)
double last_mid_vz = 0.0;
double last_mid_pitch = 0.0; // rad, from Mid IEKF rot
double lidar2_pose_gyr_gate = 0.35; // rad/s; above => Odin densify-off + no pose update
double lidar2_pitch_gyr_gate = 0.20; // rad/s |ωy|; stairs / slope
double lidar2_vz_gate = 0.12;       // m/s |vz|; climbing
double lidar2_pitch_gate_deg = 10.0; // |pitch| of Mid state
// Reject L2 IEKF/bundle pair when |t_L2 - t_Mid| exceeds this (TCP lag gate).
double lidar2_max_sync_dt = 0.08;
// Soft Odin IEKF: Mid updates first at LASER_POINT_COV, then Odin at cov*scale.
double lidar2_residual_cov_scale = 25.0;
// Reject Odin point-to-plane residuals larger than this (meters) in Stage2.
double lidar2_max_residual = 0.12;
// If false: Odin still updates IEKF (tight couple) but never enters ikdtree.
bool lidar2_map_insert_en = false;
// True only after a successful Stage2 Odin update this frame (gates map insert).
bool last_odin_iekf_ok = false;
long long n_skip_l2_sync = 0;
long long n_odin_iekf_updates = 0;
long long n_odin_hold_stairs = 0;
long long n_odin_res_clip = 0;
// Mid-only Odin viz: buffer latest body cloud; pose with the SAME Mid
// state_point right after IEKF. Stamp-based lookup failed in practice:
// /cloud_odin_registered stamps lagged /Odometry by ~5s → old yaw × new body
// walls → orange accum rotated while Mid stayed fixed.
PointCloudXYZI::Ptr odin_pending_body;
bool odin_pending_ready = false;
bool odin_mid_pose_ready = false;
constexpr float kOdinIntensityTag = 100.0f;
// 0=unused, 1=Mid residuals only, 2=Odin residuals only (two-stage tight couple)
int residual_select = 1;

inline double mid_pitch_from_rot(const M3D &R)
{
    // R: body→map; pitch ≈ asin(-R(2,0)) for Z-up, X-forward.
    const double s = -R(2, 0);
    return std::asin(std::max(-1.0, std::min(1.0, s)));
}

inline double yaw_from_rot(const M3D &R)
{
    return std::atan2(R(1, 0), R(0, 0));
}

inline double wrap_pi(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

void odin_odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    const auto &p = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;
    Eigen::Quaterniond qq(q.w, q.x, q.y, q.z);
    if (qq.norm() < 1e-9)
        return;
    qq.normalize();
    std::lock_guard<std::mutex> lk(mtx_odin_odom);
    odin_odom_stamp = rclcpp::Time(msg->header.stamp).seconds();
    odin_odom_pos << p.x, p.y, p.z;
    odin_odom_rot = qq.toRotationMatrix();
    odin_odom_have = true;
}

// Scheme ②: Mid owns point IEKF; Odin contributes a soft relative-pose factor.
void apply_odin_pose_factor()
{
    if (!lidar2_pose_factor_en)
        return;
    double t_o = -1.0;
    Eigen::Vector3d p_o = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_o = Eigen::Matrix3d::Identity();
    {
        std::lock_guard<std::mutex> lk(mtx_odin_odom);
        if (!odin_odom_have)
            return;
        t_o = odin_odom_stamp;
        p_o = odin_odom_pos;
        R_o = odin_odom_rot;
    }
    if (lidar2_pose_max_dt > 0.0 &&
        std::fabs(t_o - publish_lidar_time) > lidar2_pose_max_dt)
        return;

    state_ikfom s = kf.get_x();
    const M3D R_m = s.rot.toRotationMatrix();
    const double yaw_m = yaw_from_rot(R_m);
    const double yaw_o = yaw_from_rot(R_o);

    if (!odin_pose_align_init)
    {
        odin_yaw_align = wrap_pi(yaw_m - yaw_o);
        odin_pose_align_init = true;
        last_pf_mid_pos = s.pos;
        last_pf_odin_pos = p_o;
        last_pf_mid_yaw = yaw_m;
        last_pf_odin_yaw = wrap_pi(yaw_o + odin_yaw_align);
        last_pf_valid = true;
        std::cout << "\033[32;1mOdin pose-factor: yaw_align="
                  << odin_yaw_align * 180.0 / M_PI << " deg\033[0m" << std::endl;
        return;
    }
    if (!last_pf_valid)
        return;

    const Eigen::Vector3d dp_m = s.pos - last_pf_mid_pos;
    const Eigen::Vector3d dp_o_src = p_o - last_pf_odin_pos;
    const double ca = std::cos(odin_yaw_align);
    const double sa = std::sin(odin_yaw_align);
    const Eigen::Vector3d dp_o(ca * dp_o_src(0) - sa * dp_o_src(1),
                               sa * dp_o_src(0) + ca * dp_o_src(1),
                               dp_o_src(2));
    Eigen::Vector3d innov = dp_o - dp_m;
    // Drop TCP/glitch jumps — do not yank Mid.
    if (innov.norm() > 0.35)
    {
        last_pf_mid_pos = s.pos;
        last_pf_odin_pos = p_o;
        last_pf_mid_yaw = yaw_m;
        last_pf_odin_yaw = wrap_pi(yaw_o + odin_yaw_align);
        return;
    }

    const double a = std::max(0.0, std::min(1.0, lidar2_pose_alpha));
    s.pos += a * innov;

    const double dy_o = wrap_pi(wrap_pi(yaw_o + odin_yaw_align) - last_pf_odin_yaw);
    const double dy_m = wrap_pi(yaw_m - last_pf_mid_yaw);
    const double innov_yaw = wrap_pi(dy_o - dy_m);
    if (std::fabs(innov_yaw) < 0.25)
    {
        const Eigen::AngleAxisd aa(a * innov_yaw, Eigen::Vector3d::UnitZ());
        const Eigen::Matrix3d R_new = aa.toRotationMatrix() * R_m;
        s.rot = SO3(Eigen::Quaterniond(R_new));
    }

    kf.change_x(s);
    n_odin_pose_factor++;

    state_point = s;
    last_pf_mid_pos = s.pos;
    last_pf_odin_pos = p_o;
    last_pf_mid_yaw = yaw_from_rot(s.rot.toRotationMatrix());
    last_pf_odin_yaw = wrap_pi(yaw_o + odin_yaw_align);
}

inline bool odin_pose_update_allowed()
{
    // Full-time tight couple: gates <=0 disable holds (stairs/turns still update).
    // Optional gates remain for experiments; production yaml sets all to 0.
    if (!lidar2_update_en)
        return false;
    if (lidar2_pose_gyr_gate > 0.0 && last_imu_gyr_norm >= lidar2_pose_gyr_gate)
        return false;
    if (lidar2_pitch_gyr_gate > 0.0 && last_imu_gyr_pitch >= lidar2_pitch_gyr_gate)
        return false;
    if (lidar2_vz_gate > 0.0 && std::fabs(last_mid_vz) >= lidar2_vz_gate)
        return false;
    if (lidar2_pitch_gate_deg > 0.0 &&
        std::fabs(last_mid_pitch) * 180.0 / M_PI >= lidar2_pitch_gate_deg)
        return false;
    return true;
}

// Never skip Stage2; inflate Odin cov under fast motion / climb so Mid stays stable.
inline double odin_cov_scale_now()
{
    double s = std::max(1.0, lidar2_residual_cov_scale);
    // Mild motion already weakens Odin — full-time couple was smearing Mid at scale=25.
    if (last_imu_gyr_norm >= 0.20)
        s *= 1.5;
    if (last_imu_gyr_norm >= 0.35)
        s *= 2.0;
    if (last_imu_gyr_pitch >= 0.12)
        s *= 1.5;
    if (last_imu_gyr_pitch >= 0.20)
        s *= 1.5;
    if (std::fabs(last_mid_vz) >= 0.08)
        s *= 1.5;
    if (std::fabs(last_mid_pitch) * 180.0 / M_PI >= 6.0)
        s *= 1.5;
    return s;
}

// Gravity-leveled cloud_slam: apply CAD translation only for IEKF (no Ry(-13°)),
// same rationale as viz yaw-align — remount pitch re-tilts already-level geometry.
inline Eigen::Matrix4d T_L2_for_iekf()
{
    if (!extrinsic_L2_gravity_frame)
        return LiDAR2_wrt_LiDAR1;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 1>(0, 3) = LiDAR2_wrt_LiDAR1.block<3, 1>(0, 3);
    return T;
}
int current_lidar_num = 1;
double lidar_mean_scantime = 0.0;
double lidar_mean_scantime2 = 0.0;
int    scan_num = 0;
int    scan_num2 = 0;
Eigen::Vector3d localizability_vec = Eigen::Vector3d::Zero();

void SigHandle(int sig)
{
    flg_exit = true;
    std::cout << "catch sig " << sig << std::endl;
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

// Forward decl: used by Mid-only Odin callback before definition.
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_odin_registered;
void publish_odin_body_to_world(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubOdin,
    const PointCloudXYZI::Ptr &odin_body_mid,
    double stamp_sec);
void publish_odin_level_to_map(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubOdin,
    const PointCloudXYZI::Ptr &odin_lidar,
    double stamp_sec,
    const M3D &R_mid,
    const V3D &t_mid);
void flush_pending_odin_to_map(double stamp_sec);

// Mid yaw-only rotation from a body->world matrix (stable; no Euler pitch/roll).
static M3D mid_yaw_rotation(const M3D &R_wi)
{
    // yaw from first column projected to XY (same as atan2(R10, R00) for ZYX).
    const double yaw = std::atan2(R_wi(1, 0), R_wi(0, 0));
    const double c = std::cos(yaw), s = std::sin(yaw);
    M3D R_yaw = Eye3d;
    R_yaw << c, -s, 0, s, c, 0, 0, 0, 1;
    return R_yaw;
}

void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
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
}

void lasermap_fov_segment()
{
    cub_needrm.clear();
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    if(cub_needrm.size() > 0) ikdtree.Delete_Point_Boxes(cub_needrm);
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
    mtx_buffer.lock();
        if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr, 0);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(rclcpp::Time(msg->header.stamp).seconds());
    last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    first_lidar_scan_check = true;
}

void standard_pcl_cbk2(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
    // Mid-only IEKF: only buffer latest Odin body cloud; pose in Mid timer with
    // the same state_point as /cloud_registered (avoids stale stamp × new body).
    if (!lidar2_update_en)
    {
        PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr, 1);
        if (!ptr->empty())
        {
            for (auto &pt : ptr->points)
                pt.intensity = 200.0f;
            std::lock_guard<std::mutex> lk(mtx_state);
            odin_pending_body = ptr;
            odin_pending_ready = true;
        }
        return;
    }

    mtx_buffer.lock();
    if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar2)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer2.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr, 1);
    // Odin points are not guaranteed time-ordered; sync uses points.back().curvature.
    if (!ptr->empty())
        std::sort(ptr->points.begin(), ptr->points.end(),
                  [](const PointType &a, const PointType &b) { return a.curvature < b.curvature; });
    ptr->header.seq = 1; //trick
    lidar_buffer2.push_back(ptr);
    time_buffer2.push_back(rclcpp::Time(msg->header.stamp).seconds());
    last_timestamp_lidar2 = rclcpp::Time(msg->header.stamp).seconds();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    first_lidar_scan_check = true;
}

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
{
    mtx_buffer.lock();
    if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr, 0);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    first_lidar_scan_check = true;
}

void livox_pcl_cbk2(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) 
{
    mtx_buffer.lock();
    if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar2)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer2.clear();
    }
    last_timestamp_lidar2 = rclcpp::Time(msg->header.stamp).seconds();
    
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr, 1);
    ptr->header.seq = 1; //trick
    lidar_buffer2.push_back(ptr);
    time_buffer2.push_back(last_timestamp_lidar2);
    
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    first_lidar_scan_check = true;
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in) 
{
    if (!first_lidar_scan_check) return; //note prevent only IMU input is stacked and then filter is too much forward propagated

    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
    double timestamp = rclcpp::Time(msg->header.stamp).seconds();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        std::cerr << "imu loop back, clear buffer" << std::endl;
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    {
        const double gx = msg->angular_velocity.x;
        const double gy = msg->angular_velocity.y;
        const double gz = msg->angular_velocity.z;
        last_imu_gyr_norm = std::sqrt(gx * gx + gy * gy + gz * gz);
        last_imu_gyr_pitch = std::fabs(gy);
    }

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

bool sync_packages_async(MeasureGroup &meas)
{
    // Mid-only: never pull Odin into IEKF (Odin is published from cbk2 via L2 TF).
    if (!lidar2_update_en)
    {
        lidar_buffer2.clear();
        time_buffer2.clear();
    }

    if ( (lidar_buffer.empty() && lidar_buffer2.empty()) || imu_buffer.empty()) {
        return false;
    }

    int lidar_use_num_now_ = 0;
    if (!lidar_buffer.empty() && !lidar_buffer2.empty())
    {
        // Async path is Mid-only for IEKF. Odin couples only via bundle.
        // Drop Odin frames older than Mid front so L2 cannot starve Mid.
        while (!time_buffer2.empty() &&
               time_buffer2.front() + 1e-3 < time_buffer.front())
        {
            lidar_buffer2.pop_front();
            time_buffer2.pop_front();
            n_skip_l2_sync++;
        }
        lidar_use_num_now_ = 1;
    }
    else
    {
        if (!lidar_buffer.empty())
        {
            lidar_use_num_now_ = 1;
        }
        else if (!lidar_buffer2.empty())
        {
            // Discard Odin-only async packages (would IMU-prop Mid to L2 time).
            lidar_buffer2.pop_front();
            time_buffer2.pop_front();
            n_skip_l2_sync++;
            return false;
        }
    }
    if (lidar_use_num_now_ == 0)
    {
        std::cerr << "lidar_use_num_now_ is 0, error!" << std::endl;
        return false;
    }
    else if (lidar_use_num_now_ == 1)
    {
        /*** push a lidar scan ***/
        if(!lidar_pushed)
        {
            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            if (async_debug) cout << "\033[31;1mFirst LiDAR!, " << meas.lidar->points.size() << "\033[0m" << endl;
            current_lidar_num = 1;

            if (meas.lidar->points.size() <= 1) // time too little
            {
                lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
                std::cerr << "Too few input point cloud!\n";
            }
            else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
            {
                lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            }
            else
            {
                scan_num ++;
                lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
                lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
            }

            meas.lidar_end_time = lidar_end_time;
            publish_lidar_time = lidar_end_time;
            lidar_pushed = true;
        }

        if (last_timestamp_imu < lidar_end_time)
        {
            return false;
        }

        /*** push imu data, and pop from imu buffer ***/
        double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        meas.imu.clear();
        while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
        {
            imu_time = get_time_sec(imu_buffer.front()->header.stamp);
            if(imu_time > lidar_end_time) break;
            meas.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }

        lidar_buffer.pop_front();
        time_buffer.pop_front();
        lidar_pushed = false;
        return true;
    }
    else if (lidar_use_num_now_ == 2)
    {
        /*** push a lidar scan ***/
        if(!lidar_pushed)
        {
            // Gate async L2-only updates: require a Mid stamp within sync dt,
            // else drop this Odin frame (TCP lag / restamp mismatch).
            if (lidar2_max_sync_dt > 0.0)
            {
                const double t2 = time_buffer2.front();
                double best_dt = 1e9;
                if (!time_buffer.empty())
                {
                    for (double t1 : time_buffer)
                        best_dt = std::min(best_dt, std::fabs(t1 - t2));
                }
                else if (last_timestamp_lidar > 0.0)
                    best_dt = std::fabs(last_timestamp_lidar - t2);
                if (best_dt > lidar2_max_sync_dt)
                {
                    n_skip_l2_sync++;
                    lidar_buffer2.pop_front();
                    time_buffer2.pop_front();
                    return false;
                }
            }
            meas.lidar2 = lidar_buffer2.front();
            meas.lidar_beg_time2 = time_buffer2.front();
            pcl::transformPointCloud(*meas.lidar2, *meas.lidar2, T_L2_for_iekf());
            if (async_debug) cout << "\033[32;1mSecond LiDAR!, " << meas.lidar2->points.size() << "\033[0m" << endl;
            current_lidar_num = 2;

            if (meas.lidar2->points.size() <= 1) // time too little
            {
                lidar_end_time2 = meas.lidar_beg_time2 + lidar_mean_scantime2;
                std::cerr << "Too few input point cloud!\n";
            }
            else if (meas.lidar2->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime2)
            {
                lidar_end_time2 = meas.lidar_beg_time2 + lidar_mean_scantime2;
            }
            else
            {
                scan_num2 ++;
                lidar_end_time2 = meas.lidar_beg_time2 + meas.lidar2->points.back().curvature / double(1000);
                lidar_mean_scantime2 += (meas.lidar2->points.back().curvature / double(1000) - lidar_mean_scantime2) / scan_num2;
            }

            meas.lidar_end_time2 = lidar_end_time2;
            publish_lidar_time = lidar_end_time2;
            lidar_pushed = true;
        }

        if (last_timestamp_imu < lidar_end_time2)
        {
            return false;
        }

        /*** push imu data, and pop from imu buffer ***/
        double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        meas.imu.clear();
        while ((!imu_buffer.empty()) && (imu_time < lidar_end_time2))
        {
            imu_time = get_time_sec(imu_buffer.front()->header.stamp);
            if(imu_time > lidar_end_time2) break;
            meas.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }

        lidar_buffer2.pop_front();
        time_buffer2.pop_front();
        lidar_pushed = false;
        return true;
    }
    return false;
}
bool sync_packages_bundle(MeasureGroup &meas)
{
    if (multi_lidar)
    {
        if (lidar_buffer.empty() || lidar_buffer2.empty() || imu_buffer.empty()) {
            return false;
        }
        /*** push a lidar scan ***/
        if(!lidar_pushed)
        {
            // Pair Mid front with nearest Odin by stamp (do NOT merge whole buffers —
            // that mixed scans from different poses and ghosted orange accum).
            const double max_bundle_dt =
                (lidar2_max_sync_dt > 0.0) ? lidar2_max_sync_dt : 0.08;
            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            if (meas.lidar->points.size() <= 1) // time too little
            {
                lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
                std::cerr << "Too few input point cloud!\n";
            }
            else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
            {
                lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            }
            else
            {
                scan_num ++;
                lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
                lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
            }
            meas.lidar_end_time = lidar_end_time;

            size_t best_i = 0;
            double best_dt = std::fabs(time_buffer2.front() - meas.lidar_beg_time);
            for (size_t i = 1; i < time_buffer2.size(); i++)
            {
                const double dt = std::fabs(time_buffer2[i] - meas.lidar_beg_time);
                if (dt < best_dt)
                {
                    best_dt = dt;
                    best_i = i;
                }
            }
            if (best_dt > max_bundle_dt)
            {
                n_skip_l2_sync++;
                // Drop the older stream head so the next tick can re-pair.
                if (time_buffer.front() < time_buffer2.front())
                {
                    lidar_buffer.pop_front();
                    time_buffer.pop_front();
                }
                else
                {
                    lidar_buffer2.pop_front();
                    time_buffer2.pop_front();
                }
                return false;
            }

            meas.lidar2.reset(new PointCloudXYZI(*lidar_buffer2[best_i]));
            // L2 → Mid lidar frame for IEKF (gravity_frame: translation-only).
            pcl::transformPointCloud(*meas.lidar2, *meas.lidar2, T_L2_for_iekf());
            meas.lidar_beg_time2 = time_buffer2[best_i];
            if (meas.lidar2->points.size() <= 1) // time too little
            {
                lidar_end_time2 = meas.lidar_beg_time2 + lidar_mean_scantime2;
                std::cerr << "Too few input point cloud!\n";
            }
            else if (meas.lidar2->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime2)
            {
                lidar_end_time2 = meas.lidar_beg_time2 + lidar_mean_scantime2;
            }
            else
            {
                scan_num2 ++;
                lidar_end_time2 = meas.lidar_beg_time2 + meas.lidar2->points.back().curvature / double(1000);
                lidar_mean_scantime2 += (meas.lidar2->points.back().curvature / double(1000) - lidar_mean_scantime2) / scan_num2;
            }
            meas.lidar_end_time2 = lidar_end_time2;

            publish_lidar_time = max(lidar_end_time, lidar_end_time2);
            lidar_pushed = true;
            bundle_pair_index_ = best_i; // remember which L2 to pop after IMU sync
            bundle_pair_dt_ = best_dt;
        }

        if (last_timestamp_imu < lidar_end_time || last_timestamp_imu < lidar_end_time2)
        {
            return false;
        }

        /*** push imu data, and pop from imu buffer ***/
        double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        meas.imu.clear();
        while ((!imu_buffer.empty()) && (imu_time < lidar_end_time || imu_time < lidar_end_time2))
        {
            imu_time = get_time_sec(imu_buffer.front()->header.stamp);
            if(imu_time > lidar_end_time && imu_time > lidar_end_time2) break;
            meas.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }

        lidar_buffer.pop_front();
        time_buffer.pop_front();
        // Drop Odin frames up to and including the paired one.
        for (size_t i = 0; i <= bundle_pair_index_ && !lidar_buffer2.empty(); i++)
        {
            lidar_buffer2.pop_front();
            time_buffer2.pop_front();
        }

        lidar_pushed = false;
        cout << "\033[36;1mBundle update! dt=" << bundle_pair_dt_ * 1000.0 << "ms\033[0m" << endl;
        return true;
    }
    else
    {
        if (lidar_buffer.empty() || imu_buffer.empty()) {
            return false;
        }
        /*** push a lidar scan ***/
        if(!lidar_pushed)
        {
            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            if (meas.lidar->points.size() <= 1) // time too little
            {
                lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
                std::cerr << "Too few input point cloud!\n";
            }
            else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
            {
                lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            }
            else
            {
                scan_num ++;
                lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
                lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
            }

            meas.lidar_end_time = lidar_end_time;
            publish_lidar_time = lidar_end_time;
            lidar_pushed = true;
        }

        if (last_timestamp_imu < lidar_end_time)
        {
            return false;
        }

        /*** push imu data, and pop from imu buffer ***/
        double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        meas.imu.clear();
        while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
        {
            imu_time = get_time_sec(imu_buffer.front()->header.stamp);
            if(imu_time > lidar_end_time) break;
            meas.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }

        lidar_buffer.pop_front();
        time_buffer.pop_front();

        lidar_pushed = false;
        return true;     
    }
}

void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    const float det_r2 = DET_RANGE * DET_RANGE;
    for (int i = 0; i < feats_down_size; i++)
    {
        const bool is_odin_pt =
            feats_down_body->points[i].intensity >= kOdinIntensityTag ||
            (!bundle_enabled && current_lidar_num == 2);
        // Tight-couple pose via residuals, but keep Mid map clean unless enabled.
        if (is_odin_pt &&
            (!lidar2_map_insert_en || !odin_pose_update_allowed() || !last_odin_iekf_ok))
            continue;
        const float bx = feats_down_body->points[i].x;
        const float by = feats_down_body->points[i].y;
        const float bz = feats_down_body->points[i].z;
        if ((bx * bx + by * by + bz * bz) > det_r2)
            continue;
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty())
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_surf)*filter_size_surf + 0.5 * filter_size_surf;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_surf)*filter_size_surf + 0.5 * filter_size_surf;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_surf)*filter_size_surf + 0.5 * filter_size_surf;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_surf && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_surf && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_surf){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }
    ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    return;
}


void publish_odin_body_to_world(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubOdin,
    const PointCloudXYZI::Ptr &odin_body_mid,
    double stamp_sec)
{
    if (!pubOdin || !odin_body_mid || odin_body_mid->empty() || !scan_pub_en)
        return;
    if (!p_imu->IMU_inited())
        return;
    const int size = static_cast<int>(odin_body_mid->size());
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++)
        RGBpointBodyToWorld(&odin_body_mid->points[i], &laserCloudWorld->points[i]);
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(stamp_sec);
    laserCloudmsg.header.frame_id = map_frame;
    pubOdin->publish(laserCloudmsg);
}

// Level Odin cloud -> map using Mid yaw + position only (no Mid pitch/roll).
// R_mid/t_mid must be the Mid state used for this publish (same as Mid scan).
// p_map = R_yaw * (R_yaml * p + t_L2) + t_mid
void publish_odin_level_to_map(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubOdin,
    const PointCloudXYZI::Ptr &odin_lidar,
    double stamp_sec,
    const M3D &R_mid,
    const V3D &t_mid)
{
    if (!pubOdin || !odin_lidar || odin_lidar->empty() || !scan_pub_en)
        return;
    if (!p_imu || !p_imu->IMU_inited())
        return;

    const M3D R_yaw = mid_yaw_rotation(R_mid);
    const M3D R_yaml = LiDAR2_R_yaml;
    const V3D t_L2 = LiDAR2_wrt_LiDAR1.block<3, 1>(0, 3);

    const int size = static_cast<int>(odin_lidar->size());
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++)
    {
        const auto &pi = odin_lidar->points[i];
        V3D p_l(pi.x, pi.y, pi.z);
        V3D p_m = R_yaw * (R_yaml * p_l + t_L2) + t_mid;
        auto &po = laserCloudWorld->points[i];
        po.x = p_m(0);
        po.y = p_m(1);
        po.z = p_m(2);
        po.intensity = pi.intensity;
    }
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(stamp_sec);
    laserCloudmsg.header.frame_id = map_frame;
    pubOdin->publish(laserCloudmsg);
}

void flush_pending_odin_to_map(double stamp_sec)
{
    if (!multi_lidar || lidar2_update_en || !scan_pub_en)
        return;
    if (!p_imu || !p_imu->IMU_inited())
        return;

    PointCloudXYZI::Ptr ptr;
    {
        std::lock_guard<std::mutex> lk(mtx_state);
        if (!odin_pending_ready || !odin_pending_body || odin_pending_body->empty())
            return;
        ptr = odin_pending_body;
        odin_pending_ready = false;
        odin_pending_body.reset();
    }

    const M3D R_mid = state_point.rot.toRotationMatrix();
    const V3D t_mid = state_point.pos;
    odin_mid_pose_ready = true;

    if (extrinsic_L2_gravity_frame)
        publish_odin_level_to_map(g_pub_odin_registered, ptr, stamp_sec, R_mid, t_mid);
    else
    {
        PointCloudXYZI::Ptr body(new PointCloudXYZI());
        pcl::transformPointCloud(*ptr, *body, LiDAR2_wrt_LiDAR1);
        // Temporarily set state already is current Mid state for RGBpointBodyToWorld.
        publish_odin_body_to_world(g_pub_odin_registered, body, stamp_sec);
    }
}

void publish_odin_from_undistorted(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubOdin,
    const PointCloudXYZI::Ptr &undistorted)
{
    // Publish Odin points already undistorted into Mid body, using current state_point.
    if (!pubOdin || !undistorted || undistorted->empty() || !scan_pub_en)
        return;
    PointCloudXYZI::Ptr odin_body(new PointCloudXYZI());
    odin_body->reserve(undistorted->size());
    for (const auto &pt : undistorted->points)
    {
        if (pt.intensity >= kOdinIntensityTag)
            odin_body->push_back(pt);
    }
    // Async lidar2 whole-frame: intensity may already be tagged; if not, take all.
    if (odin_body->empty() && current_lidar_num == 2)
        *odin_body = *undistorted;
    if (odin_body->empty())
        return;
    publish_odin_body_to_world(pubOdin, odin_body, publish_lidar_time);
}

void publish_mid_from_undistorted(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubMid,
    const PointCloudXYZI::Ptr &undistorted)
{
    if (!pubMid || !undistorted || undistorted->empty() || !scan_pub_en)
        return;
    PointCloudXYZI::Ptr mid_body(new PointCloudXYZI());
    mid_body->reserve(undistorted->size());
    for (const auto &pt : undistorted->points)
    {
        if (pt.intensity < kOdinIntensityTag)
            mid_body->push_back(pt);
    }
    if (mid_body->empty())
        return;
    const int size = static_cast<int>(mid_body->size());
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++)
        RGBpointBodyToWorld(&mid_body->points[i], &laserCloudWorld->points[i]);
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(publish_lidar_time);
    laserCloudmsg.header.frame_id = map_frame;
    pubMid->publish(laserCloudmsg);
}

void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFull, const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullTransFormed)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = get_ros_time(publish_lidar_time);
        laserCloudmsg.header.frame_id = map_frame;
        pubLaserCloudFull->publish(laserCloudmsg);
        
        if (publish_tf_results)
        {
            PointCloudXYZI::Ptr laserCloudWorldTransFormed(new PointCloudXYZI(size, 1));
            pcl::transformPointCloud(*laserCloudWorld, *laserCloudWorldTransFormed, LiDAR1_wrt_drone);
            sensor_msgs::msg::PointCloud2 laserCloudmsg2;
            pcl::toROSMsg(*laserCloudWorldTransFormed, laserCloudmsg2);
            laserCloudmsg2.header.stamp = get_ros_time(publish_lidar_time);
            laserCloudmsg2.header.frame_id = map_frame;
            pubLaserCloudFullTransFormed->publish(laserCloudmsg2);
        }
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(publish_lidar_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body->publish(laserCloudmsg);
}

void publish_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudMap)
{
    sensor_msgs::msg::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = get_ros_time(publish_lidar_time);
    laserCloudMap.header.frame_id = map_frame;
    pubLaserCloudMap->publish(laserCloudMap);
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
    return;
}

void publish_visionpose(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &publisher)
{
    geometry_msgs::msg::PoseStamped msg_out_;
    msg_out_.header.frame_id = map_frame;
    msg_out_.header.stamp = get_ros_time(publish_lidar_time);

    Eigen::Matrix4d current_pose_eig_ = Eigen::Matrix4d::Identity();
    current_pose_eig_.block<3, 3>(0, 0) = state_point.rot.toRotationMatrix();
    current_pose_eig_.block<3, 1>(0, 3) = state_point.pos;
    Eigen::Matrix4d tfed_vision_pose_eig_ = LiDAR1_wrt_drone * current_pose_eig_ * LiDAR1_wrt_drone.inverse(); //note
    msg_out_.pose.position.x = tfed_vision_pose_eig_(0, 3);
    msg_out_.pose.position.y = tfed_vision_pose_eig_(1, 3);
    msg_out_.pose.position.z = tfed_vision_pose_eig_(2, 3);
    Eigen::Quaterniond tfed_quat_(tfed_vision_pose_eig_.block<3, 3>(0, 0));
    msg_out_.pose.orientation.x = tfed_quat_.x();
    msg_out_.pose.orientation.y = tfed_quat_.y();
    msg_out_.pose.orientation.z = tfed_quat_.z();
    msg_out_.pose.orientation.w = tfed_quat_.w();
    publisher->publish(msg_out_);
    return;
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> &tf_br)
{
    odomAftMapped.header.frame_id = map_frame;
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = get_ros_time(publish_lidar_time);
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

    // B2: when Odin owns system TF (map→body), Mid must not broadcast the same frame.
    if (publish_odom_tf)
    {
        Eigen::Vector3d p(
            odomAftMapped.pose.pose.position.x,
            odomAftMapped.pose.pose.position.y,
            odomAftMapped.pose.pose.position.z);
        Eigen::Quaterniond q(
            odomAftMapped.pose.pose.orientation.w,
            odomAftMapped.pose.pose.orientation.x,
            odomAftMapped.pose.pose.orientation.y,
            odomAftMapped.pose.pose.orientation.z);
        q.normalize();
        if (!tf_ema_inited || tf_pose_ema >= 0.999)
        {
            tf_ema_pos = p;
            tf_ema_q = q;
            tf_ema_inited = true;
        }
        else
        {
            const double dp = (p - tf_ema_pos).norm();
            const double dang_deg = tf_ema_q.angularDistance(q) * (180.0 / M_PI);
            // Deadband: while robot is still, IEKF still wanders ~mm — do NOT
            // chase it or map→body / base_link shake in RViz.
            if (dp < 0.012 && dang_deg < 0.40)
            {
                // hold tf_ema_*
            }
            else
            {
                // Track when clearly moving; heavy damp on small motions.
                double a = tf_pose_ema;
                if (dp > 0.03 || dang_deg > 0.8)
                    a = std::max(tf_pose_ema, 0.65);
                else
                    a = std::min(tf_pose_ema, 0.08);
                a = std::clamp(a, 0.01, 1.0);
                tf_ema_pos = (1.0 - a) * tf_ema_pos + a * p;
                tf_ema_q = tf_ema_q.slerp(a, q).normalized();
            }
        }

        geometry_msgs::msg::TransformStamped trans;
        trans.header.frame_id = map_frame;
        trans.header.stamp = odomAftMapped.header.stamp;
        trans.child_frame_id = "body";
        trans.transform.translation.x = tf_ema_pos.x();
        trans.transform.translation.y = tf_ema_pos.y();
        trans.transform.translation.z = tf_ema_pos.z();
        trans.transform.rotation.w = tf_ema_q.w();
        trans.transform.rotation.x = tf_ema_q.x();
        trans.transform.rotation.y = tf_ema_q.y();
        trans.transform.rotation.z = tf_ema_q.z();
        tf_br->sendTransform(trans);
    }
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(publish_lidar_time);
    msg_body_pose.header.frame_id = map_frame;

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        // Two-stage tight couple: residual_select 1=Mid, 2=Odin (weak cov).
        const bool is_odin = point_body.intensity >= kOdinIntensityTag;
        if (residual_select == 1 && is_odin)
        {
            point_selected_surf[i] = false;
            continue;
        }
        if (residual_select == 2 && !is_odin)
        {
            point_selected_surf[i] = false;
            continue;
        }
        if (is_odin)
        {
            if (!bundle_enabled || !odin_pose_update_allowed())
            {
                point_selected_surf[i] = false;
                continue;
            }
        }
        else if (!bundle_enabled && current_lidar_num == 2)
        {
            point_selected_surf[i] = false;
            continue;
        }

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        // Odin: stricter plane fit so weak/misaligned L2 cannot dominate Mid.
        const float plane_thr = is_odin ? 0.05f : 0.1f;
        const float s_thr = is_odin ? 0.95f : 0.9f;
        if (esti_plane(pabcd, points_near, plane_thr))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > s_thr)
            {
                // Soft tight-couple: clip large Odin residuals so bad L2 cannot yank Mid.
                if (is_odin && lidar2_max_residual > 0.0 &&
                    std::fabs(pd2) > lidar2_max_residual)
                {
                    point_selected_surf[i] = false;
                    #ifdef MP_EN
                    #pragma omp atomic
                    #endif
                    n_odin_res_clip++;
                    continue;
                }
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effect_feat_num = 0;
    localizability_vec = Eigen::Vector3d::Zero();
    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effect_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effect_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effect_feat_num ++;
            localizability_vec += Eigen::Vector3d(normvec->points[i].x, normvec->points[i].y, normvec->points[i].z).array().square().matrix();
        }
    }
    localizability_vec = localizability_vec.cwiseSqrt();

    if (effect_feat_num < 1)
    {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        return;
    }

    res_mean_last = total_residual / effect_feat_num;
        
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effect_feat_num, 12); //23
    ekfom_data.h.resize(effect_feat_num);

    for (int i = 0; i < effect_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
}

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("laser_mapping", options)
    {
        //--------------------------------------
        // 1. Declare parameters
        //--------------------------------------
        this->declare_parameter<int>("common.max_iteration", 4);
        this->declare_parameter<bool>("common.async_debug", false);
        this->declare_parameter<bool>("common.multi_lidar", true);
        this->declare_parameter<bool>("common.publish_tf_results", true);
        this->declare_parameter<bool>("common.publish_odom_tf", true);
        this->declare_parameter<double>("common.tf_pose_ema", 0.15);

        this->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<std::string>("common.lid_topic2", "/livox/lidar");
        this->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<std::string>("common.map_frame", "map");

        this->declare_parameter<int>("method.voxelized_pt_num_thres", 100);
        this->declare_parameter<double>("method.effect_pt_num_ratio_thres", 0.4);
        this->declare_parameter<int>("method.bundle_enabled_tic_thres", 5);

        this->declare_parameter<double>("preprocess.filter_size_surf", 0.5);
        this->declare_parameter<int>("preprocess.point_filter_num", 2);
        this->declare_parameter<int>("preprocess.point_filter_num2", 2);
        this->declare_parameter<int>("preprocess.lidar_type", (int)AVIA);
        this->declare_parameter<int>("preprocess.lidar_type2", (int)AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.scan_line2", 16);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("preprocess.scan_rate2", 10);
        this->declare_parameter<int>("preprocess.timestamp_unit", (int)US);
        this->declare_parameter<int>("preprocess.timestamp_unit2", (int)US);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<double>("preprocess.blind2", 0.01);
        this->declare_parameter<double>("preprocess.max_range", 0.0);
        this->declare_parameter<double>("preprocess.max_range2", 0.0);
        this->declare_parameter<double>("preprocess.min_confidence", 0.0);
        this->declare_parameter<double>("preprocess.min_confidence2", 30.0);

        this->declare_parameter<double>("mapping.cube_side_length", 200.0);
        this->declare_parameter<float>("mapping.det_range", 300.0f);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);

        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        this->declare_parameter<bool>("mapping.lidar2_update_en", false);
        this->declare_parameter<bool>("mapping.lidar2_point_update_en", false);
        this->declare_parameter<bool>("mapping.lidar2_pose_factor_en", true);
        this->declare_parameter<double>("mapping.lidar2_pose_alpha", 0.12);
        this->declare_parameter<double>("mapping.lidar2_pose_max_dt", 0.20);
        this->declare_parameter<std::string>("mapping.odin_odom_topic", "/odin1/odometry");
        this->declare_parameter<double>("mapping.lidar2_pose_gyr_gate", 0.35);
        this->declare_parameter<double>("mapping.lidar2_max_sync_dt", 0.08);
        this->declare_parameter<double>("mapping.lidar2_residual_cov_scale", 25.0);
        this->declare_parameter<double>("mapping.lidar2_max_residual", 0.12);
        this->declare_parameter<bool>("mapping.lidar2_map_insert_en", false);
        this->declare_parameter<double>("mapping.lidar2_pitch_gyr_gate", 0.20);
        this->declare_parameter<double>("mapping.lidar2_vz_gate", 0.12);
        this->declare_parameter<double>("mapping.lidar2_pitch_gate_deg", 10.0);
        this->declare_parameter<bool>("mapping.extrinsic_L2_gravity_frame", false);
        this->declare_parameter<bool>("mapping.extrinsic_imu_to_lidars", true);

        this->declare_parameter<std::vector<double>>("mapping.extrinsic_T", {});
        this->declare_parameter<std::vector<double>>("mapping.extrinsic_R", {});
        this->declare_parameter<std::vector<double>>("mapping.extrinsic_T2", {});
        this->declare_parameter<std::vector<double>>("mapping.extrinsic_R2", {});
        this->declare_parameter<std::vector<double>>("mapping.extrinsic_T_L2_wrt_L1", {});
        this->declare_parameter<std::vector<double>>("mapping.extrinsic_R_L2_wrt_L1", {});
        this->declare_parameter<std::vector<double>>("mapping.extrinsic_T_L1_wrt_drone", {});
        this->declare_parameter<std::vector<double>>("mapping.extrinsic_R_L1_wrt_drone", {});

        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);

        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);

        //--------------------------------------
        // 2. Get parameters
        //--------------------------------------
        this->get_parameter_or("common.max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or("common.async_debug", async_debug, false);
        this->get_parameter_or("common.multi_lidar", multi_lidar, true);
        this->get_parameter_or("common.publish_tf_results", publish_tf_results, true);
        this->get_parameter_or("common.publish_odom_tf", publish_odom_tf, true);
        this->get_parameter_or("common.tf_pose_ema", tf_pose_ema, 0.15);

        this->get_parameter_or("common.lid_topic", lid_topic, std::string("/livox/lidar"));
        this->get_parameter_or("common.lid_topic2", lid_topic2, std::string("/livox/lidar"));
        this->get_parameter_or("common.imu_topic", imu_topic, std::string("/livox/imu"));
        this->get_parameter_or("common.map_frame", map_frame, std::string("map"));

        this->get_parameter_or("method.voxelized_pt_num_thres", voxelized_pt_num_thres, 100);
        this->get_parameter_or("method.effect_pt_num_ratio_thres", effect_pt_num_ratio_thres, 0.4);
        this->get_parameter_or("method.bundle_enabled_tic_thres", bundle_enabled_tic_thres, 5);

        this->get_parameter_or("preprocess.filter_size_surf", filter_size_surf, 0.5);
        this->get_parameter_or("preprocess.point_filter_num", p_pre->point_filter_num[0], 2);
        this->get_parameter_or("preprocess.point_filter_num2", p_pre->point_filter_num[1], 2);
        this->get_parameter_or("preprocess.lidar_type", p_pre->lidar_type[0], (int)AVIA);
        this->get_parameter_or("preprocess.lidar_type2", p_pre->lidar_type[1], (int)AVIA);
        this->get_parameter_or("preprocess.scan_line", p_pre->N_SCANS[0], 16);
        this->get_parameter_or("preprocess.scan_line2", p_pre->N_SCANS[1], 16);
        this->get_parameter_or("preprocess.scan_rate", p_pre->SCAN_RATE[0], 10);
        this->get_parameter_or("preprocess.scan_rate2", p_pre->SCAN_RATE[1], 10);
        this->get_parameter_or("preprocess.timestamp_unit", p_pre->time_unit[0], (int)US);
        this->get_parameter_or("preprocess.timestamp_unit2", p_pre->time_unit[1], (int)US);
        this->get_parameter_or("preprocess.blind", p_pre->blind[0], 0.01);
        this->get_parameter_or("preprocess.blind2", p_pre->blind[1], 0.01);
        this->get_parameter_or("preprocess.max_range", p_pre->max_range[0], 0.0);
        this->get_parameter_or("preprocess.max_range2", p_pre->max_range[1], 0.0);
        {
            double c0 = 0.0, c1 = 30.0;
            this->get_parameter_or("preprocess.min_confidence", c0, 0.0);
            this->get_parameter_or("preprocess.min_confidence2", c1, 30.0);
            p_pre->min_confidence[0] = static_cast<float>(c0);
            p_pre->min_confidence[1] = static_cast<float>(c1);
        }
        p_pre->set();

        this->get_parameter_or("mapping.cube_side_length", cube_len, 200.0);
        this->get_parameter_or("mapping.det_range", DET_RANGE, 300.0f);
        this->get_parameter_or("mapping.gyr_cov", gyr_cov, 0.1);
        this->get_parameter_or("mapping.acc_cov", acc_cov, 0.1);
        this->get_parameter_or("mapping.b_gyr_cov", b_gyr_cov, 0.0001);
        this->get_parameter_or("mapping.b_acc_cov", b_acc_cov, 0.0001);
        this->get_parameter_or("mapping.extrinsic_est_en", extrinsic_est_en, true);
        this->get_parameter_or("mapping.lidar2_update_en", lidar2_update_en, false);
        this->get_parameter_or("mapping.lidar2_point_update_en", lidar2_point_update_en, false);
        this->get_parameter_or("mapping.lidar2_pose_factor_en", lidar2_pose_factor_en, true);
        this->get_parameter_or("mapping.lidar2_pose_alpha", lidar2_pose_alpha, 0.12);
        this->get_parameter_or("mapping.lidar2_pose_max_dt", lidar2_pose_max_dt, 0.20);
        this->get_parameter_or("mapping.odin_odom_topic", odin_odom_topic, std::string("/odin1/odometry"));
        this->get_parameter_or("mapping.lidar2_pose_gyr_gate", lidar2_pose_gyr_gate, 0.35);
        this->get_parameter_or("mapping.lidar2_max_sync_dt", lidar2_max_sync_dt, 0.08);
        this->get_parameter_or("mapping.lidar2_residual_cov_scale", lidar2_residual_cov_scale, 25.0);
        this->get_parameter_or("mapping.lidar2_max_residual", lidar2_max_residual, 0.12);
        this->get_parameter_or("mapping.lidar2_map_insert_en", lidar2_map_insert_en, false);
        this->get_parameter_or("mapping.lidar2_pitch_gyr_gate", lidar2_pitch_gyr_gate, 0.20);
        this->get_parameter_or("mapping.lidar2_vz_gate", lidar2_vz_gate, 0.12);
        this->get_parameter_or("mapping.lidar2_pitch_gate_deg", lidar2_pitch_gate_deg, 10.0);
        this->get_parameter_or("mapping.extrinsic_L2_gravity_frame", extrinsic_L2_gravity_frame, false);
        this->get_parameter_or("mapping.extrinsic_imu_to_lidars", extrinsic_imu_to_lidars, true);

        this->get_parameter_or("mapping.extrinsic_T", extrinT, std::vector<double>());
        this->get_parameter_or("mapping.extrinsic_R", extrinR, std::vector<double>());
        this->get_parameter_or("mapping.extrinsic_T2", extrinT2, std::vector<double>());
        this->get_parameter_or("mapping.extrinsic_R2", extrinR2, std::vector<double>());
        this->get_parameter_or("mapping.extrinsic_T_L2_wrt_L1", extrinT3, std::vector<double>());
        this->get_parameter_or("mapping.extrinsic_R_L2_wrt_L1", extrinR3, std::vector<double>());
        this->get_parameter_or("mapping.extrinsic_T_L1_wrt_drone", extrinT4, std::vector<double>());
        this->get_parameter_or("mapping.extrinsic_R_L1_wrt_drone", extrinR4, std::vector<double>());

        this->get_parameter_or("publish.path_en", path_en, true);
        this->get_parameter_or("publish.scan_publish_en", scan_pub_en, true);
        this->get_parameter_or("publish.dense_publish_en", dense_pub_en, true);
        this->get_parameter_or("publish.scan_bodyframe_pub_en", scan_body_pub_en, true);

        this->get_parameter_or("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or("pcd_save.interval", pcd_save_interval, -1);

        //---------------------------
        // 3. Create Subscribers
        //---------------------------
        if (p_pre->lidar_type[0] == (int)AVIA)
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk);
        else
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);

        if (multi_lidar)
        {
            if (p_pre->lidar_type[1] == (int)AVIA)
                sub_pcl_livox2_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic2, 20, livox_pcl_cbk2);
            else
                sub_pcl_pc2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic2, rclcpp::SensorDataQoS(), standard_pcl_cbk2);
        }

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk);
        if (lidar2_pose_factor_en)
        {
            sub_odin_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
                odin_odom_topic, 50, odin_odom_cb);
            std::cout << "\033[32;1mOdin pose-factor subscribe: " << odin_odom_topic
                      << " alpha=" << lidar2_pose_alpha << "\033[0m" << std::endl;
        }

        //---------------------------
        // 4. Create Publishers
        //---------------------------
        pubLaserCloudFull_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
        pubLaserCloudOdin_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_odin_registered", 20);
        g_pub_odin_registered = pubLaserCloudOdin_;
        pubLaserCloudFullTransformed_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_tf", 20);
        pubLaserCloudFull_body_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
        pubLaserCloudMap_ = create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
        pubOdomAftMapped_ = create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
        pubMavrosVisionPose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 20);
        pubPath_ = create_publisher<nav_msgs::msg::Path>("/path", 20);

        pubCaclTime_ = create_publisher<std_msgs::msg::Float32>("/calc_time", 20);
        pubPointNum_ = create_publisher<std_msgs::msg::Float32>("/point_number", 20);
        pubLocalizabilityX_ = create_publisher<std_msgs::msg::Float32>("/localizability_x", 20);
        pubLocalizabilityY_ = create_publisher<std_msgs::msg::Float32>("/localizability_y", 20);
        pubLocalizabilityZ_ = create_publisher<std_msgs::msg::Float32>("/localizability_z", 20);

        //---------------------------
        // 5. TF Broadcaster
        //---------------------------
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //---------------------------
        // 6. Timers (replacing ROS1 loop)
        //---------------------------
        auto period_ms = std::chrono::milliseconds(10); // 100 Hz
        timer_ = rclcpp::create_timer(this, get_clock(), period_ms,
                                      std::bind(&LaserMappingNode::timer_callback, this));

        //---------------------------
        // 7. Initialize Path
        //---------------------------
        path.header.frame_id = map_frame;
        path.header.stamp = this->get_clock()->now();

        //---------------------------
        // 8. Set Parameters
        //---------------------------
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf, filter_size_surf, filter_size_surf);
        ikdtree.set_downsample_param(filter_size_surf);

        //---------------------------
        // 9. Extrinsics setup
        //---------------------------
        V3D Lidar_T_wrt_IMU(Zero3d);
        M3D Lidar_R_wrt_IMU(Eye3d);
        Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);

        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        // for multi lidar tf only
        if (multi_lidar)
        {
            if (extrinsic_imu_to_lidars)
            {
                Eigen::Matrix4d Lidar_wrt_IMU = Eigen::Matrix4d::Identity();
                Eigen::Matrix4d Lidar2_wrt_IMU = Eigen::Matrix4d::Identity();
                V3D LiDAR2_T_wrt_IMU;
                LiDAR2_T_wrt_IMU << VEC_FROM_ARRAY(extrinT2);
                M3D LiDAR2_R_wrt_IMU;
                LiDAR2_R_wrt_IMU << MAT_FROM_ARRAY(extrinR2);
                Lidar_wrt_IMU.block<3, 3>(0, 0) = Lidar_R_wrt_IMU;
                Lidar_wrt_IMU.block<3, 1>(0, 3) = Lidar_T_wrt_IMU;
                Lidar2_wrt_IMU.block<3, 3>(0, 0) = LiDAR2_R_wrt_IMU;
                Lidar2_wrt_IMU.block<3, 1>(0, 3) = LiDAR2_T_wrt_IMU;
                LiDAR2_wrt_LiDAR1 = Lidar_wrt_IMU.inverse() * Lidar2_wrt_IMU;
            }
            else
            {
                V3D LiDAR2_T_wrt_LiDAR1;
                LiDAR2_T_wrt_LiDAR1 << VEC_FROM_ARRAY(extrinT3);
                M3D Lidar2_R_wrt_LiDAR1;
                Lidar2_R_wrt_LiDAR1 << MAT_FROM_ARRAY(extrinR3);
                LiDAR2_wrt_LiDAR1.block<3, 3>(0, 0) = Lidar2_R_wrt_LiDAR1;
                LiDAR2_wrt_LiDAR1.block<3, 1>(0, 3) = LiDAR2_T_wrt_LiDAR1;
                LiDAR2_R_yaml = Lidar2_R_wrt_LiDAR1;
            }
            if (extrinsic_imu_to_lidars)
                LiDAR2_R_yaml = LiDAR2_wrt_LiDAR1.block<3, 3>(0, 0);
            cout << "\033[32;1mMulti LiDAR on!" << endl;
            cout << "lidar_type[0]: " << p_pre->lidar_type[0] << ", " << "lidar_type[1]: " << p_pre->lidar_type[1] << endl
                 << "lidar2_update_en: " << (lidar2_update_en ? "true" : "false")
                 << " point_update=" << (lidar2_point_update_en ? "true" : "false")
                 << " pose_factor=" << (lidar2_pose_factor_en ? "true" : "false")
                 << " pose_alpha=" << lidar2_pose_alpha
                 << " max_sync_dt=" << lidar2_max_sync_dt << "s"
                 << " odin_cov_scale=" << lidar2_residual_cov_scale
                 << " max_odin_res=" << lidar2_max_residual << "m"
                 << " odin_map_insert=" << (lidar2_map_insert_en ? "true" : "false")
                 << endl
                 << endl;
            cout << "L2 wrt L1 TF: " << endl
                 << LiDAR2_wrt_LiDAR1 << "\033[0m" << endl
                 << endl;
        }
        if (publish_tf_results)
        {
            V3D LiDAR1_T_wrt_drone;
            LiDAR1_T_wrt_drone << VEC_FROM_ARRAY(extrinT4);
            M3D LiDAR2_R_wrt_drone;
            LiDAR2_R_wrt_drone << MAT_FROM_ARRAY(extrinR4);
            LiDAR1_wrt_drone.block<3, 3>(0, 0) = LiDAR2_R_wrt_drone;
            LiDAR1_wrt_drone.block<3, 1>(0, 3) = LiDAR1_T_wrt_drone;
            cout << "\033[32;1mLiDAR wrt Drone:" << endl;
            cout << LiDAR1_wrt_drone << "\033[0m" << endl
                 << endl;
        }

        //---------------------------
        // 10. Initialize Kalman Filter
        //---------------------------
        double epsi[23];
        std::fill(epsi, epsi + 23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        RCLCPP_INFO(this->get_logger(), "LaserMappingNode initialized.");
    }

    ~LaserMappingNode()
    {
        /**************** save map (on shutdown) ****************/
        /* 1. make sure you have enough memory
         * 2. pcd save will largely influence the real-time performance */
        if (pcd_save_en && pcl_wait_save && !pcl_wait_save->empty())
        {
            std::string file_name = "scans.pcd";
            std::string all_points_dir = std::string(ROOT_DIR) + "PCD/" + file_name;

            pcl::PCDWriter pcd_writer;
            RCLCPP_INFO(this->get_logger(), "Saving final map to: %s", all_points_dir.c_str());
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
        }
    }

private:
    //----------------------------------------
    // TIMER CALLBACK (mapping loop replacement)
    //----------------------------------------
    void timer_callback()
    {
        if (flg_exit)
            return;
        // Stock FAST_LIO_MULTI adaptive: bundle when enabled, else async.
        // If bundle is on but L2 dried up (downsample waiting on Mid odom), fall
        // back to async Mid-only so /Odometry keeps flowing and breaks the deadlock.
        bool synced = false;
        if (bundle_enabled)
        {
            synced = sync_packages_bundle(Measures); // bundle
            if (!synced && !lidar_buffer.empty() && lidar_buffer2.empty() && !imu_buffer.empty())
                synced = sync_packages_async(Measures);
        }
        else
            synced = sync_packages_async(Measures); // single or async
        if (synced)
        {
            // RCLCPP_INFO(this->get_logger(), "Timer tick: synced=%d, bundle=%d", synced, bundle_enabled);
            high_resolution_clock::time_point t1 = high_resolution_clock::now();

            // Async Odin-only: do NOT IMU-propagate Mid to L2 stamps (Process
            // would kf.predict along Odin time and leave Mid pose twisted until
            // the next Mid update). Independent densify owns Odin viz.
            if (multi_lidar && !bundle_enabled && current_lidar_num == 2)
                return;

            if (bundle_enabled)
            {
                p_imu->Process(Measures, kf, feats_undistort, true);
            }
            else
            {
                p_imu->Process(Measures, kf, feats_undistort, false, current_lidar_num);
            }
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            // Level Odin: flush pending body cloud with this Mid state (same tick).
            if (multi_lidar && !odin_gravity_ext_comp && p_imu->IMU_inited())
            {
                odin_gravity_ext_comp = true;
                if (extrinsic_L2_gravity_frame)
                    std::cout << "\033[32;1mOdin L2: Mid yaw+pos with Mid IEKF state "
                                 "(pending flush; no Mid pitch/roll)\033[0m"
                              << std::endl;
                else
                    std::cout << "\033[32;1mOdin L2 extrinsic used as body-fixed:\n"
                              << LiDAR2_wrt_LiDAR1 << "\033[0m" << std::endl;
            }
            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            feats_down_size = feats_down_body->points.size();
            // RCLCPP_INFO(this->get_logger(),
            //             "Downsampled size = %d (input=%zu)",
            //             feats_down_size,
            //             feats_undistort->size());

            /*** initialize the map kdtree ***/
            if (multi_lidar)
            {
                // Tight-coupling bootstrap: do NOT block Mid IEKF/odom on first Odin frame.
                // Waiting for lidar2_ikd_init while lid_topic2 is produced by a bridge that
                // itself waits on /Odometry deadlocks (no Odom → no L2 → no Odom).
                if (ikdtree.Root_Node == nullptr || !lidar1_ikd_init)
                {
                    // RCLCPP_WARN(this->get_logger(),
                    //             "IKD-tree not initialized yet: root=%p L1=%d L2=%d feats_down_size=%d",
                    //             ikdtree.Root_Node,
                    //             lidar1_ikd_init,
                    //             lidar2_ikd_init,
                    //             feats_down_size);
                    if (feats_down_size > 5)
                    {
                        feats_down_world->resize(feats_down_size);
                        PointVector init_pts;
                        init_pts.reserve(feats_down_size);
                        for (int i = 0; i < feats_down_size; i++)
                        {
                            if (!odin_pose_update_allowed() &&
                                feats_down_body->points[i].intensity >= kOdinIntensityTag)
                                continue;
                            pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                            init_pts.push_back(feats_down_world->points[i]);
                        }
                        if (!init_pts.empty())
                            ikdtree.Add_Points(init_pts, true);
                        if (current_lidar_num == 1 || bundle_enabled)
                            lidar1_ikd_init = true;
                        if (current_lidar_num == 2 || bundle_enabled || !lidar2_update_en)
                            lidar2_ikd_init = true;
                    }
                    return;
                }
                if (lidar2_update_en && !lidar2_ikd_init && current_lidar_num == 2)
                    lidar2_ikd_init = true;
            }
            else if (ikdtree.Root_Node == nullptr)
            {
                if (feats_down_size > 5)
                {
                    feats_down_world->resize(feats_down_size);
                    for (int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Add_Points(feats_down_world->points, true);
                }
                return;
            }

            /*** ICP and iterated Kalman filter update ***/
            // RCLCPP_INFO(this->get_logger(), "Performing ICP + EKF...");
            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            // Never let async Odin-only packages update Mid IEKF (twists Mid pose).
            // Bundle Mid+Odin may still use gated Odin residuals; L2-only = densify/viz.
            const bool skip_l2_update =
                (!bundle_enabled && current_lidar_num == 2) ||
                (!odin_pose_update_allowed() && current_lidar_num == 2);

            if (!skip_l2_update)
            {
                normvec->resize(feats_down_size);
                feats_down_world->resize(feats_down_size);
                Nearest_Points.resize(feats_down_size);
                /*** iterated state estimation (two-stage tight couple) ***/
                double solve_H_time = 0;
                last_odin_iekf_ok = false;
                // Stage 1: Mid residuals lock pose (prevents Odin from twisting Mid).
                residual_select = 1;
                kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
                // Optional Stage 2 point residuals (usually OFF — prefer pose-factor).
                const bool odin_ok = lidar2_point_update_en && bundle_enabled &&
                                     odin_pose_update_allowed() &&
                                     lidar2_residual_cov_scale > 0.0;
                if (odin_ok)
                {
                    residual_select = 2;
                    const double R_odin = LASER_POINT_COV * odin_cov_scale_now();
                    double solve2 = 0;
                    kf.update_iterated_dyn_share_modified(R_odin, solve2);
                    solve_H_time += solve2;
                    last_odin_iekf_ok = (effect_feat_num > 0);
                    if (last_odin_iekf_ok)
                        n_odin_iekf_updates++;
                }
                else if (bundle_enabled && lidar2_update_en && lidar2_point_update_en)
                    n_odin_hold_stairs++;
                residual_select = 1;
                // Scheme ②: soft relative pose factor from Odin odometry.
                apply_odin_pose_factor();
                state_point = kf.get_x();
                pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
                // Refresh climb/pitch gates from Mid state for the next frame.
                last_mid_vz = state_point.vel(2);
                last_mid_pitch = mid_pitch_from_rot(state_point.rot.toRotationMatrix());
                geoQuat.x = state_point.rot.coeffs()[0];
                geoQuat.y = state_point.rot.coeffs()[1];
                geoQuat.z = state_point.rot.coeffs()[2];
                geoQuat.w = state_point.rot.coeffs()[3];

                /******* Publish odometry *******/
                if (publish_tf_results)
                    publish_visionpose(pubMavrosVisionPose_);
                publish_odometry(pubOdomAftMapped_, tf_broadcaster_);

                /*** add the feature points to map kdtree ***/
                map_incremental();

                // Publish accumulated ikdtree map (stock code left this under if(0)).
                {
                    static int map_pub_tic = 0;
                    if (ikdtree.Root_Node != nullptr && (++map_pub_tic % 2 == 0))
                    {
                        PointVector().swap(ikdtree.PCL_Storage);
                        ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                        featsFromMap->clear();
                        featsFromMap->points = ikdtree.PCL_Storage;
                        publish_map(pubLaserCloudMap_);
                    }
                }
                if (path_en)
                    publish_path(pubPath_);
            }
            else
            {
                // Safety net if an Odin frame reached IEKF path during hold.
                publish_odin_from_undistorted(pubLaserCloudOdin_, feats_undistort);
                return;
            }

            /******* Publish points *******/
            // Always split Mid/Odin: RViz /cloud_registered must stay Mid-only.
            // Fused Mid+Odin made turns look like Mid was misaligned.
            if (multi_lidar)
            {
                publish_mid_from_undistorted(pubLaserCloudFull_, feats_undistort);
                publish_odin_from_undistorted(pubLaserCloudOdin_, feats_undistort);
                if (!lidar2_update_en)
                    flush_pending_odin_to_map(publish_lidar_time);
            }
            else if (scan_pub_en || pcd_save_en)
                publish_frame_world(pubLaserCloudFull_, pubLaserCloudFullTransformed_);
            if (scan_pub_en && scan_body_pub_en)
                publish_frame_body(pubLaserCloudFull_body_);

            /*** Bundle update or async update ***/
            if (multi_lidar && lidar2_update_en)
            {
                // Full-time tight couple: always prefer Mid+Odin bundle when L2 on.
                bundle_enabled = true;
                bundle_enabled_tic = 0;
            }
            else if (multi_lidar && !lidar2_update_en)
            {
                // Odin viz-only: do not force bundle (keeps Mid realtime under async).
                bundle_enabled = false;
            }
            high_resolution_clock::time_point t2 = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(t2 - t1).count() / 1000.0;
            std_msgs::msg::Float32 calc_time;
            calc_time.data = duration;
            pubCaclTime_->publish(calc_time);
            std_msgs::msg::Float32 point_num;
            point_num.data = feats_down_size;
            pubPointNum_->publish(point_num);
            std_msgs::msg::Float32 localizability_x, localizability_y, localizability_z;
            localizability_x.data = localizability_vec(0);
            localizability_y.data = localizability_vec(1);
            localizability_z.data = localizability_vec(2);
            pubLocalizabilityX_->publish(localizability_x);
            pubLocalizabilityY_->publish(localizability_y);
            pubLocalizabilityZ_->publish(localizability_z);
        }
    }

    //----------------------------------------
    // Member variables
    //----------------------------------------
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc2_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox2_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odin_odom_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudOdin_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullTransformed_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubMavrosVisionPose_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubCaclTime_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubPointNum_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubLocalizabilityX_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubLocalizabilityY_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubLocalizabilityZ_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

///===========================
///  ROS2 MAIN()
///===========================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, SigHandle);

    rclcpp::spin(std::make_shared<LaserMappingNode>());

    rclcpp::shutdown();
    return 0;
}
