// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//
// Modifier: Livox               dev@livoxtech.com
//
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
#include <mutex>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <condition_variable>
/// module headers
#include <omp.h>
/// Eigen
#include <Eigen/Core>
/// ros2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
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

#define LASER_POINT_COV (0.001)

/**************************/
bool pcd_save_en = false, extrinsic_est_en = true, path_en = true;
float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

string lid_topic, lid_topic2, imu_topic, map_frame = "map";
bool multi_lidar = false, async_debug = false, publish_tf_results = false;
bool extrinsic_imu_to_lidars = false;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_lidar2 = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_surf = 0;
double cube_len = 0, lidar_end_time = 0, lidar_end_time2 = 0;
double publish_lidar_time = 0.0;
int effect_feat_num = 0;
int feats_down_size = 0, NUM_MAX_ITERATIONS = 0, pcd_save_interval = -1, pcd_index = 0;
bool point_selected_surf[100000] = {0};
bool lidar_pushed = false, flg_exit = false;
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

vector<BoxPointType> cub_needrm;
vector<PointVector> Nearest_Points;
// temporal variables for extrinsics
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
vector<double> extrinT2(3, 0.0);
vector<double> extrinR2(9, 0.0);
vector<double> extrinT3(3, 0.0);
vector<double> extrinR3(9, 0.0);
vector<double> extrinT4(3, 0.0);
vector<double> extrinR4(9, 0.0);

deque<double> time_buffer;
deque<double> time_buffer2;
deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<PointCloudXYZI::Ptr> lidar_buffer2;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;
mutex mtx_buffer;
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
double lidar_mean_scantime = 0.0;
double lidar_mean_scantime2 = 0.0;
int scan_num = 0;
int scan_num2 = 0;
Eigen::Vector3d localizability_vec = Eigen::Vector3d::Zero();

// --- New globals for zeroing the published frame ---
bool zero_start_pose = false;   // parameter-controlled
int zero_start_delay_scans = 10; // how many scans to wait before zeroing
bool zero_pose_initialized = false;

Eigen::Matrix4d T_world_to_zero = Eigen::Matrix4d::Identity(); // T_ZW
Eigen::Vector3d pub_pos = Eigen::Vector3d::Zero();
Eigen::Quaterniond pub_quat = Eigen::Quaterniond::Identity();

int processed_scans_for_zero = 0;

void SigHandle(int sig)
{
    flg_exit = true;
    std::cout << "catch sig " << sig << std::endl;
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

void pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void lasermap_fov_segment()
{
    cub_needrm.clear();
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized)
    {
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move)
        return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    if (cub_needrm.size() > 0)
        ikdtree.Delete_Point_Boxes(cub_needrm);
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
    mtx_buffer.lock();
    if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
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
    mtx_buffer.lock();
    if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar2)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer2.clear();
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr, 1);
    ptr->header.seq = 1; // trick to distinguish lidar2 if needed
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

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
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

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr, 1);
    ptr->header.seq = 1; // trick
    lidar_buffer2.push_back(ptr);
    time_buffer2.push_back(last_timestamp_lidar2);

    mtx_buffer.unlock();
    sig_buffer.notify_all();
    first_lidar_scan_check = true;
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    // Prevent stacking IMU when no lidar yet (filter would propagate too far)
    if (!first_lidar_scan_check)
        return;

    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
    double timestamp = rclcpp::Time(msg->header.stamp).seconds();

    mtx_buffer.lock();
    if (timestamp < last_timestamp_imu)
    {
        std::cerr << "imu loop back, clear buffer" << std::endl;
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;
    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

// Pure bundle sync (multi-lidar or single)
bool sync_packages(MeasureGroup &meas)
{
    if (multi_lidar)
    {
        if (lidar_buffer.empty() || lidar_buffer2.empty() || imu_buffer.empty())
        {
            return false;
        }

        /*** push a lidar scan (bundle both lidars) ***/
        if (!lidar_pushed)
        {
            // --- Lidar 1 ---
            meas.lidar = lidar_buffer.front();
            for (size_t i = 1; i < lidar_buffer.size(); i++)
            {
                *meas.lidar += *lidar_buffer[i];
            }
            meas.lidar_beg_time = time_buffer.front();
            if (meas.lidar->points.size() <= 1)
            {
                lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
                std::cerr << "Too few input point cloud (lidar1)!\n";
            }
            else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
            {
                lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            }
            else
            {
                scan_num++;
                lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
                lidar_mean_scantime +=
                    (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
            }
            meas.lidar_end_time = lidar_end_time;

            // --- Lidar 2 ---
            meas.lidar2 = lidar_buffer2.front();
            for (size_t i = 1; i < lidar_buffer2.size(); i++)
            {
                *meas.lidar2 += *lidar_buffer2[i];
            }
            pcl::transformPointCloud(*meas.lidar2, *meas.lidar2, LiDAR2_wrt_LiDAR1);
            meas.lidar_beg_time2 = time_buffer2.front();
            if (meas.lidar2->points.size() <= 1)
            {
                lidar_end_time2 = meas.lidar_beg_time2 + lidar_mean_scantime2;
                std::cerr << "Too few input point cloud (lidar2)!\n";
            }
            else if (meas.lidar2->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime2)
            {
                lidar_end_time2 = meas.lidar_beg_time2 + lidar_mean_scantime2;
            }
            else
            {
                scan_num2++;
                lidar_end_time2 = meas.lidar_beg_time2 + meas.lidar2->points.back().curvature / double(1000);
                lidar_mean_scantime2 +=
                    (meas.lidar2->points.back().curvature / double(1000) - lidar_mean_scantime2) / scan_num2;
            }
            meas.lidar_end_time2 = lidar_end_time2;

            // Use max end time for stamping
            publish_lidar_time = std::max(lidar_end_time, lidar_end_time2);
            lidar_pushed = true;
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
            if (imu_time > lidar_end_time && imu_time > lidar_end_time2)
                break;
            meas.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }

        lidar_buffer.clear();
        time_buffer.clear();
        lidar_buffer2.clear();
        time_buffer2.clear();

        lidar_pushed = false;
        std::cout << "Bundle update!" << std::endl;
        return true;
    }
    else
    {
        if (lidar_buffer.empty() || imu_buffer.empty())
        {
            return false;
        }

        /*** push a lidar scan (single lidar) ***/
        if (!lidar_pushed)
        {
            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            if (meas.lidar->points.size() <= 1)
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
                scan_num++;
                lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
                lidar_mean_scantime +=
                    (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
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
            if (imu_time > lidar_end_time)
                break;
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
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty())
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_surf) * filter_size_surf + 0.5 * filter_size_surf;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_surf) * filter_size_surf + 0.5 * filter_size_surf;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_surf) * filter_size_surf + 0.5 * filter_size_surf;
            float dist = calc_dist(feats_down_world->points[i], mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_surf &&
                fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_surf &&
                fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_surf)
            {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
            {
                if (points_near.size() < NUM_MATCH_POINTS)
                    break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
                PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }
    ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
}

void publish_frame_world(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFull,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullTransFormed)
{
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                                &laserCloudWorld->points[i]);
        }

        if (zero_start_pose && zero_pose_initialized)
        {
            pcl::transformPointCloud(*laserCloudWorld, *laserCloudWorld, T_world_to_zero);
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
     * 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(
            new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i],
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
        {
            pcd_index++;
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
        RGBpointBodyLidarToIMU(&feats_undistort->points[i],
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
    PointCloudXYZI::Ptr map_world(new PointCloudXYZI(*featsFromMap));
    if (zero_start_pose && zero_pose_initialized)
    {
        pcl::transformPointCloud(*map_world, *map_world, T_world_to_zero);
    }

    sensor_msgs::msg::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*map_world, laserCloudMap);
    laserCloudMap.header.stamp = get_ros_time(publish_lidar_time);
    laserCloudMap.header.frame_id = map_frame;
    pubLaserCloudMap->publish(laserCloudMap);
}

// template <typename T>
// void set_posestamp(T &out)
// {
//     out.pose.position.x = state_point.pos(0);
//     out.pose.position.y = state_point.pos(1);
//     out.pose.position.z = state_point.pos(2);
//     out.pose.orientation.x = geoQuat.x;
//     out.pose.orientation.y = geoQuat.y;
//     out.pose.orientation.z = geoQuat.z;
//     out.pose.orientation.w = geoQuat.w;
// }

template <typename T>
void set_posestamp(T &out)
{
    out.pose.position.x = pub_pos(0);
    out.pose.position.y = pub_pos(1);
    out.pose.position.z = pub_pos(2);
    out.pose.orientation.x = pub_quat.x();
    out.pose.orientation.y = pub_quat.y();
    out.pose.orientation.z = pub_quat.z();
    out.pose.orientation.w = pub_quat.w();
}

void publish_visionpose(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &publisher)
{
    geometry_msgs::msg::PoseStamped msg_out_;
    msg_out_.header.frame_id = map_frame;
    msg_out_.header.stamp = get_ros_time(publish_lidar_time);

    Eigen::Matrix4d current_pose_eig_ = Eigen::Matrix4d::Identity();
    // current_pose_eig_.block<3, 3>(0, 0) = state_point.rot.toRotationMatrix();
    // current_pose_eig_.block<3, 1>(0, 3) = state_point.pos;
    current_pose_eig_.block<3, 3>(0, 0) = pub_quat.toRotationMatrix();
    current_pose_eig_.block<3, 1>(0, 3) = pub_pos;
    Eigen::Matrix4d tfed_vision_pose_eig_ =
        LiDAR1_wrt_drone * current_pose_eig_ * LiDAR1_wrt_drone.inverse();
    msg_out_.pose.position.x = tfed_vision_pose_eig_(0, 3);
    msg_out_.pose.position.y = tfed_vision_pose_eig_(1, 3);
    msg_out_.pose.position.z = tfed_vision_pose_eig_(2, 3);
    Eigen::Quaterniond tfed_quat_(tfed_vision_pose_eig_.block<3, 3>(0, 0));
    msg_out_.pose.orientation.x = tfed_quat_.x();
    msg_out_.pose.orientation.y = tfed_quat_.y();
    msg_out_.pose.orientation.z = tfed_quat_.z();
    msg_out_.pose.orientation.w = tfed_quat_.w();
    publisher->publish(msg_out_);
}

void publish_odometry(
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped,
    std::unique_ptr<tf2_ros::TransformBroadcaster> &tf_br)
{
    odomAftMapped.header.frame_id = map_frame;
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = get_ros_time(publish_lidar_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped->publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = map_frame;
    trans.header.stamp = odomAftMapped.header.stamp;
    trans.child_frame_id = "body";
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    tf_br->sendTransform(trans);
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(publish_lidar_time);
    msg_body_pose.header.frame_id = map_frame;

    /*** if path is too large, rviz will crash ***/
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
        PointType &point_body = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
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
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                                                                : true;
        }

        if (!point_selected_surf[i])
            continue;

        VF(4)
        pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y +
                        pabcd(2) * point_world.z + pabcd(3);
            float s_tmp = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s_tmp > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = fabs(pd2);
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
            effect_feat_num++;
            localizability_vec += Eigen::Vector3d(
                                      normvec->points[i].x,
                                      normvec->points[i].y,
                                      normvec->points[i].z)
                                      .array()
                                      .square()
                                      .matrix();
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

    /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effect_feat_num, 12);
    ekfom_data.h.resize(effect_feat_num);

    for (int i = 0; i < effect_feat_num; i++)
    {
        const PointType &laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measurement Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
            ekfom_data.h_x.block<1, 12>(i, 0)
                << norm_p.x,
                norm_p.y, norm_p.z,
                VEC_FROM_ARRAY(A),
                VEC_FROM_ARRAY(B),
                VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i, 0)
                << norm_p.x,
                norm_p.y, norm_p.z,
                VEC_FROM_ARRAY(A),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measurement: distance to the closest surface ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
}

///======================================
///   ROS2 NODE (Bundle-only variant)
///======================================
class LaserMappingBundleNode : public rclcpp::Node
{
public:
    LaserMappingBundleNode()
        : Node("laser_mapping_bundle")
    {
        //--------------------------------------
        // 1. Declare parameters
        //--------------------------------------
        this->declare_parameter<int>("common.max_iteration", 4);
        this->declare_parameter<bool>("common.async_debug", false);
        this->declare_parameter<bool>("common.multi_lidar", true);
        this->declare_parameter<bool>("common.publish_tf_results", true);

        this->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<std::string>("common.lid_topic2", "/livox/lidar");
        this->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<std::string>("common.map_frame", "map");

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

        this->declare_parameter<double>("mapping.cube_side_length", 200.0);
        this->declare_parameter<float>("mapping.det_range", 300.0f);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);

        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
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

        this->declare_parameter<bool>("mapping.zero_start_pose", false);
        this->declare_parameter<int>("mapping.zero_start_delay_scans", 10);

        //--------------------------------------
        // 2. Get parameters
        //--------------------------------------
        this->get_parameter_or("common.max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or("common.async_debug", async_debug, false);
        this->get_parameter_or("common.multi_lidar", multi_lidar, true);
        this->get_parameter_or("common.publish_tf_results", publish_tf_results, true);

        this->get_parameter_or("common.lid_topic", lid_topic, std::string("/livox/lidar"));
        this->get_parameter_or("common.lid_topic2", lid_topic2, std::string("/livox/lidar"));
        this->get_parameter_or("common.imu_topic", imu_topic, std::string("/livox/imu"));
        this->get_parameter_or("common.map_frame", map_frame, std::string("map"));

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

        this->get_parameter_or("mapping.cube_side_length", cube_len, 200.0);
        this->get_parameter_or("mapping.det_range", DET_RANGE, 300.0f);
        this->get_parameter_or("mapping.gyr_cov", gyr_cov, 0.1);
        this->get_parameter_or("mapping.acc_cov", acc_cov, 0.1);
        this->get_parameter_or("mapping.b_gyr_cov", b_gyr_cov, 0.0001);
        this->get_parameter_or("mapping.b_acc_cov", b_acc_cov, 0.0001);

        this->get_parameter_or("mapping.extrinsic_est_en", extrinsic_est_en, true);
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

        this->get_parameter_or("mapping.zero_start_pose", zero_start_pose, false);
        this->get_parameter_or("mapping.zero_start_delay_scans",
                               zero_start_delay_scans, 10);

        //--------------------------------------
        // 3. Initialize fixed structures
        //--------------------------------------
        path.header.stamp = this->now();
        path.header.frame_id = map_frame;
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf, filter_size_surf, filter_size_surf);
        ikdtree.set_downsample_param(filter_size_surf);

        //--------------------------------------
        // 4. Set IMU extrinsics & covariances
        //--------------------------------------
        V3D Lidar_T_wrt_IMU(Zero3d);
        M3D Lidar_R_wrt_IMU(Eye3d);
        Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        //--------------------------------------
        // 5. Multi-lidar static transform
        //--------------------------------------
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
            }
            cout << "\033[32;1mMulti LiDAR on!" << endl;
            cout << "lidar_type[0]: " << p_pre->lidar_type[0] << ", "
                 << "lidar_type[1]: " << p_pre->lidar_type[1] << endl
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

        //--------------------------------------
        // 6. Preprocess configuration
        //--------------------------------------
        p_pre->set();

        //--------------------------------------
        // 7. Init EKF
        //--------------------------------------
        double epsi[23];
        std::fill(epsi, epsi + 23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        //--------------------------------------
        // 8. Subscribers
        //--------------------------------------
        if (p_pre->lidar_type[0] == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                lid_topic, rclcpp::SensorDataQoS(),
                livox_pcl_cbk);
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lid_topic, rclcpp::SensorDataQoS(),
                standard_pcl_cbk);
        }

        if (multi_lidar)
        {
            if (p_pre->lidar_type[1] == AVIA)
            {
                sub_pcl_livox2_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                    lid_topic2, rclcpp::SensorDataQoS(),
                    livox_pcl_cbk2);
            }
            else
            {
                sub_pcl_pc2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    lid_topic2, rclcpp::SensorDataQoS(),
                    standard_pcl_cbk2);
            }
        }

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::SensorDataQoS(),
            imu_cbk);

        //--------------------------------------
        // 9. Publishers
        //--------------------------------------
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 10);
        pubLaserCloudFullTransformed_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_tf", 10);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 10);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 10);

        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 10);
        pubMavrosVisionPose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

        pubCaclTime_ = this->create_publisher<std_msgs::msg::Float32>("/calc_time", 10);
        pubPointNum_ = this->create_publisher<std_msgs::msg::Float32>("/point_number", 10);
        pubLocalizabilityX_ = this->create_publisher<std_msgs::msg::Float32>("/localizability_x", 10);
        pubLocalizabilityY_ = this->create_publisher<std_msgs::msg::Float32>("/localizability_y", 10);
        pubLocalizabilityZ_ = this->create_publisher<std_msgs::msg::Float32>("/localizability_z", 10);

        //--------------------------------------
        // 10. TF broadcaster & timer
        //--------------------------------------
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Timer: run mapping loop at ~100 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&LaserMappingBundleNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "LaserMappingBundleNode initialized (bundle-only).");
    }

    ~LaserMappingBundleNode()
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
    // TIMER CALLBACK (mapping loop)
    //----------------------------------------
    void timer_callback()
    {
        if (flg_exit)
            return;

        if (!sync_packages(Measures))
            return;

        auto t1 = high_resolution_clock::now();
        // always bundle mode
        p_imu->Process(Measures, kf, feats_undistort, multi_lidar);

        state_point = kf.get_x();
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

        if (!feats_undistort || feats_undistort->empty())
        {
            RCLCPP_WARN(this->get_logger(), "No point, skip this scan!");
            return;
        }

        /*** Segment the map in lidar FOV ***/
        lasermap_fov_segment();

        /*** downsample the feature points in a scan ***/
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        feats_down_size = feats_down_body->points.size();

        /*** initialize the map kdtree ***/
        if (ikdtree.Root_Node == nullptr)
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
        if (feats_down_size < 5)
        {
            RCLCPP_WARN(this->get_logger(), "Too few downsampled points, skip this scan!");
            return;
        }

        normvec->resize(feats_down_size);
        feats_down_world->resize(feats_down_size);
        Nearest_Points.resize(feats_down_size);

        /*** iterated state estimation ***/
        double solve_H_time = 0;
        kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
        state_point = kf.get_x();
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
        // geoQuat.x = state_point.rot.coeffs()[0];
        // geoQuat.y = state_point.rot.coeffs()[1];
        // geoQuat.z = state_point.rot.coeffs()[2];
        // geoQuat.w = state_point.rot.coeffs()[3];

        // --- New: build world->body transform from state_point ---
        Eigen::Matrix4d T_WB = Eigen::Matrix4d::Identity();
        T_WB.block<3, 3>(0, 0) = state_point.rot.toRotationMatrix();
        T_WB.block<3, 1>(0, 3) = state_point.pos;

        // --- Zero-start logic ---
        processed_scans_for_zero++;

        if (zero_start_pose &&
            !zero_pose_initialized &&
            processed_scans_for_zero >= zero_start_delay_scans)
        {
            // At this moment, define Z so that T_ZB(t0) = Identity
            T_world_to_zero = T_WB.inverse(); // T_ZW
            zero_pose_initialized = true;
            RCLCPP_INFO(this->get_logger(),
                        "Zero-start pose initialized at scan %d.",
                        processed_scans_for_zero);
        }

        // Pose to publish (either original or zeroed)
        Eigen::Matrix4d T_ZB = T_WB;
        if (zero_start_pose && zero_pose_initialized)
        {
            T_ZB = T_world_to_zero * T_WB; // T_ZB = T_ZW * T_WB
        }

        pub_pos = T_ZB.block<3, 1>(0, 3);
        pub_quat = Eigen::Quaterniond(T_ZB.block<3, 3>(0, 0));
        pub_quat.normalize(); // for safety

        /******* Publish odometry *******/
        if (publish_tf_results)
            publish_visionpose(pubMavrosVisionPose_);
        publish_odometry(pubOdomAftMapped_, tf_broadcaster_);

        /*** add the feature points to map kdtree ***/
        map_incremental();

        if (0) // If you need to see map points, change to "if(1)"
        {
            PointVector().swap(ikdtree.PCL_Storage);
            ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
            featsFromMap->clear();
            featsFromMap->points = ikdtree.PCL_Storage;
        }

        /******* Publish points *******/
        if (path_en)
            publish_path(pubPath_);
        if (scan_pub_en || pcd_save_en)
            publish_frame_world(pubLaserCloudFull_, pubLaserCloudFullTransformed_);
        if (scan_pub_en && scan_body_pub_en)
            publish_frame_body(pubLaserCloudFull_body_);
        // publish_map(pubLaserCloudMap_);

        auto t2 = high_resolution_clock::now();
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

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc2_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox2_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
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
    rclcpp::spin(std::make_shared<LaserMappingBundleNode>());
    rclcpp::shutdown();
    return 0;
}
