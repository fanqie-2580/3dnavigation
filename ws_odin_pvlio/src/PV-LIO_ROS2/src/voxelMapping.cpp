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
string  lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_surf_min = 0;
double total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
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

bool has_save_opt = false;
bool pcd_save_en = false;
std::string pcd_save_path;

std::unordered_map<VOXEL_LOC, OctoTree *> voxel_map;
std::mutex global_cloud_map_mtx;
PointCloudXYZI::Ptr global_cloud_map(new PointCloudXYZI());
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
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig)
{
    flg_exit = true;
    RCLCPP_WARN(rclcpp::get_logger("pv_lio"), "catch sig %d", sig);
    sig_buffer.notify_all();
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

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/ 
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            RCLCPP_WARN(rclcpp::get_logger("pv_lio"), "Too few input point cloud!");
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

        lidar_pushed = true;
    }

    // 最新的IMU数据没达到雷达结束时间，即整个雷达帧周期内，imu没填充满，这样是不行的，缺数据，false
    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
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

void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull_body)
{
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
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
        pv_list[i] = pv;
    }

    double match_start = omp_get_wtime();
    std::vector<ptpl> ptpl_list;
    std::vector<V3D> non_match_list;
    BuildResidualListOMP(voxel_map, max_voxel_size, 3.0, max_layer, pv_list,
                         ptpl_list, non_match_list);
    double match_end = omp_get_wtime();

    effct_feat_num = ptpl_list.size();
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
        ekfom_data.h(i) = -pd2;

        V3D point_world = ptpl_list[i].point_world;
        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq.block<1, 3>(0, 0) = point_world - ptpl_list[i].center;
        J_nq.block<1, 3>(0, 3) = -ptpl_list[i].normal;
        double sigma_l = J_nq * ptpl_list[i].plane_cov * J_nq.transpose();

        M3D cov_lidar = ptpl_list[i].cov_lidar;
        M3D R_cov_Rt = s.rot * s.offset_R_L_I * cov_lidar * s.offset_R_L_I.conjugate() * s.rot.conjugate();
        double R_inv = 1.0 / (sigma_l + norm_vec.transpose() * R_cov_Rt * norm_vec);

        ekfom_data.R(i) = R_inv;
    }

    res_mean_last = total_residual / effct_feat_num;
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

void saveMapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    has_save_opt = true;
    if (saveMapToFile(pcd_save_path)) {
        res->success = true;
        res->message = "Map saved successfully.";
    } else {
        res->success = false;
        res->message = "Failed to save map.";
    }
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
    node->get_parameter("publish.pub_voxel_map", publish_voxel_map);
    node->get_parameter("publish.publish_global_cloud_map", publish_global_cloud_map);
    node->get_parameter("publish.publish_max_voxel_layer", publish_max_voxel_layer);

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
    downSizeFilterGlobal.setLeafSize(0.1, 0.1, 0.1);            //0.4
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
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

    PointCloudXYZI::Ptr global_cloud_map_t(new PointCloudXYZI());
    PointCloudXYZI::Ptr filter_global_cloud_map_t(new PointCloudXYZI());

    // Timer to publish global map
    auto timer = node->create_wall_timer(1s,
        [&]()
        {
            {
                std::lock_guard<std::mutex> lck(global_cloud_map_mtx);
                *global_cloud_map_t = *global_cloud_map;
                global_cloud_map.reset(new PointCloudXYZI());
            }
            downSizeFilterGlobal.setInputCloud(global_cloud_map_t);
            downSizeFilterGlobal.filter(*filter_global_cloud_map_t);
            {
                std::lock_guard<std::mutex> lck(global_cloud_map_mtx);
                *global_cloud_map += *filter_global_cloud_map_t;
            }
            sensor_msgs::msg::PointCloud2 laserCloudmsg;
            pcl::toROSMsg(*filter_global_cloud_map_t, laserCloudmsg);
            laserCloudmsg.header.stamp = node->now();
            laserCloudmsg.header.frame_id = "world";
            pubGlobalCloudMap->publish(laserCloudmsg);
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

            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(node->get_logger(), "No point, skip this scan!");
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;

            if (flg_EKF_inited && !init_map) {
                PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI);
                transformLidar(state_point, feats_undistort, world_lidar);

                {
                    std::lock_guard<std::mutex> lck(global_cloud_map_mtx);
                    *global_cloud_map += *world_lidar;
                }

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

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);

            sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);

            feats_down_size = feats_down_body->points.size();

            var_down_body.clear();
            for (auto & pt:feats_down_body->points) {
                V3D point_this(pt.x, pt.y, pt.z);
                var_down_body.push_back(calcBodyCov(point_this, ranging_cov, angle_cov));
            }

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                RCLCPP_WARN(node->get_logger(), "No point, skip this scan!");
                continue;
            }

            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            state_ikfom state_before_update = kf.get_x();
            kf.update_iterated_dyn_share_diagonal();
            double t_update_end = omp_get_wtime();
            sum_optimize_time += t_update_end - t_update_start;

            // Guard against divergence: if the updated state is NaN or has blown up,
            // restore the previous valid state to prevent crash
            state_point = kf.get_x();
            bool state_bad = std::isnan(state_point.pos(0)) || std::isnan(state_point.pos(1)) || std::isnan(state_point.pos(2));
            // Check for position blow-up (normal driving should never exceed 1e4 meters)
            double pos_norm = state_point.pos.norm();
            if (pos_norm > 1e4) state_bad = true;
            // Check rotation matrix validity (Frobenius norm of a valid SO3 should be ~sqrt(3)≈1.732)
            auto rot_coeffs = state_point.rot.coeffs();
            double rot_norm = 0;
            for (int i = 0; i < 4; i++) rot_norm += rot_coeffs[i] * rot_coeffs[i];
            if (std::isnan(rot_norm) || std::fabs(rot_norm - 1.0) > 0.1) state_bad = true;
            if (state_bad)
            {
                RCLCPP_ERROR(node->get_logger(), "Divergence detected (pos_norm=%.1f, rot_norm=%.4f)! Restoring previous state.", pos_norm, rot_norm);
                kf.change_x(state_before_update);
                state_point = kf.get_x();
            }
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
            {
                std::lock_guard<std::mutex> lck(global_cloud_map_mtx);
                *global_cloud_map += *world_lidar;
            }

            // std::cout << "result----x: " << state_point.pos(0) << " " << state_point.pos(1) << " " << state_point.pos(2)
            //             << " " << euler_cur[0] << " " << euler_cur[1] << " " << euler_cur[2] << std::endl;
            // Print online-estimated LiDAR-IMU extrinsic
            Eigen::Vector3d ext_t = state_point.offset_T_L_I;
            auto ext_q_coeffs = state_point.offset_R_L_I.coeffs(); // [x, y, z, w]
            Eigen::Quaterniond ext_q(ext_q_coeffs[3], ext_q_coeffs[0], ext_q_coeffs[1], ext_q_coeffs[2]);
            Eigen::Matrix3d ext_R = ext_q.toRotationMatrix();
            // std::cout << "extrinsic_T: [" << ext_t(0) << ", " << ext_t(1) << ", " << ext_t(2) << "]"
            //           << "  extrinsic_R (quat wxyz): [" << ext_q.w() << ", " << ext_q.x() << ", " << ext_q.y() << ", " << ext_q.z() << "]"
            //           << "  R_pitch(y-deg): " << (std::asin(ext_R(0,2)) * 180.0 / M_PI)
            //           << std::endl;
            for (size_t i = 0; i < feats_down_body->size(); i++) {
                pointWithCov pv;
                pv.point << feats_down_body->points[i].x, feats_down_body->points[i].y, feats_down_body->points[i].z;
                M3D cov_lidar = var_down_body[i];
                M3D cov_world = transformLiDARCovToWorld(pv.point, kf, cov_lidar);

                pv.cov = cov_world;
                pv.point << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
                pv_list.push_back(pv);
            }

            t_update_start = omp_get_wtime();
            std::sort(pv_list.begin(), pv_list.end(), var_contrast);
            updateVoxelMapOMP(pv_list, max_voxel_size, max_layer, layer_size,
                           max_points_size, max_points_size, min_eigen_value,
                           voxel_map);
            t_update_end = omp_get_wtime();
            sum_update_time += t_update_end - t_update_start;

            scan_index++;

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped, tf_br, node);
            publish_pose(pubFreqPose);

            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);

            if (scan_pub_en)
                                                 publish_frame_world(pubLaserCloudFull);

            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
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
