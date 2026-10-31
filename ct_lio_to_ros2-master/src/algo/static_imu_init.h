//
// Created by xiang on 2021/11/11.
//

#ifndef SLAM_IN_AUTO_DRIVING_STATIC_IMU_INIT_H
#define SLAM_IN_AUTO_DRIVING_STATIC_IMU_INIT_H

#include "common/eigen_types.h"
#include "tools/imu.h"
#include "tools/odom.h"

#include <deque>

namespace zjloc
{

    /**
     * 静止状态下的 IMU 初始化器。
     * 它在一段静止时间内统计 IMU 均值和方差，给 ESKF 提供初始陀螺 bias、
     * 加速度 bias 以及重力方向。
     */
    class StaticIMUInit
    {
    public:
        struct Options
        {
            Options() {}
            double init_time_seconds_ = 1.0;            // 静止采样时长
            int init_imu_queue_max_size_ = 2000;        // 初始化队列最大长度
            int static_odom_pulse_ = 5;                 // 静止时轮速脉冲阈值
            double max_static_gyro_var = 0.5;           // 陀螺静止方差阈值
            double max_static_acce_var = 0.6;           // 加计静止方差阈值
            double gravity_norm_ = 9.81;                // 重力大小
            bool use_speed_for_static_checking_ = true; // 是否用轮速判断静止
        };

        explicit StaticIMUInit(Options options = Options()) : options_(options) {}

        bool AddIMU(const IMU &imu);
        bool AddOdom(const Odom &odom);

        bool InitSuccess() const { return init_success_; }

        Vec3d GetCovGyro() const { return cov_gyro_; }
        Vec3d GetCovAcce() const { return cov_acce_; }
        Vec3d GetInitBg() const { return init_bg_; }
        Vec3d GetInitBa() const { return init_ba_; }
        Vec3d GetGravity() const { return gravity_; }
        Vec3d GetMeanAcce() const { return mean_acce_; }

    private:
        bool TryInit();

        Options options_;
        bool init_success_ = false;
        Vec3d cov_gyro_ = Vec3d::Zero();
        Vec3d cov_acce_ = Vec3d::Zero();
        Vec3d init_bg_ = Vec3d::Zero();
        Vec3d init_ba_ = Vec3d::Zero();
        Vec3d gravity_ = Vec3d::Zero();
        Vec3d mean_acce_ = Vec3d::Zero(); // 机体系静止平均加速度
        bool is_static_ = false;
        std::deque<IMU> init_imu_deque_;
        double current_time_ = 0.0;
        double init_start_time_ = 0.0;
    };

} // namespace zjloc

#endif // SLAM_IN_AUTO_DRIVING_STATIC_IMU_INIT_H
