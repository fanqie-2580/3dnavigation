#include <algorithm>
#include <cmath>
#include <filesystem>
#include <random>
#include <unordered_map>
#include <vector>
#include <pcl/io/pcd_io.h>

#include "lidarodom.h"

namespace zjloc
{
  namespace
  {
    SO3 BuildInitialOrientationFromMeanAcce(const Vec3d &mean_acce)
    {
      if (mean_acce.norm() < 1e-6)
      {
        return SO3();
      }

      Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(
          mean_acce.normalized(), Vec3d::UnitZ());
      q.normalize();
      return SO3(q.toRotationMatrix());
    }

    Vec3d RotationMatrixToRpyDeg(const Mat3d &rotation)
    {
      const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
      const double pitch = std::atan2(-rotation(2, 0),
                                      std::sqrt(rotation(2, 1) * rotation(2, 1) +
                                                rotation(2, 2) * rotation(2, 2)));
      const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
      constexpr double kRadToDeg = 57.2957795130823208768;
      return Vec3d(roll * kRadToDeg, pitch * kRadToDeg, yaw * kRadToDeg);
    }

    double QuantileInPlace(std::vector<float> *values, double q)
    {
      if (!values || values->empty())
      {
        return 0.0;
      }
      q = std::clamp(q, 0.0, 1.0);
      const size_t idx =
          static_cast<size_t>(q * static_cast<double>(values->size() - 1));
      std::nth_element(values->begin(), values->begin() + static_cast<long>(idx),
                       values->end());
      return (*values)[idx];
    }
  } // namespace
#define USE_ANALYTICAL_DERIVATE 1 //    是否使用解析求导

  lidarodom::lidarodom(/* args */)
  {
    laser_point_cov = 0.001;
    current_state = new state(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    // 激光雷达置信度
    CT_ICP::LidarPlaneNormFactor::sqrt_info = sqrt(1 / laser_point_cov);

    CT_ICP::CTLidarPlaneNormFactor::sqrt_info = sqrt(1 / laser_point_cov);

    index_frame = 1;
    points_world.reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

  lidarodom::~lidarodom()
  {
  }

  void lidarodom::loadOptions(std::shared_ptr<rclcpp::Node> &nh)
  {
    // 声明并获取参数，使用ROS 2参数服务器方式
    nh->declare_parameter<double>("surf_res", 0.5);
    nh->get_parameter("surf_res", options_.surf_res);

    nh->declare_parameter<bool>("log_print", false);
    nh->get_parameter("log_print", options_.log_print);

    nh->declare_parameter<int>("max_num_iteration", 5);
    nh->get_parameter("max_num_iteration", options_.max_num_iteration);

    nh->declare_parameter<double>("size_voxel_map", 1.0);
    nh->get_parameter("size_voxel_map", options_.size_voxel_map);

    nh->declare_parameter<double>("min_distance_points", 0.1);
    nh->get_parameter("min_distance_points", options_.min_distance_points);

    nh->declare_parameter<int>("max_num_points_in_voxel", 20);
    nh->get_parameter("max_num_points_in_voxel", options_.max_num_points_in_voxel);

    nh->declare_parameter<double>("max_distance", 100.0);
    nh->get_parameter("max_distance", options_.max_distance);

    nh->declare_parameter<double>("weight_alpha", 0.5);
    nh->get_parameter("weight_alpha", options_.weight_alpha);

    nh->declare_parameter<double>("weight_neighborhood", 0.5);
    nh->get_parameter("weight_neighborhood", options_.weight_neighborhood);

    nh->declare_parameter<double>("max_dist_to_plane_icp", 0.5);
    nh->get_parameter("max_dist_to_plane_icp", options_.max_dist_to_plane_icp);

    nh->declare_parameter<int>("init_num_frames", 20);
    nh->get_parameter("init_num_frames", options_.init_num_frames);

    nh->declare_parameter<int>("voxel_neighborhood", 1);
    nh->get_parameter("voxel_neighborhood", options_.voxel_neighborhood);

    nh->declare_parameter<int>("max_number_neighbors", 20);
    nh->get_parameter("max_number_neighbors", options_.max_number_neighbors);

    nh->declare_parameter<int>("threshold_voxel_occupancy", 1);
    nh->get_parameter("threshold_voxel_occupancy", options_.threshold_voxel_occupancy);

    nh->declare_parameter<bool>("estimate_normal_from_neighborhood", true);
    nh->get_parameter("estimate_normal_from_neighborhood", options_.estimate_normal_from_neighborhood);

    nh->declare_parameter<int>("min_number_neighbors", 5);
    nh->get_parameter("min_number_neighbors", options_.min_number_neighbors);

    nh->declare_parameter<double>("power_planarity", 2.0);
    nh->get_parameter("power_planarity", options_.power_planarity);

    nh->declare_parameter<int>("num_closest_neighbors", 1);
    nh->get_parameter("num_closest_neighbors", options_.num_closest_neighbors);

    nh->declare_parameter<double>("sampling_rate", 1.0);
    nh->get_parameter("sampling_rate", options_.sampling_rate);

    nh->declare_parameter<double>("ratio_of_nonground", 0.5);
    nh->get_parameter("ratio_of_nonground", options_.ratio_of_nonground);

    nh->declare_parameter<int>("max_num_residuals", 2000);
    nh->get_parameter("max_num_residuals", options_.max_num_residuals);

    nh->declare_parameter<int>("min_num_residuals", 300);
    nh->get_parameter("min_num_residuals", options_.min_num_residuals);

    nh->declare_parameter<double>("beta_location_consistency", 0.0);
    nh->get_parameter("beta_location_consistency", options_.beta_location_consistency);

    nh->declare_parameter<double>("beta_orientation_consistency", 0.0);
    nh->get_parameter("beta_orientation_consistency", options_.beta_orientation_consistency);

    nh->declare_parameter<double>("beta_constant_velocity", 0.0);
    nh->get_parameter("beta_constant_velocity", options_.beta_constant_velocity);

    nh->declare_parameter<double>("beta_small_velocity", 0.0);
    nh->get_parameter("beta_small_velocity", options_.beta_small_velocity);

    nh->declare_parameter<double>("thres_orientation_norm", 0.1);
    nh->get_parameter("thres_orientation_norm", options_.thres_orientation_norm);

    nh->declare_parameter<double>("thres_translation_norm", 0.1);
    nh->get_parameter("thres_translation_norm", options_.thres_translation_norm);

    nh->declare_parameter<bool>("publish_full_map", true);
    nh->get_parameter("publish_full_map", options_.publish_full_map);

    nh->declare_parameter<int>("publish_full_map_every_n_frames", 10);
    nh->get_parameter("publish_full_map_every_n_frames", options_.publish_full_map_every_n_frames);
    if (options_.publish_full_map_every_n_frames < 1)
    {
      options_.publish_full_map_every_n_frames = 1;
    }

    nh->declare_parameter<bool>("level_map_enable", true);
    nh->get_parameter("level_map_enable", options_.level_map_enable);
    nh->declare_parameter<bool>("level_map_on_save", true);
    nh->get_parameter("level_map_on_save", options_.level_map_on_save);
    nh->declare_parameter<bool>("level_map_on_publish", true);
    nh->get_parameter("level_map_on_publish", options_.level_map_on_publish);
    nh->declare_parameter<int>("level_map_min_points", 3000);
    nh->get_parameter("level_map_min_points", options_.level_map_min_points);
    nh->declare_parameter<int>("level_map_lock_frame", 40);
    nh->get_parameter("level_map_lock_frame", options_.level_map_lock_frame);
    nh->declare_parameter<double>("level_map_low_z_quantile", 0.35);
    nh->get_parameter("level_map_low_z_quantile", options_.level_map_low_z_quantile);
    nh->declare_parameter<double>("level_map_min_tilt_deg", 0.5);
    nh->get_parameter("level_map_min_tilt_deg", options_.level_map_min_tilt_deg);
    if (options_.level_map_min_points < 100)
    {
      options_.level_map_min_points = 100;
    }
    if (options_.level_map_lock_frame < 1)
    {
      options_.level_map_lock_frame = 1;
    }
    options_.level_map_low_z_quantile =
        std::clamp(options_.level_map_low_z_quantile, 0.05, 0.8);

    nh->declare_parameter<bool>("map_rear_filter_enabled", false);
    nh->get_parameter("map_rear_filter_enabled", options_.map_rear_filter_enabled);

    nh->declare_parameter<double>("map_rear_filter_x_min", -2.0);
    nh->get_parameter("map_rear_filter_x_min", options_.map_rear_filter_x_min);

    nh->declare_parameter<double>("map_rear_filter_x_max", -0.4);
    nh->get_parameter("map_rear_filter_x_max", options_.map_rear_filter_x_max);

    nh->declare_parameter<double>("map_rear_filter_y_min", -0.3);
    nh->get_parameter("map_rear_filter_y_min", options_.map_rear_filter_y_min);

    nh->declare_parameter<double>("map_rear_filter_y_max", 0.3);
    nh->get_parameter("map_rear_filter_y_max", options_.map_rear_filter_y_max);

    nh->declare_parameter<double>("map_rear_filter_z_min", -0.2);
    nh->get_parameter("map_rear_filter_z_min", options_.map_rear_filter_z_min);

    nh->declare_parameter<double>("map_rear_filter_z_max", 1.7);
    nh->get_parameter("map_rear_filter_z_max", options_.map_rear_filter_z_max);

    auto normalize_bounds = [](double &min_value, double &max_value)
    {
      if (min_value > max_value)
      {
        const double tmp = min_value;
        min_value = max_value;
        max_value = tmp;
      }
    };
    normalize_bounds(options_.map_rear_filter_x_min, options_.map_rear_filter_x_max);
    normalize_bounds(options_.map_rear_filter_y_min, options_.map_rear_filter_y_max);
    normalize_bounds(options_.map_rear_filter_z_min, options_.map_rear_filter_z_max);
    if (options_.map_rear_filter_enabled)
    {
      LOG(INFO) << "Map rear filter enabled in lidar frame: "
                << "x[" << options_.map_rear_filter_x_min << ", "
                << options_.map_rear_filter_x_max << "], y["
                << options_.map_rear_filter_y_min << ", "
                << options_.map_rear_filter_y_max << "], z["
                << options_.map_rear_filter_z_min << ", "
                << options_.map_rear_filter_z_max << "]";
    }

    // motion compensation参数
    std::string str_motion_compensation;
    nh->declare_parameter<std::string>("motion_compensation", "CONTINUOUS");
    nh->get_parameter("motion_compensation", str_motion_compensation);
    if (str_motion_compensation == "NONE")
      options_.motion_compensation = MotionCompensation::NONE;
    else if (str_motion_compensation == "CONSTANT_VELOCITY")
      options_.motion_compensation = MotionCompensation::CONSTANT_VELOCITY;
    else if (str_motion_compensation == "ITERATIVE")
      options_.motion_compensation = MotionCompensation::ITERATIVE;
    else if (str_motion_compensation == "CONTINUOUS")
      options_.motion_compensation = MotionCompensation::CONTINUOUS;
    else
      std::cout << "The [motion_compensation](file:///home/wts_/simulate/src/aw2025_sentry_nav/src/sentry_localization/ct-lio/src/liw/lio/lidarodom.h#L51-L51) " << str_motion_compensation << " is not supported." << std::endl;

    // icpmodel参数
    std::string str_icpmodel;
    nh->declare_parameter<std::string>("icpmodel", "CT_POINT_TO_PLANE");
    nh->get_parameter("icpmodel", str_icpmodel);
    if (str_icpmodel == "POINT_TO_PLANE")
      options_.icpmodel = POINT_TO_PLANE;
    else if (str_icpmodel == "CT_POINT_TO_PLANE")
      options_.icpmodel = CT_POINT_TO_PLANE;
    else
      std::cout << "The `icp_residual` " << str_icpmodel << " is not supported." << std::endl;
  }

  bool lidarodom::init(std::shared_ptr<rclcpp::Node> &nh)
  {
    StaticIMUInit::Options imu_init_options;
    imu_init_options.use_speed_for_static_checking_ = false; // 本节数据不需要轮速计
    nh->declare_parameter<double>("max_static_acce_var", imu_init_options.max_static_acce_var);
    nh->declare_parameter<double>("max_static_gyro_var", imu_init_options.max_static_gyro_var);
    nh->declare_parameter<double>("imu_init_time_seconds", imu_init_options.init_time_seconds_);
    nh->declare_parameter<bool>("imu_gravity_level_enable", true);
    nh->get_parameter("max_static_acce_var", imu_init_options.max_static_acce_var);
    nh->get_parameter("max_static_gyro_var", imu_init_options.max_static_gyro_var);
    nh->get_parameter("imu_init_time_seconds", imu_init_options.init_time_seconds_);
    nh->get_parameter("imu_gravity_level_enable", imu_gravity_level_enable_);
    imu_init_ = StaticIMUInit(imu_init_options);

    // 从ROS 2参数服务器读取delay_time参数
    nh->declare_parameter<double>("delay_time", 0.0);
    nh->get_parameter("delay_time", delay_time_);

    // 从ROS 2参数服务器读取外参参数
    std::vector<double> ext_t(3, 0.0);
    std::vector<double> ext_r(9, 0.0);

    // 声明并获取lidar_T_wrt_IMU参数
    nh->declare_parameter<std::vector<double>>("extrinsic_T", ext_t);
    nh->get_parameter("extrinsic_T", ext_t);

    // 声明并获取lidar_R_wrt_IMU参数
    nh->declare_parameter<std::vector<double>>("extrinsic_R", ext_r);
    nh->get_parameter("extrinsic_R", ext_r);

    Vec3d lidar_T_wrt_IMU = math::VecFromArray(ext_t);
    Mat3d lidar_R_wrt_IMU = math::MatFromArray(ext_r);

    Eigen::Quaterniond q_IL(lidar_R_wrt_IMU);
    q_IL.normalize();
    lidar_R_wrt_IMU = q_IL;
    // init TIL
    TIL_ = SE3(q_IL, lidar_T_wrt_IMU);
    R_imu_lidar = lidar_R_wrt_IMU;
    t_imu_lidar = lidar_T_wrt_IMU;
    std::cout << "RIL:\n"
              << R_imu_lidar << std::endl;
    std::cout << "tIL:" << t_imu_lidar.transpose() << std::endl;

    //   TODO: need init here
    CT_ICP::LidarPlaneNormFactor::t_il = t_imu_lidar;
    CT_ICP::LidarPlaneNormFactor::q_il = TIL_.rotationMatrix();
    CT_ICP::CTLidarPlaneNormFactor::t_il = t_imu_lidar;
    CT_ICP::CTLidarPlaneNormFactor::q_il = TIL_.rotationMatrix();

    loadOptions(nh);
    switch (options_.motion_compensation)
    {
    case NONE:
    case CONSTANT_VELOCITY:
      options_.point_to_plane_with_distortion = false;
      options_.icpmodel = POINT_TO_PLANE;
      break;
    case ITERATIVE:
      options_.point_to_plane_with_distortion = true;
      options_.icpmodel = POINT_TO_PLANE;
      break;
    case CONTINUOUS:
      options_.point_to_plane_with_distortion = true;
      options_.icpmodel = CT_POINT_TO_PLANE;
      break;
    }
    LOG(WARNING) << "motion_compensation:" << options_.motion_compensation << ", model: " << options_.icpmodel;

    return true;
  }

  void lidarodom::pushData(std::vector<point3D> msg, std::pair<double, double> data)
  {
    if (data.first < last_timestamp_lidar_)
    {
      LOG(ERROR) << "lidar loop back, clear buffer";
      lidar_buffer_.clear();
      time_buffer_.clear();
    }

    mtx_buf.lock();
    lidar_buffer_.push_back(msg);
    time_buffer_.push_back(data);
    cout << "lidar point size" << (lidar_buffer_.back()).size() << std::endl;
    last_timestamp_lidar_ = data.first;
    mtx_buf.unlock();
    cond.notify_one();
  }
  void lidarodom::pushData(IMUPtr imu)
  {
    double timestamp = imu->timestamp_;
    if (timestamp < last_timestamp_imu_)
    {
      LOG(WARNING) << "imu loop back, clear buffer";
      imu_buffer_.clear();
    }

    last_timestamp_imu_ = timestamp;

    mtx_buf.lock();
    imu_buffer_.emplace_back(imu);
    mtx_buf.unlock();
    cond.notify_one();
  }

  void lidarodom::run()
  {
    while (true)
    {
      std::vector<MeasureGroup> measurements;
      std::unique_lock<std::mutex> lk(mtx_buf);

      std::cout << "lidarodom::run()" << std::endl;
      cond.wait(lk, [&]
                { return (measurements = getMeasureMents()).size() != 0; });
      lk.unlock();

      for (auto &m : measurements)
      {
        zjloc::common::Timer::Evaluate([&]()
                                       { ProcessMeasurements(m); },
                                       "processMeasurement");

        {
          auto real_time = std::chrono::high_resolution_clock::now();
          static std::chrono::system_clock::time_point prev_real_time = real_time;

          if (real_time - prev_real_time > std::chrono::seconds(5))
          {
            auto data_time = m.lidar_end_time_;
            static double prev_data_time = data_time;
            auto delta_real = std::chrono::duration_cast<std::chrono::milliseconds>(real_time - prev_real_time).count() * 0.001;
            auto delta_sim = data_time - prev_data_time;
            printf("Processing the rosbag at %.1fX speed.", delta_sim / delta_real);
            //>1.0: 实时处理速度快于数据产生速度（例如3.2X表示处理速度是数据产生速度的3.2倍）
            //<1.0: 实时处理速度慢于数据产生速度（系统有延迟）
            // 接近1.0: 实时处理速度与数据产生速度基本一致

            prev_data_time = data_time;
            prev_real_time = real_time;
          }
        }
      }
    }
  }

  void lidarodom::ProcessMeasurements(MeasureGroup &meas)
  {
    measures_ = meas;

    if (imu_need_init_)
    {
      TryInitIMU();
      return;
    }

    std::cout << ANSI_DELETE_LAST_LINE;
    std::cout << ANSI_COLOR_GREEN << "============== process frame: "
              << index_frame << ANSI_COLOR_RESET << std::endl;
    imu_states_.clear(); //   need clear here

    // 利用IMU数据进行状态预测
    zjloc::common::Timer::Evaluate([&]()
                                   { Predict(); },
                                   "predict");
    //
    zjloc::common::Timer::Evaluate([&]()
                                   { stateInitialization(); },
                                   "state init");

    std::vector<point3D> const_surf;
    const_surf.insert(const_surf.end(), meas.lidar_.begin(), meas.lidar_.end());
    // const_surf.assign(meas.lidar_.begin(), meas.lidar_.end());

    cloudFrame *p_frame;
    zjloc::common::Timer::Evaluate([&]()
                                   { p_frame = buildFrame(const_surf, current_state,
                                                          meas.lidar_begin_time_,
                                                          meas.lidar_end_time_); },
                                   "build frame");

    //   lio
    zjloc::common::Timer::Evaluate([&]()
                                   { poseEstimation(p_frame); },
                                   "poseEstimate");

    //   观测
    SE3 pose_of_lo_ = SE3(current_state->rotation, current_state->translation);
    // std::cout << "obs: " << current_state->translation.transpose() << ", " << current_state->rotation.transpose() << std::endl;
    // SE3 pred_pose = eskf_.GetNominalSE3();
    // std::cout << "pred: " << pred_pose.translation().transpose() << ", " << pred_pose.so3().log().transpose() << std::endl;
    zjloc::common::Timer::Evaluate([&]()
                                   { eskf_.ObserveSE3(pose_of_lo_, 1e-2, 1e-2); },
                                   "eskf_obs");

    zjloc::common::Timer::Evaluate([&]()
                                   {
               std::string laser_topic = "laser";
               SE3 pose_for_pub = pose_of_lo_;
               if (options_.level_map_enable && options_.level_map_on_publish &&
                   level_map_locked_)
               {
                 pose_for_pub = applyLevelRotationToPose(pose_of_lo_);
               }
               pub_pose_to_ros(laser_topic, pose_for_pub, meas.lidar_end_time_);
               laser_topic = "velocity";
               SE3 pred_pose = eskf_.GetNominalSE3();
               Eigen::Vector3d vel_world = eskf_.GetNominalVel();
               Eigen::Vector3d vel_base = pred_pose.rotationMatrix().inverse()*vel_world;
               pub_data_to_ros(laser_topic, vel_base.x(), 0);
               if(index_frame%8==0)
               {
                    laser_topic = "dist";
                    static Eigen::Vector3d last_t = Eigen::Vector3d::Zero();
                    Eigen::Vector3d t = pred_pose.translation();
                    static double dist = 0;
                    dist += (t - last_t).norm();
                    last_t = t;
                    pub_data_to_ros(laser_topic, dist, 0);
                    // std::cout << eskf_.GetGravity().transpose() << std::endl;
               } },
                                   "pub cloud");

    p_frame->p_state = new state(current_state, true);
    // all_cloud_frame.push_back(p_frame); //   TODO:     保存这个，特别费内存
    state *tmp_state = new state(current_state, true);
    all_state_frame.push_back(tmp_state);
    current_state = new state(current_state, false);

    index_frame++;
    p_frame->release();
    std::vector<point3D>().swap(meas.lidar_);
    std::vector<point3D>().swap(const_surf);
  }

  void lidarodom::poseEstimation(cloudFrame *p_frame)
  {
    //   TODO: check current_state data
    if (index_frame > 1)
    {
      zjloc::common::Timer::Evaluate([&]()
                                     { optimize(p_frame); },
                                     "optimize");
    }

    bool add_points = true;
    if (add_points)
    { //   update map here
      zjloc::common::Timer::Evaluate([&]()
                                     { map_incremental(p_frame); },
                                     "map update");
    }

    zjloc::common::Timer::Evaluate([&]()
                                   { lasermap_fov_segment(); },
                                   "fov segment");
  }

  void lidarodom::optimize(cloudFrame *p_frame)
  {
    std::cout << "size of point_cloud" << p_frame->point_surf.size() << std::endl;

    state *previous_state = nullptr;
    Eigen::Vector3d previous_translation = Eigen::Vector3d::Zero();
    Eigen::Vector3d previous_velocity = Eigen::Vector3d::Zero();
    Eigen::Quaterniond previous_orientation = Eigen::Quaterniond::Identity();

    // 开始的R和T：如果是第一帧则用IMU预测出来的，如果不是第一帧则用上一帧的
    // 结束的R和T：用IMU预测出来的
    state *curr_state = p_frame->p_state;
    Eigen::Quaterniond begin_quat = Eigen::Quaterniond(curr_state->rotation_begin);
    Eigen::Quaterniond end_quat = Eigen::Quaterniond(curr_state->rotation);
    Eigen::Vector3d begin_t = curr_state->translation_begin;
    Eigen::Vector3d end_t = curr_state->translation;

    if (p_frame->frame_id > 1)
    {
      if (options_.log_print)
        std::cout << "all_cloud_frame.size():" << all_state_frame.size() << ", " << p_frame->frame_id << std::endl;
      // previous_state = all_cloud_frame[p_frame->frame_id - 2]->p_state;
      previous_state = all_state_frame[p_frame->frame_id - 2];
      previous_translation = previous_state->translation;
      previous_velocity = previous_state->translation - previous_state->translation_begin;
      previous_orientation = previous_state->rotation;
    }
    if (options_.log_print)
    {
      std::cout << "prev end: " << previous_translation.transpose() << std::endl;
      std::cout << "curr begin: " << p_frame->p_state->translation_begin.transpose()
                << "\ncurr end: " << p_frame->p_state->translation.transpose() << std::endl;
    }

    std::vector<point3D> surf_keypoints;
    gridSampling(p_frame->point_surf, surf_keypoints,
                 options_.sampling_rate * options_.surf_res);

    size_t num_size = p_frame->point_surf.size();

    auto transformKeypoints = [&](std::vector<point3D> &point_frame)
    {
      Eigen::Matrix3d R;
      Eigen::Vector3d t;
      for (auto &keypoint : point_frame)
      {
        if (options_.point_to_plane_with_distortion ||
            options_.icpmodel == IcpModel::CT_POINT_TO_PLANE)
        {
          double alpha_time = keypoint.alpha_time;

          Eigen::Quaterniond q = begin_quat.slerp(alpha_time, end_quat);
          q.normalize();
          R = q.toRotationMatrix();
          t = (1.0 - alpha_time) * begin_t + alpha_time * end_t;
        }
        else
        {
          R = end_quat.normalized().toRotationMatrix();
          t = end_t;
        }
        keypoint.point = R * (TIL_ * keypoint.raw_point) + t;
      }
    };

    for (int iter(0); iter < options_.max_num_iteration; iter++)
    {
      transformKeypoints(surf_keypoints);

      // ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1 / (1.5e-3));
      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.5);
      ceres::Problem::Options problem_options;
      ceres::Problem problem(problem_options);
#ifdef USE_ANALYTICAL_DERIVATE
      ceres::LocalParameterization *parameterization = new RotationParameterization();
#else
      auto *parameterization = new ceres::EigenQuaternionParameterization();
#endif

      switch (options_.icpmodel)
      {
      case IcpModel::CT_POINT_TO_PLANE:
        problem.AddParameterBlock(&begin_quat.x(), 4, parameterization);
        problem.AddParameterBlock(&end_quat.x(), 4, parameterization);
        problem.AddParameterBlock(&begin_t.x(), 3);
        problem.AddParameterBlock(&end_t.x(), 3);
        break;
      case IcpModel::POINT_TO_PLANE:
        problem.AddParameterBlock(&end_quat.x(), 4, parameterization);
        problem.AddParameterBlock(&end_t.x(), 3);
        break;
      }

      std::vector<ceres::CostFunction *> surfFactor;
      std::vector<Eigen::Vector3d> normalVec;
      addSurfCostFactor(surfFactor, normalVec, surf_keypoints, p_frame);

      //   TODO: 退化后，该如何处理
      checkLocalizability(normalVec);

      int surf_num = 0;
      if (options_.log_print)
        std::cout << "get factor: " << surfFactor.size() << std::endl;
      // e:一个ICP点到平面匹配的残差项
      for (auto &e : surfFactor)
      {
        surf_num++;
        switch (options_.icpmodel)
        {
        case IcpModel::CT_POINT_TO_PLANE:
          problem.AddResidualBlock(e, loss_function, &begin_t.x(), &begin_quat.x(), &end_t.x(), &end_quat.x());
          break;
        case IcpModel::POINT_TO_PLANE:
          problem.AddResidualBlock(e, loss_function, &end_t.x(), &end_quat.x());
          break;
        }
        // if (surf_num > options_.max_num_residuals)
        //      break;
      }
      //   release
      std::vector<Eigen::Vector3d>().swap(normalVec);
      std::vector<ceres::CostFunction *>().swap(surfFactor);

      if (options_.icpmodel == IcpModel::CT_POINT_TO_PLANE)
      {
        if (options_.beta_location_consistency > 0.) //   location consistency
        {
#ifdef USE_ANALYTICAL_DERIVATE
          CT_ICP::LocationConsistencyFactor *cost_location_consistency =
              new CT_ICP::LocationConsistencyFactor(previous_translation, sqrt(surf_num * options_.beta_location_consistency * laser_point_cov));
#else
          auto *cost_location_consistency =
              CT_ICP::LocationConsistencyFunctor::Create(previous_translation, sqrt(surf_num * options_.beta_location_consistency));
#endif
          problem.AddResidualBlock(cost_location_consistency, nullptr, &begin_t.x());
        }

        if (options_.beta_orientation_consistency > 0.) // orientation consistency
        {
#ifdef USE_ANALYTICAL_DERIVATE
          CT_ICP::RotationConsistencyFactor *cost_rotation_consistency =
              new CT_ICP::RotationConsistencyFactor(previous_orientation, sqrt(surf_num * options_.beta_orientation_consistency * laser_point_cov));
#else
          auto *cost_rotation_consistency =
              CT_ICP::OrientationConsistencyFunctor::Create(previous_orientation, sqrt(surf_num * options_.beta_orientation_consistency));
#endif
          problem.AddResidualBlock(cost_rotation_consistency, nullptr, &begin_quat.x());
        }

        if (options_.beta_small_velocity > 0.) //     small velocity
        {
#ifdef USE_ANALYTICAL_DERIVATE
          CT_ICP::SmallVelocityFactor *cost_small_velocity =
              new CT_ICP::SmallVelocityFactor(sqrt(surf_num * options_.beta_small_velocity * laser_point_cov));
#else
          auto *cost_small_velocity =
              CT_ICP::SmallVelocityFunctor::Create(sqrt(surf_num * options_.beta_small_velocity));
#endif
          problem.AddResidualBlock(cost_small_velocity, nullptr, &begin_t.x(), &end_t.x());
        }

        // if (options_.beta_constant_velocity > 0.) //  const velocity
        // {
        //      CT_ICP::VelocityConsistencyFactor2 *cost_velocity_consistency =
        //          new CT_ICP::VelocityConsistencyFactor2(previous_velocity, sqrt(surf_num * options_.beta_constant_velocity * laser_point_cov));
        //      problem.AddResidualBlock(cost_velocity_consistency, nullptr, PR_begin, PR_end);
        // }
      }

      if (surf_num < options_.min_num_residuals)
      {
        std::stringstream ss_out;
        ss_out << "[Optimization] Error : not enough keypoints selected in ct-icp !" << std::endl;
        ss_out << "[Optimization] number_of_residuals : " << surf_num << std::endl;
        std::cout << "ERROR: " << ss_out.str();
      }

      ceres::Solver::Options options;
      options.max_num_iterations = 5;
      options.num_threads = 3;
      options.minimizer_progress_to_stdout = false;
      options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;

      // ceres::Solver::Options options;
      // options.linear_solver_type = ceres::DENSE_SCHUR;
      // options.trust_region_strategy_type = ceres::DOGLEG;
      // options.max_num_iterations = 10;
      // options.minimizer_progress_to_stdout = false;
      // options.num_threads = 6;

      ceres::Solver::Summary summary;

      ceres::Solve(options, &problem, &summary);

      if (!summary.IsSolutionUsable())
      {
        std::cout << summary.FullReport() << std::endl;
        throw std::runtime_error("Error During Optimization");
      }

      begin_quat.normalize();
      end_quat.normalize();

      double diff_trans = 0, diff_rot = 0;
      diff_trans += (current_state->translation_begin - begin_t).norm();
      diff_rot += AngularDistance(current_state->rotation_begin, begin_quat);

      diff_trans += (current_state->translation - end_t).norm();
      diff_rot += AngularDistance(current_state->rotation, end_quat);

      if (options_.icpmodel == IcpModel::CT_POINT_TO_PLANE)
      {
        p_frame->p_state->translation_begin = begin_t;
        p_frame->p_state->rotation_begin = begin_quat;
        p_frame->p_state->translation = end_t;
        p_frame->p_state->rotation = end_quat;

        current_state->translation_begin = begin_t;
        current_state->translation = end_t;
        current_state->rotation_begin = begin_quat;
        current_state->rotation = end_quat;
      }
      if (options_.icpmodel == IcpModel::POINT_TO_PLANE)
      {
        p_frame->p_state->translation = end_t;
        p_frame->p_state->rotation = end_quat;

        current_state->translation = end_t;
        current_state->rotation = end_quat;
      }

      if (diff_rot < options_.thres_orientation_norm &&
          diff_trans < options_.thres_translation_norm)
      {
        if (options_.log_print)
          std::cout << "Optimization: Finished with N=" << iter << " ICP iterations" << std::endl;
        break;
      }
    }
    std::vector<point3D>().swap(surf_keypoints);
    if (options_.log_print)
    {
      std::cout << "opt: " << p_frame->p_state->translation_begin.transpose()
                << ",end: " << p_frame->p_state->translation.transpose() << std::endl;
    }

    //   transpose point before added
    transformKeypoints(p_frame->point_surf);
  }

  double lidarodom::checkLocalizability(std::vector<Eigen::Vector3d> planeNormals)
  {
    //   使用参与计算的法向量分布，进行退化检测，若全是平面场景，则z轴的奇异值会特别小；一般使用小于3/4即可
    Eigen::MatrixXd mat;
    if (planeNormals.size() > 10)
    {
      mat.setZero(planeNormals.size(), 3);
      for (int i = 0; i < planeNormals.size(); i++)
      {
        mat(i, 0) = planeNormals[i].x();
        mat(i, 1) = planeNormals[i].y();
        mat(i, 2) = planeNormals[i].z();
      }
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(planeNormals.size(), 3);
      svd.compute(mat);
      if (svd.singularValues().z() < 3.5)
        std::cout << ANSI_COLOR_YELLOW << "Low convincing result -> singular values:"
                  << svd.singularValues().x() << ", " << svd.singularValues().y() << ", "
                  << svd.singularValues().z() << ANSI_COLOR_RESET << std::endl;
      return svd.singularValues().z();
    }
    else
    {
      std::cout << ANSI_COLOR_RED << "Too few normal vector received -> " << planeNormals.size() << ANSI_COLOR_RESET << std::endl;
      return -1;
    }
  }

  // 算出该点的质心、法向量、协方差矩阵和a2D
  Neighborhood lidarodom::computeNeighborhoodDistribution(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points)
  {
    Neighborhood neighborhood;
    // Compute the normals
    Eigen::Vector3d barycenter(Eigen::Vector3d(0, 0, 0));
    for (auto &point : points)
    {
      barycenter += point;
    }

    barycenter /= (double)points.size();
    neighborhood.center = barycenter;

    Eigen::Matrix3d covariance_Matrix(Eigen::Matrix3d::Zero());
    for (auto &point : points)
    {
      for (int k = 0; k < 3; ++k)
        for (int l = k; l < 3; ++l)
          covariance_Matrix(k, l) += (point(k) - barycenter(k)) *
                                     (point(l) - barycenter(l));
    }
    covariance_Matrix(1, 0) = covariance_Matrix(0, 1);
    covariance_Matrix(2, 0) = covariance_Matrix(0, 2);
    covariance_Matrix(2, 1) = covariance_Matrix(1, 2);
    neighborhood.covariance = covariance_Matrix;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance_Matrix);
    Eigen::Vector3d normal(es.eigenvectors().col(0).normalized());
    neighborhood.normal = normal;

    double sigma_1 = sqrt(std::abs(es.eigenvalues()[2]));
    double sigma_2 = sqrt(std::abs(es.eigenvalues()[1]));
    double sigma_3 = sqrt(std::abs(es.eigenvalues()[0]));
    neighborhood.a2D = (sigma_2 - sigma_3) / sigma_1;

    if (neighborhood.a2D != neighborhood.a2D)
    {
      throw std::runtime_error("error");
    }

    return neighborhood;
  }

  void lidarodom::addSurfCostFactor(std::vector<ceres::CostFunction *> &surf, std::vector<Eigen::Vector3d> &normals,
                                    std::vector<point3D> &keypoints, const cloudFrame *p_frame)
  {

    auto estimatePointNeighborhood = [&](std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &vector_neighbors,
                                         Eigen::Vector3d &location, double &planarity_weight)
    {
      auto neighborhood = computeNeighborhoodDistribution(vector_neighbors);
      planarity_weight = std::pow(neighborhood.a2D, options_.power_planarity);

      if (neighborhood.normal.dot(p_frame->p_state->translation_begin - location) < 0)
      {
        neighborhood.normal = -1.0 * neighborhood.normal;
      }
      return neighborhood;
    };

    double lambda_weight = std::abs(options_.weight_alpha);
    double lambda_neighborhood = std::abs(options_.weight_neighborhood);
    const double kMaxPointToPlane = options_.max_dist_to_plane_icp;
    const double sum = lambda_weight + lambda_neighborhood;

    lambda_weight /= sum;
    lambda_neighborhood /= sum;

    const short nb_voxels_visited = p_frame->frame_id < options_.init_num_frames
                                        ? 2
                                        : options_.voxel_neighborhood;

    const int kThresholdCapacity = p_frame->frame_id < options_.init_num_frames
                                       ? 1
                                       : options_.threshold_voxel_occupancy;

    size_t num = keypoints.size();
    int num_residuals = 0;

    for (int k = 0; k < num; k++)
    {
      auto &keypoint = keypoints[k];
      auto &raw_point = keypoint.raw_point;

      std::vector<voxel> voxels;
      auto vector_neighbors = searchNeighbors(voxel_map, keypoint.point,
                                              nb_voxels_visited,
                                              options_.size_voxel_map,
                                              options_.max_number_neighbors,
                                              kThresholdCapacity,
                                              options_.estimate_normal_from_neighborhood
                                                  ? nullptr
                                                  : &voxels);

      if (vector_neighbors.size() < options_.min_number_neighbors)
      {
        std::cout << "Too few neighbors -> " << vector_neighbors.size() << std::endl;
        continue;
      }

      double weight;

      Eigen::Vector3d location = TIL_ * raw_point;

      auto neighborhood = estimatePointNeighborhood(vector_neighbors, location /*raw_point*/, weight);
      // 平面性权重（planarity weight）
      // 邻近性权重（neighborhood weight）
      weight = lambda_weight * weight + lambda_neighborhood *
                                            std::exp(-(vector_neighbors[0] -
                                                       keypoint.point)
                                                          .norm() /
                                                     (kMaxPointToPlane *
                                                      options_.min_number_neighbors));

      double point_to_plane_dist;
      std::set<voxel> neighbor_voxels;
      for (int i(0); i < options_.num_closest_neighbors; ++i)
      {
        point_to_plane_dist = std::abs((keypoint.point - vector_neighbors[i]).transpose() * neighborhood.normal);

        if (point_to_plane_dist < options_.max_dist_to_plane_icp)
        {

          num_residuals++;

          Eigen::Vector3d norm_vector = neighborhood.normal;
          norm_vector.normalize();

          normals.push_back(norm_vector); //   record normal
          // 平面方程的偏移量
          double norm_offset = -norm_vector.dot(vector_neighbors[i]);

          switch (options_.icpmodel)
          {
          case IcpModel::CT_POINT_TO_PLANE:
          {
#ifdef USE_ANALYTICAL_DERIVATE
            CT_ICP::CTLidarPlaneNormFactor *cost_function =
                new CT_ICP::CTLidarPlaneNormFactor(keypoints[k].raw_point, norm_vector, norm_offset, keypoints[k].alpha_time, weight);
#else
            auto *cost_function = CT_ICP::CTPointToPlaneFunctor::Create(vector_neighbors[0],
                                                                        keypoints[k].raw_point,
                                                                        norm_vector,
                                                                        keypoints[k].alpha_time,
                                                                        weight);
#endif
            surf.push_back(cost_function);
            // problem.AddResidualBlock(cost_function, loss_function, &begin_t.x(), &begin_quat.x(), &end_t.x(), &end_quat.x());
            break;
          }
          case IcpModel::POINT_TO_PLANE:
          {
            Eigen::Vector3d point_end = p_frame->p_state->rotation.inverse() * keypoints[k].point -
                                        p_frame->p_state->rotation.inverse() * p_frame->p_state->translation;
#ifdef USE_ANALYTICAL_DERIVATE
            CT_ICP::LidarPlaneNormFactor *cost_function =
                new CT_ICP::LidarPlaneNormFactor(point_end, norm_vector, norm_offset, weight);
#else
            auto *cost_function = CT_ICP::PointToPlaneFunctor::Create(vector_neighbors[0],
                                                                      point_end, norm_vector, weight);
#endif
            surf.push_back(cost_function);
            // problem.AddResidualBlock(cost_function, loss_function, &end_t.x(), &end_quat.x());
            break;
          }
          }
        }
      }

      if (num_residuals >= options_.max_num_residuals)
        break;
    }
  }

  ///  ===================  for search neighbor  ===================================================
  using pair_distance_t = std::tuple<double, Eigen::Vector3d, voxel>;

  struct comparator
  {
    bool operator()(const pair_distance_t &left, const pair_distance_t &right) const
    {
      return std::get<0>(left) < std::get<0>(right);
    }
  };

  using priority_queue_t = std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, comparator>;

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
  lidarodom::searchNeighbors(const voxelHashMap &map, const Eigen::Vector3d &point,
                             int nb_voxels_visited, double size_voxel_map,
                             int max_num_neighbors, int threshold_voxel_capacity,
                             std::vector<voxel> *voxels)
  {

    if (voxels != nullptr)
      voxels->reserve(max_num_neighbors);

    short kx = static_cast<short>(point[0] / size_voxel_map);
    short ky = static_cast<short>(point[1] / size_voxel_map);
    short kz = static_cast<short>(point[2] / size_voxel_map);

    priority_queue_t priority_queue;
    // 遍历邻近nb_voxels_visited的体素，筛去点云少于threshold_voxel_capacity的体素，维护一个最大堆，把距离最小的max_num_neighbors个点云入队
    voxel voxel_temp(kx, ky, kz);
    for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx)
    {
      for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy)
      {
        for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz)
        {
          voxel_temp.x = kxx;
          voxel_temp.y = kyy;
          voxel_temp.z = kzz;

          auto search = map.find(voxel_temp);
          if (search != map.end())
          {
            const auto &voxel_block = search.value();
            if (voxel_block.NumPoints() < threshold_voxel_capacity)
              continue;
            for (int i(0); i < voxel_block.NumPoints(); ++i)
            {
              auto &neighbor = voxel_block.points[i];
              double distance = (neighbor - point).norm();
              if (priority_queue.size() == max_num_neighbors)
              {
                if (distance < std::get<0>(priority_queue.top()))
                {
                  priority_queue.pop();
                  priority_queue.emplace(distance, neighbor, voxel_temp);
                }
              }
              else
                priority_queue.emplace(distance, neighbor, voxel_temp);
            }
          }
        }
      }
    }

    auto size = priority_queue.size();
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> closest_neighbors(size);
    if (voxels != nullptr)
    {
      voxels->resize(size);
    }
    // 从小到大取出点云以及体素
    for (auto i = 0; i < size; ++i)
    {
      closest_neighbors[size - 1 - i] = std::get<1>(priority_queue.top());
      if (voxels != nullptr)
        (*voxels)[size - 1 - i] = std::get<2>(priority_queue.top());
      priority_queue.pop();
    }

    return closest_neighbors;
  }

  void lidarodom::addPointToMap(voxelHashMap &map, const Eigen::Vector3d &point,
                                double voxel_size, int max_num_points_in_voxel,
                                double min_distance_points, int min_num_points)
  {
    short kx = static_cast<short>(point[0] / voxel_size);
    short ky = static_cast<short>(point[1] / voxel_size);
    short kz = static_cast<short>(point[2] / voxel_size);

    voxelHashMap::iterator search = map.find(voxel(kx, ky, kz));

    if (search != map.end())
    {
      auto &voxel_block = (search.value());

      if (!voxel_block.IsFull())
      {
        // 计算新点与应插入的点云内所有点的距离，取最小的距离，如果距离小于阈值，则不插入
        double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
        for (int i(0); i < voxel_block.NumPoints(); ++i)
        {
          auto &_point = voxel_block.points[i];
          double sq_dist = (_point - point).squaredNorm();
          if (sq_dist < sq_dist_min_to_points)
          {
            sq_dist_min_to_points = sq_dist;
          }
        }
        if (sq_dist_min_to_points > (min_distance_points * min_distance_points))
        {
          if (min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points)
          {
            voxel_block.AddPoint(point);
            // addPointToPcl(points_world, point, intensity, p_frame);
          }
        }
      }
    }
    else
    {
      // 新点所在体素块不存在，创建新的体素块并插入到map
      if (min_num_points <= 0)
      {
        voxelBlock block(max_num_points_in_voxel);
        block.AddPoint(point);
        map[voxel(kx, ky, kz)] = std::move(block);
      }
    }
  }

  bool lidarodom::shouldExcludePointFromMap(const point3D &point) const
  {
    if (!options_.map_rear_filter_enabled)
    {
      return false;
    }

    // raw_point stays in the local lidar frame after motion compensation, so
    // this box only affects map insertion around the robot instead of global
    // world coordinates.
    const Eigen::Vector3d &local_point = point.raw_point;
    return local_point.x() >= options_.map_rear_filter_x_min &&
           local_point.x() <= options_.map_rear_filter_x_max &&
           local_point.y() >= options_.map_rear_filter_y_min &&
           local_point.y() <= options_.map_rear_filter_y_max &&
           local_point.z() >= options_.map_rear_filter_z_min &&
           local_point.z() <= options_.map_rear_filter_z_max;
  }

  void lidarodom::addPointToPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points, const Eigen::Vector3d &point, const double &intensity, cloudFrame *p_frame)
  {
    pcl::PointXYZI cloudTemp;

    cloudTemp.x = point.x();
    cloudTemp.y = point.y();
    cloudTemp.z = point.z();
    cloudTemp.intensity = intensity;
    // cloudTemp.intensity = 50 * (point.z() - p_frame->p_state->translation.z());
    pcl_points->points.push_back(cloudTemp);
  }

  CloudPtr lidarodom::buildGlobalMapCloudLocked() const
  {
    auto map_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    for (const auto &pair : voxel_map)
    {
      for (const auto &point : pair.second.points)
      {
        pcl::PointXYZI cloud_point;
        cloud_point.x = point.x();
        cloud_point.y = point.y();
        cloud_point.z = point.z();
        cloud_point.intensity = point.z();
        map_cloud->points.push_back(cloud_point);
      }
    }

    map_cloud->width = static_cast<uint32_t>(map_cloud->points.size());
    map_cloud->height = 1;
    map_cloud->is_dense = false;
    return map_cloud;
  }

  CloudPtr lidarodom::buildGlobalMapCloud()
  {
    std::lock_guard<std::mutex> lock(mtx_voxel_map_);
    return buildGlobalMapCloudLocked();
  }

  bool lidarodom::estimateGroundLevelRotation(const CloudPtr &cloud,
                                              Eigen::Matrix3d *R_out,
                                              double *tilt_deg_out) const
  {
    if (R_out)
    {
      *R_out = Eigen::Matrix3d::Identity();
    }
    if (tilt_deg_out)
    {
      *tilt_deg_out = 0.0;
    }
    if (!cloud || cloud->size() < 50)
    {
      return false;
    }

    // Voxel-downsample then RANSAC a near-horizontal plane. Low-z SVD alone
    // often locks onto a small flat patch and reports ~1° while the true floor
    // (dominant near-Z plane) is still ~13° vs the RViz Grid.
    constexpr double kVoxel = 0.08;
    std::unordered_map<voxel, Eigen::Vector3d, std::hash<voxel>> samples;
    samples.reserve(cloud->size() / 4);
    for (const auto &pt : cloud->points)
    {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      {
        continue;
      }
      const voxel v = voxel((int)std::floor(pt.x / kVoxel),
                            (int)std::floor(pt.y / kVoxel),
                            (int)std::floor(pt.z / kVoxel));
      if (samples.find(v) == samples.end())
      {
        samples.emplace(v, Eigen::Vector3d(pt.x, pt.y, pt.z));
      }
    }
    if (samples.size() < 80)
    {
      return false;
    }
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(samples.size());
    for (const auto &kv : samples)
    {
      pts.push_back(kv.second);
    }

    std::mt19937 rng(42);
    Eigen::Vector3d best_n = Eigen::Vector3d::UnitZ();
    int best_inliers = 0;
    const int iters = 400;
    const double dist_thr = 0.06;
    // Accept planes within 35° of horizontal as floor candidates.
    const double min_nz = std::cos(35.0 * 3.14159265358979323846 / 180.0);

    for (int it = 0; it < iters; ++it)
    {
      const int i0 = std::uniform_int_distribution<int>(0, (int)pts.size() - 1)(rng);
      const int i1 = std::uniform_int_distribution<int>(0, (int)pts.size() - 1)(rng);
      const int i2 = std::uniform_int_distribution<int>(0, (int)pts.size() - 1)(rng);
      if (i0 == i1 || i0 == i2 || i1 == i2)
      {
        continue;
      }
      const Eigen::Vector3d &p0 = pts[i0];
      const Eigen::Vector3d &p1 = pts[i1];
      const Eigen::Vector3d &p2 = pts[i2];
      Eigen::Vector3d n = (p1 - p0).cross(p2 - p0);
      if (n.norm() < 1e-8)
      {
        continue;
      }
      n.normalize();
      if (n.z() < 0.0)
      {
        n = -n;
      }
      if (n.z() < min_nz)
      {
        continue;
      }
      const double d = -n.dot(p0);
      int inl = 0;
      for (const auto &p : pts)
      {
        if (std::abs(n.dot(p) + d) <= dist_thr)
        {
          ++inl;
        }
      }
      if (inl > best_inliers)
      {
        best_inliers = inl;
        best_n = n;
      }
    }

    if (best_inliers < 60)
    {
      return false;
    }

    // Refine normal with SVD on RANSAC inliers (median plane offset).
    std::vector<double> projs;
    projs.reserve(pts.size());
    for (const auto &p : pts)
    {
      projs.push_back(-best_n.dot(p));
    }
    std::nth_element(projs.begin(), projs.begin() + static_cast<long>(projs.size() / 2),
                     projs.end());
    const double d_best = projs[projs.size() / 2];

    std::vector<Eigen::Vector3d> inlier_pts;
    inlier_pts.reserve(static_cast<size_t>(best_inliers));
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto &p : pts)
    {
      if (std::abs(best_n.dot(p) + d_best) <= dist_thr)
      {
        inlier_pts.push_back(p);
        mean += p;
      }
    }
    if (inlier_pts.size() >= 30)
    {
      mean /= static_cast<double>(inlier_pts.size());
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      for (const auto &p : inlier_pts)
      {
        const Eigen::Vector3d dp = p - mean;
        cov += dp * dp.transpose();
      }
      cov /= static_cast<double>(inlier_pts.size());
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
      if (solver.info() == Eigen::Success)
      {
        Eigen::Vector3d n = solver.eigenvectors().col(0);
        if (n.z() < 0.0)
        {
          n = -n;
        }
        if (n.z() >= min_nz)
        {
          best_n = n.normalized();
        }
      }
    }

    const double cos_tilt = std::clamp(best_n.z(), -1.0, 1.0);
    const double tilt_deg =
        std::acos(cos_tilt) * 57.2957795130823208768;
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(
        best_n, Eigen::Vector3d::UnitZ());
    q.normalize();
    if (R_out)
    {
      *R_out = q.toRotationMatrix();
    }
    if (tilt_deg_out)
    {
      *tilt_deg_out = tilt_deg;
    }
    return true;
  }

  void lidarodom::applyLevelRotationToCloud(const CloudPtr &cloud) const
  {
    if (!cloud || cloud->empty())
    {
      return;
    }
    if (R_level_.isIdentity(1e-8))
    {
      return;
    }
    for (auto &pt : cloud->points)
    {
      const Eigen::Vector3d p = R_level_ * Eigen::Vector3d(pt.x, pt.y, pt.z);
      pt.x = static_cast<float>(p.x());
      pt.y = static_cast<float>(p.y());
      pt.z = static_cast<float>(p.z());
    }
  }

  SE3 lidarodom::applyLevelRotationToPose(const SE3 &pose) const
  {
    const Eigen::Matrix3d R_new = R_level_ * pose.rotationMatrix();
    const Eigen::Vector3d t_new = R_level_ * pose.translation();
    return SE3(SO3(R_new), t_new);
  }

  CloudPtr lidarodom::leveledCloudCopy(const CloudPtr &cloud) const
  {
    if (!cloud)
    {
      return nullptr;
    }
    CloudPtr out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*cloud);
    if (options_.level_map_enable && level_map_locked_)
    {
      applyLevelRotationToCloud(out);
    }
    return out;
  }

  void lidarodom::maybeLockLevelRotationFromMap()
  {
    if (!options_.level_map_enable || !options_.level_map_on_publish)
    {
      return;
    }
    if (index_frame < options_.level_map_lock_frame)
    {
      return;
    }
    // Re-estimate periodically so early lock cannot freeze a bad residual.
    const int refresh_every = std::max(options_.level_map_lock_frame, 20);
    if (level_map_locked_ && (index_frame % refresh_every) != 0)
    {
      return;
    }

    CloudPtr map_cloud;
    {
      std::lock_guard<std::mutex> lock(mtx_voxel_map_);
      map_cloud = buildGlobalMapCloudLocked();
    }
    if (!map_cloud ||
        static_cast<int>(map_cloud->size()) < options_.level_map_min_points)
    {
      return;
    }

    Eigen::Matrix3d R_est = Eigen::Matrix3d::Identity();
    double tilt_deg = 0.0;
    if (!estimateGroundLevelRotation(map_cloud, &R_est, &tilt_deg))
    {
      return;
    }

    // Voxel map is stored unleveled; R_level_ is always absolute vs raw map.
    if (tilt_deg < options_.level_map_min_tilt_deg)
    {
      R_level_ = Eigen::Matrix3d::Identity();
    }
    else
    {
      R_level_ = R_est;
    }
    level_map_locked_ = true;
    const Vec3d rpy = RotationMatrixToRpyDeg(R_level_);
    std::cout << ANSI_COLOR_GREEN_BOLD
              << "[ct_lio] Updated map ground leveling. residual_tilt_deg="
              << tilt_deg << ", R_level rpy_deg=" << rpy.transpose()
              << ANSI_COLOR_RESET << std::endl;
  }

  bool lidarodom::saveMapToPcd(const std::string &file_path)
  {
    if (file_path.empty())
    {
      std::cout << "[ct_lio] saveMapToPcd skipped because file_path is empty." << std::endl;
      return false;
    }

    auto map_cloud = buildGlobalMapCloud();
    if (!map_cloud || map_cloud->empty())
    {
      std::cout << "[ct_lio] saveMapToPcd skipped because the map is empty." << std::endl;
      return false;
    }

    double tilt_before = 0.0;
    Eigen::Matrix3d R_save = Eigen::Matrix3d::Identity();
    if (options_.level_map_enable && options_.level_map_on_save)
    {
      if (estimateGroundLevelRotation(map_cloud, &R_save, &tilt_before))
      {
        if (tilt_before >= options_.level_map_min_tilt_deg)
        {
          R_level_ = R_save;
          level_map_locked_ = true;
          applyLevelRotationToCloud(map_cloud);
        }
        else
        {
          // Already flat enough; keep identity leveling.
          R_level_ = Eigen::Matrix3d::Identity();
          level_map_locked_ = true;
        }
      }
      else if (level_map_locked_)
      {
        applyLevelRotationToCloud(map_cloud);
      }
    }

    double tilt_after = 0.0;
    estimateGroundLevelRotation(map_cloud, nullptr, &tilt_after);
    std::cout << "[ct_lio] saveMapToPcd leveling: tilt_before_deg=" << tilt_before
              << ", tilt_after_deg=" << tilt_after
              << ", points=" << map_cloud->size() << std::endl;

    try
    {
      const std::filesystem::path output_path(file_path);
      if (output_path.has_parent_path())
      {
        std::filesystem::create_directories(output_path.parent_path());
      }
    }
    catch (const std::exception &e)
    {
      std::cout << "[ct_lio] Failed to prepare map save directory: " << e.what() << std::endl;
      return false;
    }

    const bool ok = pcl::io::savePCDFileBinary(file_path, *map_cloud) == 0;
    if (ok)
    {
      std::cout << "[ct_lio] Saved leveled map to " << file_path << std::endl;
    }
    return ok;
  }

  // 把p_frame中的点插入地图中并发布points_world，最后清空points_world
  void lidarodom::map_incremental(cloudFrame *p_frame, int min_num_points)
  {
    CloudPtr registered_cloud;
    CloudPtr scan_cloud;
    CloudPtr full_map_cloud;
    size_t filtered_points = 0;

    {
      std::lock_guard<std::mutex> lock(mtx_voxel_map_);
      points_world->clear();
      registered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
      registered_cloud->points.reserve(p_frame->point_surf.size());
      for (const auto &point : p_frame->point_surf)
      {
        addPointToPcl(points_world, point.point, point.intensity, p_frame);
        if (shouldExcludePointFromMap(point))
        {
          ++filtered_points;
          continue;
        }
        addPointToPcl(registered_cloud, point.point, point.intensity, p_frame);
        addPointToMap(voxel_map, point.point, options_.size_voxel_map,
                      options_.max_num_points_in_voxel,
                      options_.min_distance_points, min_num_points);
      }

      scan_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*points_world);
      if (options_.publish_full_map &&
          (index_frame % options_.publish_full_map_every_n_frames == 0))
      {
        full_map_cloud = buildGlobalMapCloudLocked();
      }
      points_world->clear();
    }

    maybeLockLevelRotationFromMap();

    if (pub_cloud_to_ros && scan_cloud && !scan_cloud->empty())
    {
      CloudPtr pub_scan = scan_cloud;
      if (options_.level_map_enable && options_.level_map_on_publish &&
          level_map_locked_)
      {
        pub_scan = leveledCloudCopy(scan_cloud);
      }
      std::string topic_name = "laser";
      pub_cloud_to_ros(topic_name, pub_scan, p_frame->time_frame_end);
    }

    if (pub_cloud_to_ros && registered_cloud && !registered_cloud->empty())
    {
      CloudPtr pub_registered = registered_cloud;
      if (options_.level_map_enable && options_.level_map_on_publish &&
          level_map_locked_)
      {
        pub_registered = leveledCloudCopy(registered_cloud);
      }
      std::string topic_name = "cloud_registered";
      pub_cloud_to_ros(topic_name, pub_registered, p_frame->time_frame_end);
    }

    if (options_.log_print && options_.map_rear_filter_enabled &&
        filtered_points > 0 && index_frame % 20 == 0)
    {
      std::cout << "[ct_lio] map_rear_filter skipped " << filtered_points
                << " rear-body points before map insertion on frame "
                << index_frame << std::endl;
    }

    if (pub_cloud_to_ros && full_map_cloud && !full_map_cloud->empty())
    {
      CloudPtr pub_map = full_map_cloud;
      if (options_.level_map_enable && options_.level_map_on_publish &&
          level_map_locked_)
      {
        pub_map = leveledCloudCopy(full_map_cloud);
      }
      std::string topic_name = "laser_map";
      pub_cloud_to_ros(topic_name, pub_map, p_frame->time_frame_end);
    }
  }

  // 删除过远的点云
  void lidarodom::lasermap_fov_segment()
  {
    //   use predict pose here
    Eigen::Vector3d location = current_state->translation;
    std::vector<voxel> voxels_to_erase;
    {
      std::lock_guard<std::mutex> lock(mtx_voxel_map_);
      for (auto &pair : voxel_map)
      {
        Eigen::Vector3d pt = pair.second.points[0];
        if ((pt - location).squaredNorm() > (options_.max_distance * options_.max_distance))
        {
          voxels_to_erase.push_back(pair.first);
        }
      }
      for (auto &vox : voxels_to_erase)
      {
        voxel_map.erase(vox);
      }
    }
    std::vector<voxel>().swap(voxels_to_erase);
  }
  // CT模式下，线性插值来把每个点云转到这一帧的结束时刻
  cloudFrame *lidarodom::buildFrame(std::vector<point3D> &const_surf, state *cur_state,
                                    double timestamp_begin, double timestamp_end)
  {
    std::vector<point3D> frame_surf(const_surf);
    if (index_frame < 2)
    {
      for (auto &point_temp : frame_surf)
      {
        point_temp.alpha_time = 1.0; //  alpha reset 0
      }
    }

    if (options_.motion_compensation == CONSTANT_VELOCITY)
      Undistort(frame_surf);

    for (auto &point_temp : frame_surf)
      transformPoint(options_.motion_compensation, point_temp, cur_state->rotation_begin,
                     cur_state->rotation, cur_state->translation_begin, cur_state->translation,
                     R_imu_lidar, t_imu_lidar);

    cloudFrame *p_frame = new cloudFrame(frame_surf, const_surf, cur_state);

    p_frame->time_frame_begin = timestamp_begin;
    p_frame->time_frame_end = timestamp_end;

    p_frame->dt_offset = 0;

    p_frame->frame_id = index_frame;

    return p_frame;
  }

  void lidarodom::stateInitialization()
  {
    if (index_frame < 2) //   only first frame
    {
      current_state->rotation_begin = Eigen::Quaterniond(imu_states_.front().R_.matrix());
      current_state->translation_begin = imu_states_.front().p_;
      current_state->rotation = Eigen::Quaterniond(imu_states_.back().R_.matrix());
      current_state->translation = imu_states_.back().p_;
    }
    else
    {
      //   use last pose
      current_state->rotation_begin = all_state_frame[all_state_frame.size() - 1]->rotation;
      current_state->translation_begin = all_state_frame[all_state_frame.size() - 1]->translation;
      // current_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
      // current_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
      //   use imu predict
      current_state->rotation = Eigen::Quaterniond(imu_states_.back().R_.matrix());
      current_state->translation = imu_states_.back().p_;
      // current_state->rotation = q_next_end;
      // current_state->translation = t_next_end;
    }
  }

  std::vector<MeasureGroup> lidarodom::getMeasureMents()
  {
    std::vector<MeasureGroup> measurements;
    while (true)
    {
      if (imu_buffer_.empty())
        return measurements;

      if (lidar_buffer_.empty())
        return measurements;

      if (imu_buffer_.back()->timestamp_ - time_curr < delay_time_)
        return measurements;

      MeasureGroup meas;

      meas.lidar_ = lidar_buffer_.front();
      meas.lidar_begin_time_ = time_buffer_.front().first;
      meas.lidar_end_time_ = meas.lidar_begin_time_ + time_buffer_.front().second;
      lidar_buffer_.pop_front();
      time_buffer_.pop_front();

      time_curr = meas.lidar_end_time_;

      double imu_time = imu_buffer_.front()->timestamp_;
      meas.imu_.clear();
      while ((!imu_buffer_.empty()) && (imu_time < meas.lidar_end_time_))
      {
        imu_time = imu_buffer_.front()->timestamp_;
        if (imu_time > meas.lidar_end_time_)
        {
          break;
        }
        meas.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
      }

      if (!imu_buffer_.empty())
        meas.imu_.push_back(imu_buffer_.front()); //   added for Interp

      measurements.push_back(meas);
    }
  }

  void lidarodom::Predict()
  {
    imu_states_.emplace_back(eskf_.GetNominalState());

    /// 对IMU状态进行预测
    double time_current = measures_.lidar_end_time_;
    Vec3d last_gyr, last_acc;
    for (auto &imu : measures_.imu_)
    {
      double time_imu = imu->timestamp_;
      if (imu->timestamp_ <= time_current)
      {
        if (last_imu_ == nullptr)
          last_imu_ = imu;
        eskf_.Predict(*imu);
        imu_states_.emplace_back(eskf_.GetNominalState());
        last_imu_ = imu;
      }
      else
      {
        double dt_1 = time_imu - time_current;
        double dt_2 = time_current - last_imu_->timestamp_;
        double w1 = dt_1 / (dt_1 + dt_2);
        double w2 = dt_2 / (dt_1 + dt_2);
        Eigen::Vector3d acc_temp = w1 * last_imu_->acce_ + w2 * imu->acce_;
        Eigen::Vector3d gyr_temp = w1 * last_imu_->gyro_ + w2 * imu->gyro_;
        IMUPtr imu_temp = std::make_shared<zjloc::IMU>(time_current, gyr_temp, acc_temp);
        eskf_.Predict(*imu_temp);
        imu_states_.emplace_back(eskf_.GetNominalState());
        last_imu_ = imu_temp;
      }
    }
  }
  // 去运动畸变
  void lidarodom::Undistort(std::vector<point3D> &points)
  {
    // auto &cloud = measures_.lidar_;
    auto imu_state = eskf_.GetNominalState(); // 最后时刻的状态
    // std::cout << __FUNCTION__ << ", " << imu_state.timestamp_ << std::endl;
    SE3 T_end = SE3(imu_state.R_, imu_state.p_);

    /// 将所有点转到最后时刻状态上
    for (auto &pt : points)
    {
      SE3 Ti = T_end;
      NavStated match;

      // 根据pt.time查找时间，pt.time是该点打到的时间与雷达开始时间之差，单位为毫秒
      math::PoseInterp<NavStated>(
          pt.timestamp, imu_states_, [](const NavStated &s)
          { return s.timestamp_; },
          [](const NavStated &s)
          { return s.GetSE3(); },
          Ti, match);

      pt.raw_point = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pt.raw_point;
    }
  }

  void lidarodom::TryInitIMU()
  {
    for (auto imu : measures_.imu_)
    {
      imu_init_.AddIMU(*imu);
    }

    if (imu_init_.InitSuccess())
    {
      // 读取初始零偏，设置ESKF
      zjloc::ESKFD::Options options;
      // 噪声由初始化器估计
      // options.gyro_var_ = sqrt(imu_init_.GetCovGyro()[0]);
      // options.acce_var_ = sqrt(imu_init_.GetCovAcce()[0]);
      // options.update_bias_acce_ = false;
      // options.update_bias_gyro_ = false;
      const Vec3d mean_acce = imu_init_.GetMeanAcce();
      const double gravity_norm = imu_init_.GetGravity().norm();
      const Vec3d gravity_world(0.0, 0.0, -gravity_norm);
      // Align body +mean_acce (specific force / "up") to world +Z so map XY is horizontal.
      // Optional: skip for sensors whose lidar frame is already level (e.g. Odin1).
      const SO3 init_rotation = imu_gravity_level_enable_
                                    ? BuildInitialOrientationFromMeanAcce(mean_acce)
                                    : SO3();
      const Vec3d init_rpy_deg = RotationMatrixToRpyDeg(init_rotation.matrix());
      const double init_timestamp =
          measures_.imu_.empty() ? -1.0 : measures_.imu_.back()->timestamp_;

      eskf_.SetInitialConditions(
          options,
          imu_init_.GetInitBg(),
          imu_init_.GetInitBa(),
          gravity_world,
          init_rotation,
          init_timestamp);
      if (!measures_.imu_.empty())
      {
        last_imu_ = measures_.imu_.back();
      }
      LOG(INFO) << "ESKF init leveling, mean acce(body) = " << mean_acce.transpose()
                << ", gravity(world) = " << gravity_world.transpose()
                << ", gravity_level_en = " << imu_gravity_level_enable_
                << ", init rpy deg = " << init_rpy_deg.transpose()
                << ", init ba = " << imu_init_.GetInitBa().transpose()
                << ", init time = " << init_timestamp;
      imu_need_init_ = false;

      std::cout << ANSI_COLOR_GREEN_BOLD << "IMU初始化成功 (gravity_level="
                << (imu_gravity_level_enable_ ? "on" : "off") << ", rpy deg "
                << init_rpy_deg.transpose() << ")" << ANSI_COLOR_RESET << std::endl;
    }
  }

}
