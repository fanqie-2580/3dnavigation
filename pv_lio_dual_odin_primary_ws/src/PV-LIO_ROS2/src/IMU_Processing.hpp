#include <cmath>
#include <math.h>
#include <numeric>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "use-ikfom.hpp"

/// *************Preconfiguration

#define MAX_INI_COUNT (20)

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::SharedPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void set_odin_imu_extrinsic(const V3D &transl, const M3D &rot);
  // Deskew Odin cloud into Odin-lidar frame at target_end_time (Odin clock).
  // target_end_time < 0 → use this scan's own end (beg+span).
  // For dual fuse: pass Mid-end mapped onto Odin clock (mid_end - stamp_bias)
  // so Odin and Mid share the same end pose before T_L2 / world insert.
  bool UndistortOdinNative(PointCloudXYZI &cloud, const MeasureGroup &meas,
                           const V3D &bg, const V3D &ba,
                           double target_end_time = -1.0);
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

  ofstream fout_imu;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;

  SO3 Initial_R_wrt_G;

 private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;
  sensor_msgs::msg::Imu::SharedPtr last_imu_;
  deque<sensor_msgs::msg::Imu::SharedPtr> v_imu_;
  vector<Pose6D> IMUpose;
  vector<M3D>    v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  M3D Odin_Lidar_R_wrt_IMU;
  V3D Odin_Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double start_timestamp_;
  double last_lidar_end_time_;
  int    init_iter_num = 1;
  bool   b_first_frame_ = true;
  bool   imu_need_init_ = true;
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;
  Q = process_noise_cov();
  cov_acc       = V3D(0.1, 0.1, 0.1);
  cov_gyr       = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last     = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  Odin_Lidar_R_wrt_IMU = Eye3d;
  Odin_Lidar_T_wrt_IMU = Zero3d;
  last_imu_.reset(new sensor_msgs::msg::Imu());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last       = Zero3d;
  imu_need_init_    = true;
  start_timestamp_  = -1;
  init_iter_num     = 1;
  v_imu_.clear();
  IMUpose.clear();
  last_imu_.reset(new sensor_msgs::msg::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_extrinsic(const MD(4,4) &T)
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

void ImuProcess::set_odin_imu_extrinsic(const V3D &transl, const M3D &rot)
{
  Odin_Lidar_T_wrt_IMU = transl;
  Odin_Lidar_R_wrt_IMU = rot;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  
  V3D cur_acc, cur_gyr;
  
  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    first_lidar_time = meas.lidar_beg_time;
  }
  //计算线角速度和角加速度协方差
  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N ++;
  }
  state_ikfom init_state = kf_state.get_x();
  // Gravity-align Mid360/IMU world so CloudMap is level vs RViz Grid.
  // Mid is pitched ~8–13° on the robot; leaving rot=I makes camera_init look tilted.
  // Match FAST_LIO_MULTI mid_odin IMU init (not stock PV-LIO).
  {
    V3D gz_body = (-mean_acc / mean_acc.norm());
    Eigen::Quaterniond q_align =
        Eigen::Quaterniond::FromTwoVectors(gz_body, V3D(0.0, 0.0, -1.0));
    init_state.rot = SO3(q_align.toRotationMatrix());
    init_state.grav = S2(V3D(0.0, 0.0, -G_m_s2));
  }
  init_state.bg  = mean_gyr;
  init_state.offset_T_L_I = Lidar_T_wrt_IMU; 
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state);

  // Keep TF world->camera_init identity-ish after rot already gravity-aligned.
  Initial_R_wrt_G = SO3(Eye3d);

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  // init_P.setZero();
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
  init_P(21,21) = init_P(22,22) = 0.00001; 
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();

}


bool ImuProcess::UndistortOdinNative(PointCloudXYZI &cloud, const MeasureGroup &meas,
                                     const V3D &bg, const V3D &ba,
                                     double target_end_time)
{
  if (cloud.points.empty() || meas.imu2.size() < 2)
    return false;
  constexpr double kMaxDt = 0.20;
  const double beg = meas.lidar_beg_time2;
  double span = 0.0;
  for (const auto &pt : cloud.points)
    span = std::max(span, static_cast<double>(pt.curvature) / 1000.0);
  if (span < 1e-4)
    return false;
  const double scan_end = beg + span;
  // Reference pose time: Mid-end on Odin clock when fusing, else this scan end.
  // Root cause of turn smear: deskew-to-Odin-end then world-transform with
  // Mid-end pose → residual Δt × yaw paints old Odin geometry at wrong heading.
  double end = scan_end;
  if (target_end_time > 0.0)
  {
    // Always chase Mid-end when within a scan period; previously a 0.15s gate
    // plus "end<=beg → fall back to scan_end" silently skipped Mid-end align
    // whenever stamp bias pushed the target before Odin beg.
    if (target_end_time >= beg - 0.02 && target_end_time <= scan_end + 0.20)
      end = std::max(beg + 1e-4, target_end_time);
  }
  if (end <= beg + 1e-4)
    end = scan_end;
  auto v_imu = meas.imu2;
  while (v_imu.size() > 2 &&
         rclcpp::Time(v_imu[1]->header.stamp).seconds() < beg - 1e-4)
    v_imu.pop_front();
  // Gyro-only deskew: measured /odin1/imu az sd≈1.5 (Mid ≈0.01) made accel
  // integration pump stationary body-cloud Z by tens of cm. Keep Mid-end time
  // align via rotation only (turn smear fix); ignore Odin accel/ba.
  (void)ba;
  const M3D R_LI = Odin_Lidar_R_wrt_IMU;
  const V3D T_LI = Odin_Lidar_T_wrt_IMU;
  const M3D R_IL = R_LI.transpose();
  std::vector<Pose6D> poses;
  poses.reserve(v_imu.size() + 2);
  M3D R = Eye3d;
  V3D pos = Zero3d, vel = Zero3d, angvel = Zero3d, acc_s = Zero3d;
  double last_t = beg;
  poses.push_back(set_pose6d(0.0, acc_s, angvel, vel, pos, R));
  for (size_t i = 0; i + 1 < v_imu.size(); ++i)
  {
    const auto &head = v_imu[i];
    const auto &tail = v_imu[i + 1];
    const double t_head = rclcpp::Time(head->header.stamp).seconds();
    double t_tail = rclcpp::Time(tail->header.stamp).seconds();
    if (t_tail < beg) continue;
    V3D w, a;
    w << .5*(head->angular_velocity.x+tail->angular_velocity.x),
         .5*(head->angular_velocity.y+tail->angular_velocity.y),
         .5*(head->angular_velocity.z+tail->angular_velocity.z);
    a = Zero3d;
    w -= bg;
    bool hit_end = false;
    if (t_tail > end) { t_tail = end; hit_end = true; }
    const double dt = t_tail - std::max(t_head, last_t);
    if (dt <= 1e-9) { if (hit_end) break; continue; }
    R = R * Exp(w, dt);
    const V3D aw = R * a;
    pos += vel * dt + .5 * aw * dt * dt;
    vel += aw * dt; angvel = w; acc_s = aw; last_t = t_tail;
    poses.push_back(set_pose6d(last_t - beg, acc_s, angvel, vel, pos, R));
    if (hit_end || t_tail >= end - 1e-9) break;
  }
  if (end - last_t > 1e-6 && !poses.empty())
  {
    const double dt = std::min(kMaxDt, end - last_t);
    R = R * Exp(angvel, dt);
    pos += vel * dt + .5 * acc_s * dt * dt;
    last_t = end;
    poses.push_back(set_pose6d(end - beg, acc_s, angvel, vel, pos, R));
  }
  if (poses.size() <= 1) return false;
  const M3D R_end = R;
  const V3D pos_end = pos;
  const M3D R_end_T = R_end.transpose();
  std::vector<int> order(cloud.points.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](int a, int b){return cloud.points[a].curvature < cloud.points[b].curvature;});
  auto apply_pose = [&](PointType &pt, double t_off, const M3D &R0,
                        const V3D &pos0, const V3D &vel0, const V3D &acc0,
                        const V3D &gyr0, double t0) {
    const double dt = t_off - t0;
    if (std::fabs(dt) > kMaxDt) return;
    const M3D R_i = R0 * Exp(gyr0, dt);
    const V3D P_i(pt.x, pt.y, pt.z);
    const V3D T_ei = pos0 + vel0 * dt + .5 * acc0 * dt * dt - pos_end;
    const V3D P = R_IL * (R_end_T * (R_i * (R_LI * P_i + T_LI) + T_ei) - T_LI);
    pt.x = P(0); pt.y = P(1); pt.z = P(2);
  };
  size_t idx = order.size();
  M3D R_imu; V3D vel_imu, pos_imu, acc_imu, gyr_imu;
  for (auto it = poses.end() - 1; it != poses.begin(); --it)
  {
    auto head = it - 1, tail = it;
    R_imu << MAT_FROM_ARRAY(head->rot); vel_imu << VEC_FROM_ARRAY(head->vel);
    pos_imu << VEC_FROM_ARRAY(head->pos); acc_imu << VEC_FROM_ARRAY(tail->acc);
    gyr_imu << VEC_FROM_ARRAY(tail->gyr);
    while (idx > 0)
    {
      auto &pt = cloud.points[order[idx - 1]];
      const double t = static_cast<double>(pt.curvature) / 1000.0;
      if (t <= head->offset_time) break;
      --idx;
      apply_pose(pt, t, R_imu, pos_imu, vel_imu, acc_imu, gyr_imu, head->offset_time);
    }
  }
  if (idx > 0)
  {
    auto head = poses.begin();
    R_imu << MAT_FROM_ARRAY(head->rot); vel_imu << VEC_FROM_ARRAY(head->vel);
    pos_imu << VEC_FROM_ARRAY(head->pos); acc_imu << VEC_FROM_ARRAY(head->acc);
    gyr_imu << VEC_FROM_ARRAY(head->gyr);
    for (size_t i=0;i<idx;i++)
    {
      auto &pt=cloud.points[order[i]];
      const double t=static_cast<double>(pt.curvature)/1000.0;
      apply_pose(pt,t,R_imu,pos_imu,vel_imu,acc_imu,gyr_imu,head->offset_time);
    }
  }
  return true;
}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  const double &imu_beg_time = rclcpp::Time(v_imu.front()->header.stamp).seconds();
  const double &imu_end_time = rclcpp::Time(v_imu.back()->header.stamp).seconds();
  const double &pcl_beg_time = meas.lidar_beg_time;
  const double &pcl_end_time = meas.lidar_end_time;
  
  /*** sort point clouds by offset time ***/
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

  /*** Initialize IMU pose ***/
  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  /*** forward propagation at each imu point ***/
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;

  double dt = 0;

  input_ikfom in;
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);
    
    if (rclcpp::Time(tail->header.stamp).seconds() < last_lidar_end_time_)    continue;
    
    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    // fout_imu << setw(10) << rclcpp::Time(head->header.stamp).seconds() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

    acc_avr     = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

    if(rclcpp::Time(head->header.stamp).seconds() < last_lidar_end_time_)
    {
      dt = rclcpp::Time(tail->header.stamp).seconds() - last_lidar_end_time_;
      // dt = rclcpp::Time(tail->header.stamp).seconds() - pcl_beg_time;
    }
    else
    {
      dt = rclcpp::Time(tail->header.stamp).seconds() - rclcpp::Time(head->header.stamp).seconds();
    }
    
    in.acc = acc_avr;
    in.gyro = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    // 这里应该判断是否有GPS measurements， 如果有GPS， 那么先传播到GPS，然后更新state，再重新从头开始向前传播
    // 看一下r2live的实现
    kf_state.predict(dt, Q, in);

    /* save the poses at each IMU measurements */
    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
    for(int i=0; i<3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];
    }
    double &&offs_t = rclcpp::Time(tail->header.stamp).seconds() - pcl_beg_time;
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);
  
  imu_state = kf_state.get_x();
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  /*反向传播*/
  /*** undistort each lidar point (backward propagation) ***/
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu<<MAT_FROM_ARRAY(head->rot);
    // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
    vel_imu<<VEC_FROM_ARRAY(head->vel);
    pos_imu<<VEC_FROM_ARRAY(head->pos);
    acc_imu<<VEC_FROM_ARRAY(tail->acc);
    angvel_avr<<VEC_FROM_ARRAY(tail->gyr);

    for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
    {
      dt = it_pcl->curvature / double(1000) - head->offset_time;

      /* Transform to the 'end' frame, using only the rotation
       * Note: Compensation direction is INVERSE of Frame's moving direction
       * So if we want to compensate a point at timestamp-i to the frame-e
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
      M3D R_i(R_imu * Exp(angvel_avr, dt));
      
      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
      V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);// not accurate!
      
      // save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) break;
    }
  }
}

void ImuProcess::Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1,t2,t3;
  t1 = omp_get_wtime();

  if(meas.imu.empty()) {return;};
  assert(meas.lidar != nullptr);

  if (imu_need_init_)
  {
    /// The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;
    
    last_imu_   = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();
    //
    if (init_iter_num > MAX_INI_COUNT)
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      RCLCPP_INFO(rclcpp::get_logger("pv_lio"), "IMU Initial Done");
      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"),ios::out);
    }

    return;
  }

  UndistortPcl(meas, kf_state, *cur_pcl_un_);
  // std::cout << "lidar: " << meas.lidar->points.size() << " undistort: " << cur_pcl_un_->points.size() << endl;

  t2 = omp_get_wtime();
  t3 = omp_get_wtime();
  
  // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}
