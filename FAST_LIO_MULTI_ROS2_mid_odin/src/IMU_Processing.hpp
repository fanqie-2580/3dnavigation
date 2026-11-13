#include <cmath>
#include <deque>
#include <numeric>
#include <vector>
#include <algorithm>
#include <csignal>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "so3_math.h"
#include "common_lib.h"
#include "use-ikfom.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>

/// *************Preconfiguration

#define MAX_INI_COUNT (10)

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  // Same convention as Bundle sync: physical Mid time ≈ L2_stamp + bias.
  void set_lidar2_stamp_bias(const double bias_sec) { lidar2_stamp_bias_ = bias_sec; }
  // Odin scan-local deskew on /odin1/imu, then one rigid T_L2_wrt_L1 into Mid L1.
  void set_odin_native_deskew(const bool en) { odin_native_deskew_ = en; }
  void set_lidar2_wrt_lidar1(const M3D &R, const V3D &T)
  {
    Lidar2_R_wrt_Lidar1_ = R;
    Lidar2_T_wrt_Lidar1_ = T;
  }
  void set_odin_imu_extrinsic(const V3D &transl, const M3D &rot)
  {
    Odin_Lidar_T_wrt_IMU_ = transl;
    Odin_Lidar_R_wrt_IMU_ = rot;
  }
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_, const bool &multi_lidar, int lidar_num = 1);
  bool IMU_inited() const { return !imu_need_init_; }

  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;

 private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out, int lidar_num = 1);
  void UndistortPclMultiLiDAR(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);
  // Deskew Odin cloud in its own lidar frame using imu2; leaves points at scan-end.
  bool UndistortOdinNative(PointCloudXYZI &cloud, const MeasureGroup &meas,
                           const V3D &bg, const V3D &ba);

  PointCloudXYZI::Ptr cur_pcl_un_;
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> v_imu_;
  vector<Pose6D> IMUpose;
  vector<M3D>    v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  M3D Lidar2_R_wrt_Lidar1_ = Eye3d;
  V3D Lidar2_T_wrt_Lidar1_ = Zero3d;
  M3D Odin_Lidar_R_wrt_IMU_ = Eye3d;
  V3D Odin_Lidar_T_wrt_IMU_ = Zero3d;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double start_timestamp_;
  double last_lidar_end_time_, last_lidar_end_time_2;
  double lidar2_stamp_bias_ = 0.0;
  bool   odin_native_deskew_ = false;
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
  last_lidar_end_time_ = 0.0;
  last_lidar_end_time_2 = 0.0;
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
  }

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
  // Odin cloud_raw is already near gravity-level; laserMapping compensates
  // LiDAR2_wrt_LiDAR1 by R_wi^T once after IMU_inited() so Odin is not re-tilted.
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

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
  init_P(21,21) = init_P(22,22) = 0.00001; 
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();

}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out, int lidar_num)
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  const double &imu_beg_time = get_time_sec(v_imu.front()->header.stamp);
  const double &imu_end_time = get_time_sec(v_imu.back()->header.stamp);
  const double &pcl_beg_time = lidar_num == 1 ? meas.lidar_beg_time : meas.lidar_beg_time2;
  const double &pcl_end_time = lidar_num == 1 ? meas.lidar_end_time : meas.lidar_end_time2;
  
  /*** sort point clouds by offset time ***/
  if (lidar_num == 2)
  {
    assert(meas.lidar2 != nullptr);
    pcl_out = *(meas.lidar2);
  }
  else
  {
    assert(meas.lidar != nullptr);
    pcl_out = *(meas.lidar);
  }
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
    
    if (get_time_sec(tail->header.stamp) < last_lidar_end_time_)    continue;
    
    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    acc_avr     = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

    if(get_time_sec(head->header.stamp) < last_lidar_end_time_)
    {
      dt = get_time_sec(tail->header.stamp) - last_lidar_end_time_;
      // dt = get_time_sec(tail->header.stamp) - pcl_beg_time;
    }
    else
    {
      dt = get_time_sec(tail->header.stamp) - get_time_sec(head->header.stamp);
    }
    
    in.acc = acc_avr;
    in.gyro = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    kf_state.predict(dt, Q, in);

    /* save the poses at each IMU measurements */
    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
    for(int i=0; i<3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];
    }
    double &&offs_t = get_time_sec(tail->header.stamp) - pcl_beg_time;
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);
  
  imu_state = kf_state.get_x();
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  /*** undistort each lidar point (backward propagation) ***/
  if (pcl_out.points.begin() == pcl_out.points.end()) return;
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

bool ImuProcess::UndistortOdinNative(PointCloudXYZI &cloud, const MeasureGroup &meas,
                                     const V3D &bg, const V3D &ba)
{
  // Odin points stay in Odin lidar frame. Time = header + offset_time (curvature ms).
  // Integrate /odin1/imu on that clock; compensate every point to this scan's end.
  if (cloud.points.empty() || meas.imu2.size() < 2)
    return false;

  constexpr double kMaxDt = 0.20;
  const double beg = meas.lidar_beg_time2;
  double span = 0.0;
  for (const auto &pt : cloud.points)
    span = std::max(span, static_cast<double>(pt.curvature) / 1000.0);
  if (span < 1e-4)
    return false;
  const double end = beg + span;

  auto v_imu = meas.imu2;
  // Ensure coverage: drop samples far before beg; keep at least one prior sample.
  while (v_imu.size() > 2 &&
         get_time_sec(v_imu[1]->header.stamp) < beg - 1e-4)
    v_imu.pop_front();

  const double acc_scale = G_m_s2 / std::max(1e-3, mean_acc.norm());
  const M3D R_LI = Odin_Lidar_R_wrt_IMU_;
  const V3D T_LI = Odin_Lidar_T_wrt_IMU_;
  const M3D R_IL = R_LI.transpose();

  // Relative IMU pose chain in a local frame that starts at Identity @ beg.
  std::vector<Pose6D> poses;
  poses.reserve(v_imu.size() + 2);

  M3D R = Eye3d;
  V3D pos = Zero3d;
  V3D vel = Zero3d;
  V3D angvel = Zero3d;
  V3D acc_s = Zero3d;
  double last_t = beg;
  poses.push_back(set_pose6d(0.0, acc_s, angvel, vel, pos, R));

  input_ikfom in;
  for (size_t i = 0; i + 1 < v_imu.size(); i++)
  {
    const auto &head = v_imu[i];
    const auto &tail = v_imu[i + 1];
    const double t_head = get_time_sec(head->header.stamp);
    const double t_tail = get_time_sec(tail->header.stamp);
    if (t_tail < beg)
      continue;

    V3D angvel_avr, acc_avr;
    angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
        0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
        0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
        0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
        0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);
    angvel_avr -= bg;
    acc_avr = acc_avr * acc_scale - ba;

    double t0 = std::max(t_head, last_t);
    double t1 = t_tail;
    bool hit_end = false;
    if (t1 > end + 1e-9)
    {
      t1 = end;
      hit_end = true;
    }
    double dt = t1 - t0;
    if (dt <= 1e-9)
    {
      if (hit_end)
        break;
      continue;
    }

    // Constant-ω / constant-a integration in the local frame (grav≈0 relative).
    R = R * Exp(angvel_avr, dt);
    const V3D acc_world = R * acc_avr;
    pos += vel * dt + 0.5 * acc_world * dt * dt;
    vel += acc_world * dt;
    angvel = angvel_avr;
    acc_s = acc_world;
    last_t = t1;
    poses.push_back(set_pose6d(last_t - beg, acc_s, angvel, vel, pos, R));
    if (hit_end || t_tail >= end - 1e-9)
      break;
  }

  // Stub to exact scan end if IMU stopped early.
  if (end - last_t > 1e-6 && !poses.empty())
  {
    const double dt = std::min(kMaxDt, end - last_t);
    R = R * Exp(angvel, dt);
    pos += vel * dt + 0.5 * acc_s * dt * dt;
    vel += acc_s * dt;
    last_t = end;
    poses.push_back(set_pose6d(span, acc_s, angvel, vel, pos, R));
  }

  if (poses.size() <= 1)
    return false;

  const M3D R_end = R;
  const V3D pos_end = pos;
  const M3D R_end_T = R_end.transpose();

  std::vector<int> order(cloud.points.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    return cloud.points[a].curvature < cloud.points[b].curvature;
  });

  auto apply_pose = [&](PointType &pt, const double t_off,
                        const M3D &R0, const V3D &pos0, const V3D &vel0,
                        const V3D &acc0, const V3D &gyr0, const double t0) {
    const double dt = t_off - t0;
    if (std::fabs(dt) > kMaxDt)
      return;
    const M3D R_i = R0 * Exp(gyr0, dt);
    const V3D P_i(pt.x, pt.y, pt.z);
    const V3D T_ei = pos0 + vel0 * dt + 0.5 * acc0 * dt * dt - pos_end;
    const V3D P_compensate =
        R_IL * (R_end_T * (R_i * (R_LI * P_i + T_LI) + T_ei) - T_LI);
    pt.x = P_compensate(0);
    pt.y = P_compensate(1);
    pt.z = P_compensate(2);
  };

  size_t idx = order.size();
  M3D R_imu;
  V3D vel_imu, pos_imu, acc_imu, angvel_avr;
  for (auto it_kp = poses.end() - 1; it_kp != poses.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu << MAT_FROM_ARRAY(head->rot);
    vel_imu << VEC_FROM_ARRAY(head->vel);
    pos_imu << VEC_FROM_ARRAY(head->pos);
    acc_imu << VEC_FROM_ARRAY(tail->acc);
    angvel_avr << VEC_FROM_ARRAY(tail->gyr);
    while (idx > 0)
    {
      PointType &pt = cloud.points[order[idx - 1]];
      const double t = static_cast<double>(pt.curvature) / 1000.0;
      if (t <= head->offset_time)
        break;
      --idx;
      apply_pose(pt, t, R_imu, pos_imu, vel_imu, acc_imu, angvel_avr,
                 head->offset_time);
    }
  }
  if (idx > 0)
  {
    auto head = poses.begin();
    R_imu << MAT_FROM_ARRAY(head->rot);
    vel_imu << VEC_FROM_ARRAY(head->vel);
    pos_imu << VEC_FROM_ARRAY(head->pos);
    acc_imu << VEC_FROM_ARRAY(head->acc);
    angvel_avr << VEC_FROM_ARRAY(head->gyr);
    for (size_t i = 0; i < idx; i++)
    {
      PointType &pt = cloud.points[order[i]];
      const double t = static_cast<double>(pt.curvature) / 1000.0;
      apply_pose(pt, t, R_imu, pos_imu, vel_imu, acc_imu, angvel_avr,
                 head->offset_time);
    }
  }

  static int s_odin_native_log = 0;
  if ((++s_odin_native_log % 40) == 0)
  {
    std::cout << "[odin-native deskew] imu2=" << meas.imu2.size()
              << " poses=" << poses.size() << " span=" << span * 1000.0
              << "ms pts=" << cloud.points.size() << "\n";
  }
  return true;
}

void ImuProcess::UndistortPclMultiLiDAR(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  const double imu_end_time = get_time_sec(v_imu.back()->header.stamp);
  // ALWAYS anchor IMU propagation + Mid deskew on Mid scan time.
  // Using min(Mid,Odin) made Mid curvature relative to an earlier Odin stamp and
  // twisted /cloud_registered whenever the TCP/restamped L2 led Mid.
  constexpr double kMaxBundleSpan = 0.20; // s
  // Odin whole-frame mid time can sit ~bias behind Mid beg → |dt| to Mid end
  // ~0.15s; 0.12 was clipping (mixed deskewed/raw → turn snap).
  constexpr double kMaxUndistortDt = 0.20; // s

  // Official KF integrates once to Mid end. Deskew may need a longer horizon
  // (signed_dt + odin_span); that path uses a shadow copy so we never
  // predict(+dt) then predict(-dt) on the real filter (non-inverse + Q inject).
  // With Odin-native deskew, Mid IMU horizon stays at Mid end (no cross-sensor).
  const double pcl_beg_time = meas.lidar_beg_time;
  const double mid_end_time = meas.lidar_end_time;
  const bool native_odin =
      odin_native_deskew_ && meas.lidar2 && !meas.lidar2->empty() &&
      meas.imu2.size() >= 2;
  // When native mode is on, lidar_end_time2 is on the Odin/USB clock — never
  // use it as a Mid-IMU propagation horizon (even on imu2-thin fallback).
  const double deskew_end_time =
      odin_native_deskew_ ? mid_end_time
                         : std::max(mid_end_time, meas.lidar_end_time2);

  PointCloudXYZI pcl_1_out = *(meas.lidar);
  PointCloudXYZI pcl_2_out = *(meas.lidar2);
  sort(pcl_1_out.points.begin(), pcl_1_out.points.end(), time_list);

  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;
  double dt = 0;
  input_ikfom in;
  V3D angvel_prop = angvel_last;
  V3D acc_s_prop = acc_s_last;

  // Track last integrated IMU time so stub never rewinds from v_imu.back()
  // when the buffer extends past end_time (Odin horizon).
  auto propagate_imu =
      [&](esekfom::esekf<state_ikfom, 12, input_ikfom> &kf, const double end_time,
          const bool record_pose, std::vector<Pose6D> *pose_out,
          V3D &angvel_io, V3D &acc_s_io) {
        state_ikfom st = kf.get_x();
        double last_prop_t = last_lidar_end_time_;
        if (record_pose && pose_out)
        {
          pose_out->clear();
          double offs0 = 0.0;
          if (last_lidar_end_time_ > 1.0)
            offs0 = last_lidar_end_time_ - pcl_beg_time;
          if (offs0 < -kMaxBundleSpan)
            offs0 = -kMaxBundleSpan;
          if (offs0 > kMaxBundleSpan)
            offs0 = kMaxBundleSpan;
          pose_out->push_back(set_pose6d(offs0, acc_s_io, angvel_io, st.vel, st.pos,
                                         st.rot.toRotationMatrix()));
        }

        for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
        {
          auto &&head = *(it_imu);
          auto &&tail = *(it_imu + 1);
          const double t_tail = get_time_sec(tail->header.stamp);
          if (t_tail < last_lidar_end_time_)
            continue;

          angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
              0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
              0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
          acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
              0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
              0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);
          acc_avr = acc_avr * G_m_s2 / mean_acc.norm();

          if (get_time_sec(head->header.stamp) < last_lidar_end_time_)
            dt = t_tail - last_lidar_end_time_;
          else
            dt = t_tail - get_time_sec(head->header.stamp);

          // Clip interval that crosses end_time so KF stops exactly on anchor.
          bool hit_end = false;
          if (t_tail > end_time + 1e-9)
          {
            const double t0 = (get_time_sec(head->header.stamp) < last_lidar_end_time_)
                                  ? last_lidar_end_time_
                                  : get_time_sec(head->header.stamp);
            dt = end_time - t0;
            hit_end = true;
            if (dt <= 1e-9)
              break;
          }

          in.acc = acc_avr;
          in.gyro = angvel_avr;
          Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
          Q.block<3, 3>(3, 3).diagonal() = cov_acc;
          Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
          Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
          kf.predict(dt, Q, in);

          st = kf.get_x();
          angvel_io = angvel_avr - st.bg;
          acc_s_io = st.rot * (acc_avr - st.ba);
          for (int i = 0; i < 3; i++)
            acc_s_io[i] += st.grav[i];
          last_prop_t = hit_end ? end_time : t_tail;
          if (record_pose && pose_out)
          {
            const double offs_t = last_prop_t - pcl_beg_time;
            pose_out->push_back(set_pose6d(offs_t, acc_s_io, angvel_io, st.vel,
                                           st.pos, st.rot.toRotationMatrix()));
          }
          if (hit_end || t_tail >= end_time - 1e-9)
            break;
        }

        // Forward stub only — never rewind from a later buffer tail.
        dt = end_time - last_prop_t;
        if (dt > kMaxBundleSpan)
          dt = kMaxBundleSpan;
        if (dt > 1e-6)
        {
          kf.predict(dt, Q, in);
          st = kf.get_x();
          last_prop_t = end_time;
          if (record_pose && pose_out)
          {
            const double offs_t = end_time - pcl_beg_time;
            pose_out->push_back(set_pose6d(offs_t, acc_s_io, angvel_io, st.vel,
                                           st.pos, st.rot.toRotationMatrix()));
          }
        }
      };

  // Shadow: full deskew horizon (may pass Mid end). Does not touch official P.
  esekfom::esekf<state_ikfom, 12, input_ikfom> kf_shadow = kf_state;
  V3D angvel_shadow = angvel_last;
  V3D acc_s_shadow = acc_s_last;
  propagate_imu(kf_shadow, deskew_end_time, true, &IMUpose, angvel_shadow,
                acc_s_shadow);

  // Official KF: integrate once to Mid end only (IEKF / publish anchor).
  propagate_imu(kf_state, mid_end_time, false, nullptr, angvel_prop, acc_s_prop);
  angvel_last = angvel_prop;
  acc_s_last = acc_s_prop;

  // Deskew compensates into this Mid-end state; IMUpose may extend past it.
  state_ikfom imu_state = kf_state.get_x();
  // Official bookmark must not jump past Mid end. meas.imu may include the
  // Odin extend window; advancing last_imu_ there leaves a hole next frame
  // (head already > last_lidar_end → dt=tail-head skips Mid-end→head).
  last_imu_ = meas.imu.front();
  for (auto it = meas.imu.rbegin(); it != meas.imu.rend(); ++it)
  {
    if (get_time_sec((*it)->header.stamp) <= mid_end_time + 1e-6)
    {
      last_imu_ = *it;
      break;
    }
  }
  last_lidar_end_time_ = mid_end_time;
  if (deskew_end_time > mid_end_time + 1e-3)
  {
    static int s_imu_bookmark_log = 0;
    if ((++s_imu_bookmark_log % 40) == 0)
    {
      const double limu = get_time_sec(last_imu_->header.stamp);
      std::cout << "[IMU bookmark] last_imu-mid_end="
                << (limu - mid_end_time) * 1000.0
                << "ms extend=" << (deskew_end_time - mid_end_time) * 1000.0
                << "ms (official bookmark must stay <=0)\n";
    }
  }

  // Deskew one cloud: point time = lidar_beg_offset + curvature_ms/1000, relative to pcl_beg.
  // Supports t < 0 (Odin USB header earlier than Mid beg after header remap).
  auto deskew_cloud = [&](PointCloudXYZI &cloud, const double lidar_beg_time)
  {
    if (cloud.points.empty() || IMUpose.size() <= 1)
      return;
    const double beg_off = lidar_beg_time - pcl_beg_time;
    std::vector<int> order(cloud.points.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
      return cloud.points[a].curvature < cloud.points[b].curvature;
    });

    auto apply_pose = [&](PointType &pt, const double t,
                          const M3D &R0, const V3D &pos0, const V3D &vel0,
                          const V3D &acc0, const V3D &gyr0, const double t0) {
      dt = t - t0;
      if (std::fabs(dt) > kMaxUndistortDt)
        return;
      M3D R_i(R0 * Exp(gyr0, dt));
      V3D P_i(pt.x, pt.y, pt.z);
      V3D T_ei(pos0 + vel0 * dt + 0.5 * acc0 * dt * dt - imu_state.pos);
      V3D P_compensate =
          imu_state.offset_R_L_I.conjugate() *
          (imu_state.rot.conjugate() *
               (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) -
           imu_state.offset_T_L_I);
      pt.x = P_compensate(0);
      pt.y = P_compensate(1);
      pt.z = P_compensate(2);
    };

    size_t idx = order.size();
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
    {
      auto head = it_kp - 1;
      auto tail = it_kp;
      R_imu << MAT_FROM_ARRAY(head->rot);
      vel_imu << VEC_FROM_ARRAY(head->vel);
      pos_imu << VEC_FROM_ARRAY(head->pos);
      acc_imu << VEC_FROM_ARRAY(tail->acc);
      angvel_avr << VEC_FROM_ARRAY(tail->gyr);

      while (idx > 0)
      {
        PointType &pt = cloud.points[order[idx - 1]];
        const double t = beg_off + static_cast<double>(pt.curvature) / 1000.0;
        if (t <= head->offset_time)
          break;
        --idx;
        apply_pose(pt, t, R_imu, pos_imu, vel_imu, acc_imu, angvel_avr, head->offset_time);
      }
    }

    // Remaining points have t <= IMUpose.front() (typically Mid beg = 0).
    // Back-propagate with front pose + its gyro (negative dt).
    if (idx > 0)
    {
      auto head = IMUpose.begin();
      R_imu << MAT_FROM_ARRAY(head->rot);
      vel_imu << VEC_FROM_ARRAY(head->vel);
      pos_imu << VEC_FROM_ARRAY(head->pos);
      acc_imu << VEC_FROM_ARRAY(head->acc);
      angvel_avr << VEC_FROM_ARRAY(head->gyr);
      for (size_t i = 0; i < idx; i++)
      {
        PointType &pt = cloud.points[order[i]];
        const double t = beg_off + static_cast<double>(pt.curvature) / 1000.0;
        apply_pose(pt, t, R_imu, pos_imu, vel_imu, acc_imu, angvel_avr, head->offset_time);
      }
    }
  };

  deskew_cloud(pcl_1_out, meas.lidar_beg_time);

  // Mid-only ticks leave pcl_2 empty (viz holds last body cloud).
  if (!pcl_2_out.points.empty())
  {
    bool native_ok = false;
    if (native_odin)
    {
      // Knife: Odin intra-scan on /odin1/imu (same device clock), then one rigid
      // T_L2_wrt_L1. Do NOT map each Odin point onto Mid IMU time.
      const state_ikfom st = kf_state.get_x();
      native_ok = UndistortOdinNative(pcl_2_out, meas, st.bg, st.ba);
      if (native_ok)
      {
        for (auto &pt : pcl_2_out.points)
        {
          const V3D p(pt.x, pt.y, pt.z);
          const V3D q = Lidar2_R_wrt_Lidar1_ * p + Lidar2_T_wrt_Lidar1_;
          pt.x = q(0);
          pt.y = q(1);
          pt.z = q(2);
        }
      }
      else
      {
        static int s_odin_native_fb = 0;
        if ((++s_odin_native_fb % 20) == 0)
          std::cout << "[odin-native deskew] fallback→Mid-IMU path (imu2 thin)\n";
      }
    }

    if (!native_ok)
    {
      // Legacy: center-lock Odin offsets onto Mid IMU timeline + SE3 deskew.
      // Non-native fill already mapped L2→L1; native-enabled path keeps Odin
      // lidar frame until here — apply T_L2 once before Mid-IMU deskew.
      if (odin_native_deskew_)
      {
        for (auto &pt : pcl_2_out.points)
        {
          const V3D p(pt.x, pt.y, pt.z);
          const V3D q = Lidar2_R_wrt_Lidar1_ * p + Lidar2_T_wrt_Lidar1_;
          pt.x = q(0);
          pt.y = q(1);
          pt.z = q(2);
        }
      }
      std::vector<double> offs;
      offs.reserve(pcl_2_out.points.size());
      for (const auto &pt : pcl_2_out.points)
        offs.push_back(static_cast<double>(pt.curvature) / 1000.0);
      double odin_med = 0.0;
      if (!offs.empty())
      {
        const size_t mid = offs.size() / 2;
        std::nth_element(offs.begin(), offs.begin() + static_cast<long>(mid),
                         offs.end());
        odin_med = offs[mid];
      }
      const double mid_span = std::max(0.0, mid_end_time - pcl_beg_time);
      const double header_off =
          (meas.lidar_beg_time2 + lidar2_stamp_bias_) - meas.lidar_beg_time;
      static int s_center_lock_log = 0;
      const double center_residual = header_off + odin_med - 0.5 * mid_span;
      if ((++s_center_lock_log % 40) == 0)
      {
        std::cout << "[deskew center-lock] cancel_header_center="
                  << center_residual * 1000.0
                  << "ms mid_span=" << mid_span * 1000.0
                  << "ms odin_med=" << odin_med * 1000.0 << "ms\n";
      }
      for (auto &pt : pcl_2_out.points)
      {
        double t_rel =
            static_cast<double>(pt.curvature) / 1000.0 - odin_med + 0.5 * mid_span;
        if (t_rel < -kMaxBundleSpan)
          t_rel = -kMaxBundleSpan;
        if (t_rel > kMaxBundleSpan + 0.12)
          t_rel = kMaxBundleSpan + 0.12;
        pt.curvature = static_cast<float>(t_rel * 1000.0);
      }
      deskew_cloud(pcl_2_out, meas.lidar_beg_time);
    }
  }

  for (auto &pt : pcl_1_out.points) pt.intensity = 10.0f;   // Mid360
  for (auto &pt : pcl_2_out.points) pt.intensity = 200.0f;  // Odin1
  pcl_out = pcl_1_out + pcl_2_out;
}

void ImuProcess::Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_, const bool &multi_lidar, int lidar_num)
{
  if(meas.imu.empty()) {return;};
  assert(meas.lidar != nullptr);

  if (imu_need_init_)
  {
    /// The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;
    
    last_imu_   = meas.imu.back();

    if (init_iter_num > MAX_INI_COUNT)
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      std::cout << "IMU Initial Done" << std::endl;
    }
    return;
  }

  if (multi_lidar) UndistortPclMultiLiDAR(meas, kf_state, *cur_pcl_un_);
  else
  {
    UndistortPcl(meas, kf_state, *cur_pcl_un_, lidar_num);
    // Async single-lidar path: still tag source so publish/viz/filters can split.
    const float tag = (lidar_num == 2) ? 200.0f : 10.0f;
    for (auto &pt : cur_pcl_un_->points) pt.intensity = tag;
  }
  
  // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}
