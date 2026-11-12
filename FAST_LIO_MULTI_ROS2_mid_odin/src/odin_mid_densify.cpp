// C++ Mid-pose densify for Odin L2 (live + still-only accum).
// Replaces Python odin_independent_accum hot path for project_with_mid.
#include <atomic>
#include <cmath>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace {

inline double stamp_sec(const builtin_interfaces::msg::Time &s)
{
  return static_cast<double>(s.sec) + 1e-9 * static_cast<double>(s.nanosec);
}

Eigen::Matrix4d pose_to_T(const geometry_msgs::msg::Pose &p)
{
  const double x = p.orientation.x, y = p.orientation.y, z = p.orientation.z, w = p.orientation.w;
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 0) = 1 - 2 * (y * y + z * z);
  T(0, 1) = 2 * (x * y - z * w);
  T(0, 2) = 2 * (x * z + y * w);
  T(1, 0) = 2 * (x * y + z * w);
  T(1, 1) = 1 - 2 * (x * x + z * z);
  T(1, 2) = 2 * (y * z - x * w);
  T(2, 0) = 2 * (x * z - y * w);
  T(2, 1) = 2 * (y * z + x * w);
  T(2, 2) = 1 - 2 * (x * x + y * y);
  T(0, 3) = p.position.x;
  T(1, 3) = p.position.y;
  T(2, 3) = p.position.z;
  return T;
}

Eigen::Matrix4d se3_from_rt(const std::vector<double> &R9, const std::vector<double> &t3)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  if (R9.size() >= 9) {
    T.block<3, 3>(0, 0) << R9[0], R9[1], R9[2], R9[3], R9[4], R9[5], R9[6], R9[7], R9[8];
  }
  if (t3.size() >= 3) {
    T(0, 3) = t3[0];
    T(1, 3) = t3[1];
    T(2, 3) = t3[2];
  }
  return T;
}

sensor_msgs::msg::PointCloud2 make_xyz(
  const std_msgs::msg::Header &header, const std::vector<float> &xyz)
{
  sensor_msgs::msg::PointCloud2 out;
  out.header = header;
  out.height = 1;
  const size_t n = xyz.size() / 3;
  out.width = static_cast<uint32_t>(n);
  out.is_bigendian = false;
  out.is_dense = true;
  out.point_step = 12;
  out.row_step = 12 * out.width;
  out.fields.resize(3);
  for (int i = 0; i < 3; ++i) {
    out.fields[i].name = (i == 0 ? "x" : (i == 1 ? "y" : "z"));
    out.fields[i].offset = static_cast<uint32_t>(i * 4);
    out.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[i].count = 1;
  }
  out.data.resize(xyz.size() * sizeof(float));
  if (!xyz.empty()) std::memcpy(out.data.data(), xyz.data(), out.data.size());
  return out;
}

struct KeyHash {
  size_t operator()(const std::tuple<int, int, int> &k) const
  {
    const auto [a, b, c] = k;
    return (static_cast<size_t>(a) * 73856093u) ^
           (static_cast<size_t>(b) * 19349663u) ^
           (static_cast<size_t>(c) * 83492791u);
  }
};

}  // namespace

class OdinMidDensify : public rclcpp::Node
{
public:
  OdinMidDensify()
  : Node("odin_independent_accum")  // keep name so clear_accum / launch match
  {
    declare_parameter<std::string>("cloud_topic", "/odin1/cloud_l2_body");
    declare_parameter<std::string>("l2_mid_pose_topic", "/odin1/l2_mid_pose");
    declare_parameter<std::string>("mid_odom_topic", "/Odometry");
    declare_parameter<std::string>("out_scan_topic", "/odin1/cloud_in_map");
    declare_parameter<std::string>("out_accum_topic", "/viz/accum_odin");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<double>("voxel_size", 0.12);
    declare_parameter<int>("max_accum_points", 400000);
    declare_parameter<double>("accum_skip_sec", 2.0);
    declare_parameter<double>("accum_max_yaw_rate_deg", 3.0);
    declare_parameter<double>("still_trans_vel", 0.05);
    declare_parameter<double>("still_yaw_rate_deg", 3.0);
    declare_parameter<double>("accum_publish_period", 0.5);
    declare_parameter<bool>("restamp_scan_to_now", true);
    declare_parameter<std::vector<double>>("extrinsic_T", {-0.011, -0.02329, 0.04412});
    declare_parameter<std::vector<double>>(
      "extrinsic_R", {1, 0, 0, 0, 1, 0, 0, 0, 1});
    declare_parameter<std::vector<double>>(
      "extrinsic_T_L2_wrt_L1", {0.12157888, 0.00549610, 0.03169932});
    declare_parameter<std::vector<double>>(
      "extrinsic_R_L2_wrt_L1",
      {0.97437006, 0.0, -0.22495105, 0.0, 1.0, 0.0, 0.22495105, 0.0, 0.97437006});

    cloud_topic_ = get_parameter("cloud_topic").as_string();
    l2_mid_pose_topic_ = get_parameter("l2_mid_pose_topic").as_string();
    mid_odom_topic_ = get_parameter("mid_odom_topic").as_string();
    out_scan_ = get_parameter("out_scan_topic").as_string();
    out_accum_ = get_parameter("out_accum_topic").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    voxel_ = std::max(1e-3, get_parameter("voxel_size").as_double());
    max_accum_ = std::max(
      1000, static_cast<int>(get_parameter("max_accum_points").as_int()));
    accum_skip_sec_ = get_parameter("accum_skip_sec").as_double();
    accum_max_yaw_ = get_parameter("accum_max_yaw_rate_deg").as_double();
    still_trans_vel_ = get_parameter("still_trans_vel").as_double();
    still_yaw_rate_ = get_parameter("still_yaw_rate_deg").as_double();
    restamp_now_ = get_parameter("restamp_scan_to_now").as_bool();
    const double period = get_parameter("accum_publish_period").as_double();

    T_body_odin_ = se3_from_rt(
                     get_parameter("extrinsic_R").as_double_array(),
                     get_parameter("extrinsic_T").as_double_array()) *
                   se3_from_rt(
                     get_parameter("extrinsic_R_L2_wrt_L1").as_double_array(),
                     get_parameter("extrinsic_T_L2_wrt_L1").as_double_array());

    auto reliable = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    pub_scan_ = create_publisher<sensor_msgs::msg::PointCloud2>(out_scan_, reliable);
    pub_accum_ = create_publisher<sensor_msgs::msg::PointCloud2>(out_accum_, reliable);

    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, cloud_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { on_cloud(std::move(msg)); });
    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      l2_mid_pose_topic_, reliable,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { on_l2_pose(std::move(msg)); });
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      mid_odom_topic_, reliable,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_mid_odom(std::move(msg)); });

    clear_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/clear_accum",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        std::lock_guard<std::mutex> lk(mtx_accum_);
        accum_.clear();
        dirty_ = true;
        t0_ = now().seconds();
        resp->success = true;
        resp->message = "cleared C++ Odin densify accum";
      });

    accum_timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(0.2, period)),
      [this]() { publish_accum(); });
    status_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() { status(); });
    t0_ = now().seconds();

    RCLCPP_INFO(
      get_logger(),
      "C++ densify cloud=%s pose=%s -> %s accum=%s voxel=%.3f still_yaw<=%.1fdeg",
      cloud_topic_.c_str(), l2_mid_pose_topic_.c_str(), out_scan_.c_str(), out_accum_.c_str(),
      voxel_, accum_max_yaw_);
  }

private:
  void on_l2_pose(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_pose_);
    pose_buf_.emplace_back(stamp_sec(msg->header.stamp), pose_to_T(msg->pose));
    while (pose_buf_.size() > 200) pose_buf_.pop_front();
  }

  void on_mid_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const double wall = now().seconds();
    const auto T = pose_to_T(msg->pose.pose);
    const double yaw = std::atan2(T(1, 0), T(0, 0));
    const Eigen::Vector3d pos = T.block<3, 1>(0, 3);
    if (have_yaw_prev_) {
      const double dw = wall - yaw_prev_wall_;
      if (dw > 1e-3) {
        double dy = yaw - yaw_prev_;
        while (dy > M_PI) dy -= 2 * M_PI;
        while (dy < -M_PI) dy += 2 * M_PI;
        yaw_rate_deg_ = std::abs(dy / dw) * (180.0 / M_PI);
      }
    }
    if (have_pos_prev_) {
      const double dw = wall - pos_prev_wall_;
      if (dw > 1e-3) {
        trans_vel_ = (pos - pos_prev_).norm() / dw;
      }
    }
    yaw_prev_wall_ = wall;
    yaw_prev_ = yaw;
    have_yaw_prev_ = true;
    pos_prev_wall_ = wall;
    pos_prev_ = pos;
    have_pos_prev_ = true;
  }

  bool lookup_pose(double t_cloud, Eigen::Matrix4d &T_out)
  {
    std::lock_guard<std::mutex> lk(mtx_pose_);
    const Eigen::Matrix4d *best = nullptr;
    double best_dt = 1e9;
    for (const auto &e : pose_buf_) {
      const double dt = std::abs(e.first - t_cloud);
      if (dt < best_dt) {
        best_dt = dt;
        best = &e.second;
      }
    }
    if (best && best_dt <= 0.05) {
      T_out = *best;
      return true;
    }
    return false;
  }

  void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      ++n_drop_;
      return;
    }
    const auto t0 = std::chrono::steady_clock::now();
    try {
      process(msg);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "densify exception: %s", e.what());
    }
    last_proc_ms_ = std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - t0)
                      .count();
    busy_.store(false);
  }

  void process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
  {
    ++n_cloud_;
    Eigen::Matrix4d T_mid;
    if (!lookup_pose(stamp_sec(msg->header.stamp), T_mid)) {
      ++n_skip_pose_;
      return;
    }
    const Eigen::Matrix4d T_map = T_mid * T_body_odin_;
    const Eigen::Matrix3d R = T_map.block<3, 3>(0, 0);
    const Eigen::Vector3d t = T_map.block<3, 1>(0, 3);

    int ox = -1, oy = -1, oz = -1;
    for (const auto &f : msg->fields) {
      if (f.name == "x") ox = static_cast<int>(f.offset);
      else if (f.name == "y") oy = static_cast<int>(f.offset);
      else if (f.name == "z") oz = static_cast<int>(f.offset);
    }
    if (ox < 0 || oy < 0 || oz < 0) return;
    const int n = static_cast<int>(msg->width) * static_cast<int>(msg->height);
    const int step = static_cast<int>(msg->point_step);
    if (n <= 0 || step <= 0) return;

    std::vector<float> out;
    out.reserve(static_cast<size_t>(n) * 3);
    for (int i = 0; i < n; ++i) {
      const uint8_t *p = msg->data.data() + static_cast<size_t>(i) * step;
      float x, y, z;
      std::memcpy(&x, p + ox, 4);
      std::memcpy(&y, p + oy, 4);
      std::memcpy(&z, p + oz, 4);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      Eigen::Vector3d pw = R * Eigen::Vector3d(x, y, z) + t;
      out.push_back(static_cast<float>(pw.x()));
      out.push_back(static_cast<float>(pw.y()));
      out.push_back(static_cast<float>(pw.z()));
    }
    if (out.empty()) return;

    std_msgs::msg::Header h;
    h.frame_id = map_frame_;
    if (restamp_now_) {
      const int64_t ns = now().nanoseconds();
      h.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
      h.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    } else {
      h.stamp = msg->header.stamp;
    }
    pub_scan_->publish(make_xyz(h, out));
    ++n_pub_;

    // Still-only keep-first accum (soft viz; official map = C++ ikdtree).
    const double yaw_lim = (accum_max_yaw_ > 0.0) ? accum_max_yaw_ : still_yaw_rate_;
    if ((now().seconds() - t0_) < accum_skip_sec_) return;
    if (yaw_lim > 0.0 && yaw_rate_deg_ > yaw_lim) {
      ++n_skip_accum_;
      return;
    }
    if (still_trans_vel_ > 0.0 && trans_vel_ > still_trans_vel_) {
      ++n_skip_accum_;
      return;
    }

    const double inv = 1.0 / voxel_;
    std::lock_guard<std::mutex> lk(mtx_accum_);
    const size_t npts = out.size() / 3;
    for (size_t i = 0; i < npts; ++i) {
      if (static_cast<int>(accum_.size()) >= max_accum_) break;
      const float x = out[i * 3], y = out[i * 3 + 1], z = out[i * 3 + 2];
      const auto key = std::make_tuple(
        static_cast<int>(std::floor(x * inv)),
        static_cast<int>(std::floor(y * inv)),
        static_cast<int>(std::floor(z * inv)));
      if (accum_.find(key) != accum_.end()) continue;
      accum_.emplace(key, Eigen::Vector3f(x, y, z));
      dirty_ = true;
    }
  }

  void publish_accum()
  {
    if (!dirty_ && accum_.empty()) return;
    std::vector<float> xyz;
    {
      std::lock_guard<std::mutex> lk(mtx_accum_);
      xyz.reserve(accum_.size() * 3);
      for (const auto &kv : accum_) {
        xyz.push_back(kv.second.x());
        xyz.push_back(kv.second.y());
        xyz.push_back(kv.second.z());
      }
      dirty_ = false;
    }
    std_msgs::msg::Header h;
    {
      const int64_t ns = now().nanoseconds();
      h.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
      h.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    }
    h.frame_id = map_frame_;
    pub_accum_->publish(make_xyz(h, xyz));
  }

  void status()
  {
    RCLCPP_INFO(
      get_logger(),
      "cpp_densify cloud=%lld pub=%lld skip_pose=%lld skip_accum=%lld drop=%lld "
      "accum_n=%zu yaw=%.1fdeg/s v=%.2f proc_ms=%.1f",
      static_cast<long long>(n_cloud_), static_cast<long long>(n_pub_),
      static_cast<long long>(n_skip_pose_), static_cast<long long>(n_skip_accum_),
      static_cast<long long>(n_drop_), accum_.size(), yaw_rate_deg_, trans_vel_,
      last_proc_ms_);
  }

  std::string cloud_topic_, l2_mid_pose_topic_, mid_odom_topic_, out_scan_, out_accum_, map_frame_;
  double voxel_ = 0.12;
  int max_accum_ = 400000;
  double accum_skip_sec_ = 2.0, accum_max_yaw_ = 3.0, still_trans_vel_ = 0.05, still_yaw_rate_ = 3.0;
  bool restamp_now_ = true;
  Eigen::Matrix4d T_body_odin_ = Eigen::Matrix4d::Identity();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_, pub_accum_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;
  rclcpp::TimerBase::SharedPtr accum_timer_, status_timer_;

  std::mutex mtx_pose_, mtx_accum_;
  std::deque<std::pair<double, Eigen::Matrix4d>> pose_buf_;
  std::unordered_map<std::tuple<int, int, int>, Eigen::Vector3f, KeyHash> accum_;
  bool dirty_ = false;
  double t0_ = 0.0;

  double yaw_rate_deg_ = 0.0, trans_vel_ = 0.0;
  double yaw_prev_ = 0.0, yaw_prev_wall_ = 0.0;
  Eigen::Vector3d pos_prev_ = Eigen::Vector3d::Zero();
  double pos_prev_wall_ = 0.0;
  bool have_yaw_prev_ = false, have_pos_prev_ = false;

  std::atomic<bool> busy_{false};
  long long n_cloud_ = 0, n_pub_ = 0, n_skip_pose_ = 0, n_skip_accum_ = 0, n_drop_ = 0;
  double last_proc_ms_ = -1.0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdinMidDensify>());
  rclcpp::shutdown();
  return 0;
}
