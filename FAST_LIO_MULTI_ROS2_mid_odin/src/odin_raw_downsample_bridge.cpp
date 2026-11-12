// C++ Odin raw → L2 downsample bridge (replaces Python hot path).
// Latest-only, pre-decimate, causal Mid pose for /odin1/l2_mid_pose.
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

namespace {

struct MidStamp {
  double t = 0.0;
  builtin_interfaces::msg::Time stamp;
  geometry_msgs::msg::Pose pose;
};

struct FieldOff {
  int offset = -1;
  uint8_t datatype = 0;
};

inline double stamp_sec(const builtin_interfaces::msg::Time &s)
{
  return static_cast<double>(s.sec) + 1e-9 * static_cast<double>(s.nanosec);
}

inline builtin_interfaces::msg::Time sec_to_stamp(double t)
{
  builtin_interfaces::msg::Time s;
  if (t < 0.0) t = 0.0;
  s.sec = static_cast<int32_t>(t);
  s.nanosec = static_cast<uint32_t>((t - static_cast<double>(s.sec)) * 1e9);
  return s;
}

inline float read_f32(const uint8_t *p, uint8_t dt)
{
  if (dt == sensor_msgs::msg::PointField::FLOAT32) {
    float v;
    std::memcpy(&v, p, 4);
    return v;
  }
  if (dt == sensor_msgs::msg::PointField::UINT16) {
    uint16_t v;
    std::memcpy(&v, p, 2);
    return static_cast<float>(v);
  }
  if (dt == sensor_msgs::msg::PointField::UINT8) {
    return static_cast<float>(*p);
  }
  if (dt == sensor_msgs::msg::PointField::UINT32) {
    uint32_t v;
    std::memcpy(&v, p, 4);
    return static_cast<float>(v) * 1e-9f;  // offset_time ns → s if uint32
  }
  return 0.f;
}

sensor_msgs::msg::PointCloud2 make_xyzit(
  const std_msgs::msg::Header &header,
  const std::vector<float> &packed)  // N*5: x y z i t
{
  sensor_msgs::msg::PointCloud2 out;
  out.header = header;
  out.height = 1;
  const size_t n = packed.size() / 5;
  out.width = static_cast<uint32_t>(n);
  out.is_bigendian = false;
  out.is_dense = true;
  out.point_step = 20;
  out.row_step = out.point_step * out.width;
  out.fields.resize(5);
  auto setf = [](sensor_msgs::msg::PointField &f, const char *name, uint32_t off) {
    f.name = name;
    f.offset = off;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
  };
  setf(out.fields[0], "x", 0);
  setf(out.fields[1], "y", 4);
  setf(out.fields[2], "z", 8);
  setf(out.fields[3], "intensity", 12);
  setf(out.fields[4], "offset_time", 16);
  out.data.resize(packed.size() * sizeof(float));
  if (!packed.empty()) {
    std::memcpy(out.data.data(), packed.data(), out.data.size());
  }
  return out;
}

}  // namespace

class OdinRawDownsampleBridge : public rclcpp::Node
{
public:
  OdinRawDownsampleBridge()
  : Node("odin_raw_downsample_bridge")
  {
    declare_parameter<std::string>("input_topic", "/odin1/cloud_raw");
    declare_parameter<std::string>("output_topic", "/odin1/cloud_raw_ds");
    declare_parameter<std::string>("l2_body_topic", "/odin1/cloud_l2_body");
    declare_parameter<std::string>("l2_mid_pose_topic", "/odin1/l2_mid_pose");
    declare_parameter<std::string>("odom_topic", "/Odometry");
    declare_parameter<double>("voxel_size", 0.12);
    declare_parameter<int>("max_points", 2000);
    declare_parameter<int>("pre_decimate_max", 2000);
    declare_parameter<double>("min_confidence", 30.0);
    declare_parameter<double>("mid_restamp_max_dt", 0.12);
    declare_parameter<double>("max_cloud_age_sec", 0.25);
    // Legacy: subtract from L2 stamp (breaks Bundle — leaves L2 between Mid
    // frames → sync dt walks to max_sync_dt). Prefer align_l2_stamp_to_mid.
    declare_parameter<double>("stamp_latency_comp_sec", 0.0);
    // Match Mid at (L2_recv - lag) then optionally copy that Mid stamp onto L2
    // so Bundle dt≈0. Live relative L2−Mid lag ≈60–80ms (not full USB 150ms).
    declare_parameter<double>("densify_mid_lag_comp_sec", 0.065);
    // Snap L2 header stamp to matched Mid stamp (kills 0→100ms Bundle sawtooth).
    declare_parameter<bool>("align_l2_stamp_to_mid", true);
    declare_parameter<bool>("publish_ds_topic", true);

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    l2_body_topic_ = get_parameter("l2_body_topic").as_string();
    l2_mid_pose_topic_ = get_parameter("l2_mid_pose_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    voxel_ = std::max(1e-3, get_parameter("voxel_size").as_double());
    max_points_ = std::max(100, static_cast<int>(get_parameter("max_points").as_int()));
    pre_decimate_max_ = std::max(
      100, static_cast<int>(get_parameter("pre_decimate_max").as_int()));
    min_confidence_ = get_parameter("min_confidence").as_double();
    mid_restamp_max_dt_ = get_parameter("mid_restamp_max_dt").as_double();
    max_cloud_age_sec_ = get_parameter("max_cloud_age_sec").as_double();
    stamp_latency_comp_sec_ = std::max(0.0, get_parameter("stamp_latency_comp_sec").as_double());
    densify_mid_lag_comp_sec_ =
      std::max(0.0, get_parameter("densify_mid_lag_comp_sec").as_double());
    align_l2_stamp_to_mid_ = get_parameter("align_l2_stamp_to_mid").as_bool();
    publish_ds_ = get_parameter("publish_ds_topic").as_bool();

    auto reliable = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    if (publish_ds_) {
      pub_ds_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, reliable);
    }
    if (!l2_body_topic_.empty()) {
      pub_l2_ = create_publisher<sensor_msgs::msg::PointCloud2>(l2_body_topic_, reliable);
    }
    if (!l2_mid_pose_topic_.empty() && pub_l2_) {
      pub_l2_mid_ = create_publisher<geometry_msgs::msg::PoseStamped>(l2_mid_pose_topic_, reliable);
    }

    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, cloud_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { on_cloud(std::move(msg)); });
    sub_mid_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, reliable,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_mid(std::move(msg)); });

    status_timer_ = create_wall_timer(
      std::chrono::seconds(2), [this]() { status(); });

    RCLCPP_INFO(
      get_logger(),
      "C++ odin bridge %s -> l2=%s ds=%s max=%d pre=%d voxel=%.3f mid_dt<=%.3f "
      "age<=%.2f stamp_comp=%.0fms mid_lag=%.0fms align_l2_to_mid=%s",
      input_topic_.c_str(), l2_body_topic_.c_str(),
      publish_ds_ ? output_topic_.c_str() : "off",
      max_points_, pre_decimate_max_, voxel_, mid_restamp_max_dt_, max_cloud_age_sec_,
      stamp_latency_comp_sec_ * 1000.0, densify_mid_lag_comp_sec_ * 1000.0,
      align_l2_stamp_to_mid_ ? "true" : "false");
  }

private:
  void on_mid(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    MidStamp m;
    m.t = stamp_sec(msg->header.stamp);
    m.stamp = msg->header.stamp;
    m.pose = msg->pose.pose;
    mid_stamps_.push_back(m);
    while (mid_stamps_.size() > 500) mid_stamps_.pop_front();
    ++n_mid_;
  }

  // Nearest Mid within mid_restamp_max_dt (prefer <= t_cloud, else closest).
  bool match_mid(
    double t_cloud, geometry_msgs::msg::Pose &pose_out,
    builtin_interfaces::msg::Time &stamp_out, std::string &tag)
  {
    const MidStamp *best = nullptr;
    double best_abs = 1e9;
    const MidStamp *before = nullptr;
    for (const auto &m : mid_stamps_) {
      const double adt = std::fabs(m.t - t_cloud);
      if (adt < best_abs) {
        best_abs = adt;
        best = &m;
      }
      if (m.t <= t_cloud) {
        if (!before || m.t > before->t) before = &m;
      }
    }
    // Prefer causal (before) if still inside window — avoids using future Mid
    // pose while Odin USB stamp is late.
    const MidStamp *pick = nullptr;
    double dt_signed = 0.0;
    if (before && (t_cloud - before->t) <= mid_restamp_max_dt_) {
      pick = before;
      dt_signed = t_cloud - before->t;
      char buf[64];
      std::snprintf(buf, sizeof(buf), "causal:%.0fms", dt_signed * 1000.0);
      tag = buf;
    } else if (best && best_abs <= mid_restamp_max_dt_) {
      pick = best;
      dt_signed = t_cloud - best->t;
      char buf[64];
      std::snprintf(buf, sizeof(buf), "nearest:%+.0fms", dt_signed * 1000.0);
      tag = buf;
    } else if (before) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "causal_old:%.0fms", (t_cloud - before->t) * 1000.0);
      tag = buf;
      return false;
    } else if (best) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "far:%.0fms", best_abs * 1000.0);
      tag = buf;
      return false;
    } else {
      tag = "none";
      return false;
    }
    pose_out = pick->pose;
    stamp_out = pick->stamp;
    return true;
  }

  void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    ++n_cloud_;
    const double wall = now().seconds();
    // Latest-only: if previous frame still processing, drop this by overwriting
    // is implicit via depth=1; also skip if busy.
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      ++n_drop_busy_;
      return;
    }
    const auto t0 = std::chrono::steady_clock::now();
    try {
      process(msg, wall);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "bridge exception: %s", e.what());
    }
    const auto t1 = std::chrono::steady_clock::now();
    last_proc_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
    busy_.store(false);
  }

  void process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, double wall)
  {
    if (max_cloud_age_sec_ > 0.0) {
      // Prefer header age if clock-aligned; also gate by callback wall delay via
      // storing nothing — depth=1 already latest. Soft age on stamp lag:
      const double age = wall - stamp_sec(msg->header.stamp);
      if (age > max_cloud_age_sec_ + 0.5) {  // allow USB stamp lag ~0.15 + margin
        // Still process: stamp lag ≠ queue lag. Only drop if absurdly old.
        if (age > 2.0) {
          ++n_skip_age_;
          return;
        }
      }
    }

    FieldOff fx, fy, fz, fi, fc, ft;
    for (const auto &f : msg->fields) {
      if (f.name == "x") fx = {static_cast<int>(f.offset), f.datatype};
      else if (f.name == "y") fy = {static_cast<int>(f.offset), f.datatype};
      else if (f.name == "z") fz = {static_cast<int>(f.offset), f.datatype};
      else if (f.name == "intensity") fi = {static_cast<int>(f.offset), f.datatype};
      else if (f.name == "confidence") fc = {static_cast<int>(f.offset), f.datatype};
      else if (f.name == "offset_time" || f.name == "time" || f.name == "t")
        ft = {static_cast<int>(f.offset), f.datatype};
    }
    if (fx.offset < 0 || fy.offset < 0 || fz.offset < 0) {
      ++n_skip_empty_;
      return;
    }

    const int n = static_cast<int>(msg->width) * static_cast<int>(msg->height);
    const int step = static_cast<int>(msg->point_step);
    if (n <= 0 || step <= 0 || static_cast<int>(msg->data.size()) < n * step) {
      ++n_skip_empty_;
      return;
    }

    int stride = 1;
    if (pre_decimate_max_ > 0 && n > pre_decimate_max_) {
      stride = (n + pre_decimate_max_ - 1) / pre_decimate_max_;
    }

    std::vector<float> pts;
    pts.reserve(static_cast<size_t>(pre_decimate_max_) * 5);

    for (int i = 0; i < n; i += stride) {
      const uint8_t *p = msg->data.data() + static_cast<size_t>(i) * step;
      const float x = read_f32(p + fx.offset, fx.datatype);
      const float y = read_f32(p + fy.offset, fy.datatype);
      const float z = read_f32(p + fz.offset, fz.datatype);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      if (fc.offset >= 0 && min_confidence_ > 0.0) {
        const float conf = read_f32(p + fc.offset, fc.datatype);
        if (conf < static_cast<float>(min_confidence_)) continue;
      }
      float inten = 0.f;
      if (fi.offset >= 0) inten = read_f32(p + fi.offset, fi.datatype);
      float ot = 0.f;
      if (ft.offset >= 0) ot = read_f32(p + ft.offset, ft.datatype);
      pts.push_back(x);
      pts.push_back(y);
      pts.push_back(z);
      pts.push_back(inten);
      pts.push_back(ot);
    }

    if (pts.empty()) {
      ++n_skip_empty_;
      return;
    }

    // Stride to max_points (skip expensive voxel unique — matches Python path).
    size_t n_pts = pts.size() / 5;
    if (static_cast<int>(n_pts) > max_points_) {
      const int step0 = static_cast<int>(
        std::ceil(static_cast<double>(n_pts) / static_cast<double>(max_points_)));
      std::vector<float> slim;
      slim.reserve(static_cast<size_t>(max_points_) * 5);
      for (size_t i = 0; i < n_pts; i += static_cast<size_t>(step0)) {
        const size_t b = i * 5;
        slim.insert(slim.end(), pts.begin() + b, pts.begin() + b + 5);
      }
      pts.swap(slim);
      n_pts = pts.size() / 5;
    }
    last_n_l2_ = static_cast<int>(n_pts);

    std_msgs::msg::Header header = msg->header;
    header.frame_id = "odin_lidar";
    const double t_recv = stamp_sec(msg->header.stamp);
    // Optional legacy subtract (usually 0). Prefer Mid-stamp align below.
    const double t_l2 =
      (stamp_latency_comp_sec_ > 0.0) ? (t_recv - stamp_latency_comp_sec_) : t_recv;
    // Match Mid near capture-approx: USB receive leads capture by ~relative lag.
    const double t_pose = t_l2 - densify_mid_lag_comp_sec_;
    geometry_msgs::msg::Pose mid_pose;
    builtin_interfaces::msg::Time mid_stamp;
    std::string tag;
    bool have_mid = false;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      have_mid = match_mid(t_pose, mid_pose, mid_stamp, tag);
      last_l2_match_ = tag;
    }
    if (align_l2_stamp_to_mid_ && have_mid) {
      // Bundle pairs by stamp — copying Mid stamp collapses 0→100ms sawtooth.
      header.stamp = mid_stamp;
    } else {
      header.stamp = sec_to_stamp(t_l2);
    }
    auto cloud = make_xyzit(header, pts);

    if (pub_ds_) {
      pub_ds_->publish(cloud);
      ++n_pub_;
    }

    if (pub_l2_) {
      pub_l2_->publish(cloud);
      ++n_l2_pub_;
      if (have_mid && pub_l2_mid_) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = header.stamp;  // == L2 stamp for densify lookup
        ps.header.frame_id = "map";
        ps.pose = mid_pose;
        pub_l2_mid_->publish(ps);
      } else {
        ++n_skip_l2_sync_;
      }
    }
  }

  void status()
  {
    RCLCPP_INFO(
      get_logger(),
      "cpp_bridge cloud=%lld mid=%lld pub=%lld l2=%lld skip_sync=%lld drop_busy=%lld "
      "skip_empty=%lld skip_age=%lld last_n=%d proc_ms=%.1f l2_match=%s",
      static_cast<long long>(n_cloud_), static_cast<long long>(n_mid_),
      static_cast<long long>(n_pub_), static_cast<long long>(n_l2_pub_),
      static_cast<long long>(n_skip_l2_sync_), static_cast<long long>(n_drop_busy_),
      static_cast<long long>(n_skip_empty_), static_cast<long long>(n_skip_age_),
      last_n_l2_, last_proc_ms_, last_l2_match_.c_str());
  }

  std::string input_topic_, output_topic_, l2_body_topic_, l2_mid_pose_topic_, odom_topic_;
  double voxel_ = 0.12;
  int max_points_ = 2000;
  int pre_decimate_max_ = 2000;
  double min_confidence_ = 30.0;
  double mid_restamp_max_dt_ = 0.12;
  double stamp_latency_comp_sec_ = 0.0;
  double densify_mid_lag_comp_sec_ = 0.065;
  double max_cloud_age_sec_ = 0.25;
  bool align_l2_stamp_to_mid_ = true;
  bool publish_ds_ = true;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ds_, pub_l2_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_l2_mid_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_mid_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::mutex mtx_;
  std::deque<MidStamp> mid_stamps_;
  std::atomic<bool> busy_{false};

  long long n_cloud_ = 0, n_mid_ = 0, n_pub_ = 0, n_l2_pub_ = 0;
  long long n_skip_l2_sync_ = 0, n_drop_busy_ = 0, n_skip_empty_ = 0, n_skip_age_ = 0;
  int last_n_l2_ = 0;
  double last_proc_ms_ = -1.0;
  std::string last_l2_match_ = "-";
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdinRawDownsampleBridge>());
  rclcpp::shutdown();
  return 0;
}
