#include "bspline_opt/uniform_bspline.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "traj_utils/msg/bspline.hpp"
#include "traj_utils/msg/trajectory_execution_state.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
using ego_planner::UniformBspline;
using ExecutionState = traj_utils::msg::TrajectoryExecutionState;
using SteadyClock = std::chrono::steady_clock;

rclcpp::Node::SharedPtr node;
std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr control_point_state_pub;
rclcpp::Publisher<ExecutionState>::SharedPtr execution_state_pub;

std::string world_frame = "map";
std::string base_frame = "base_link";
std::string odom_topic = "odom_world";
std::string cmd_topic = "/cmd_vel";
std::string control_point_topic = "/control_point_state_";
std::string execution_state_topic = "planning/trajectory_execution_state";

double kp_pos_x = 0.8;
double kp_pos_y = 1.0;
double kp_yaw = 1.5;
double finish_distance = 0.12;
double yaw_lookahead_time = 0.5;
double heading_error_threshold = 0.8;
double align_stuck_timeout = 3.0;
double align_progress_min = 0.05;
double max_vx = 0.8;
double max_vy = 0.35;
double max_wz = 0.6;
int cmd_interval_ms = 20;
double align_watch_abs_error = -1.0;
SteadyClock::time_point align_watch_steady;
bool force_track_after_align_stuck = false;

std::vector<UniformBspline> trajectory;
double trajectory_duration = 0.0;
int64_t trajectory_id = -1;
double execution_time = 0.0;
uint8_t controller_state = ExecutionState::WAIT_TRAJECTORY;
std::string fault_reason;
bool have_trajectory = false;
bool trajectory_is_stationary = false;
double last_desired_yaw = 0.0;

Eigen::Vector3d odom_position = Eigen::Vector3d::Zero();
double odom_yaw = 0.0;
bool have_odom = false;
SteadyClock::time_point last_control_steady;
bool have_last_control_time = false;

geometry_msgs::msg::Twist last_cmd_vel;

double clamp(double value, double lower, double upper)
{
  return std::min(std::max(value, lower), upper);
}

double normalizeAngle(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi)
    angle -= 2.0 * kPi;
  while (angle < -kPi)
    angle += 2.0 * kPi;
  return angle;
}

Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double limit)
{
  const double bounded_limit = std::max(0.0, limit);
  const double norm = value.norm();
  if (norm <= bounded_limit || norm < 1e-9)
    return value;
  return value * (bounded_limit / norm);
}

void setControllerState(uint8_t new_state, const std::string &reason = std::string())
{
  if (controller_state != new_state || fault_reason != reason)
  {
    RCLCPP_INFO(node->get_logger(), "Trajectory controller state %u -> %u%s%s",
                static_cast<unsigned>(controller_state), static_cast<unsigned>(new_state),
                reason.empty() ? "" : ": ", reason.c_str());
  }
  controller_state = new_state;
  fault_reason = reason;
}

void enterFault(const std::string &reason)
{
  setControllerState(ExecutionState::FAULT_STOP, reason);
}

void publishExecutionState(const rclcpp::Time &stamp)
{
  ExecutionState msg;
  msg.traj_id = trajectory_id;
  msg.state = controller_state;
  msg.exec_time = execution_time;
  msg.frozen = controller_state == ExecutionState::ALIGN_HEADING;
  msg.stamp = stamp;
  msg.fault_reason = fault_reason;
  execution_state_pub->publish(msg);
}

void publishZeroCommand()
{
  geometry_msgs::msg::Twist cmd;
  cmd_vel_pub->publish(cmd);
  last_cmd_vel = cmd;
}

bool finitePoint(const geometry_msgs::msg::Point &point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool validateTrajectoryMessage(const traj_utils::msg::Bspline &msg, std::string &reason)
{
  if (msg.order < 1)
  {
    reason = "trajectory order must be positive";
    return false;
  }
  if (msg.pos_pts.size() < static_cast<size_t>(msg.order + 1))
  {
    reason = "trajectory has too few control points";
    return false;
  }
  if (msg.knots.size() != msg.pos_pts.size() + static_cast<size_t>(msg.order) + 1)
  {
    reason = "knot count does not match control point count and spline order";
    return false;
  }
  for (const auto &point : msg.pos_pts)
  {
    if (!finitePoint(point))
    {
      reason = "trajectory contains a non-finite control point";
      return false;
    }
  }
  for (size_t i = 0; i < msg.knots.size(); ++i)
  {
    if (!std::isfinite(msg.knots[i]))
    {
      reason = "trajectory contains a non-finite knot";
      return false;
    }
    if (i > 0 && msg.knots[i] + 1e-9 < msg.knots[i - 1])
    {
      reason = "trajectory knots are not monotonic";
      return false;
    }
  }
  return true;
}

bool desiredYawAt(double sample_time, double &yaw)
{
  if (!have_trajectory)
    return false;
  if (trajectory_is_stationary)
  {
    yaw = odom_yaw;
    return true;
  }

  const double t = clamp(sample_time, 0.0, trajectory_duration);
  const double forward_t = std::min(trajectory_duration, t + yaw_lookahead_time);
  const Eigen::Vector3d current = trajectory[0].evaluateDeBoorT(t);

  Eigen::Vector2d direction =
      (trajectory[0].evaluateDeBoorT(forward_t) - current).head<2>();
  if (direction.squaredNorm() < 1e-4)
    direction = trajectory[1].evaluateDeBoorT(t).head<2>();
  if (direction.squaredNorm() < 1e-4)
  {
    yaw = odom_yaw;
    return true;
  }

  yaw = std::atan2(direction.y(), direction.x());
  return true;
}

bool trajectoryHasPlanarMotion(double &initial_yaw)
{
  const Eigen::Vector2d start = trajectory[0].evaluateDeBoorT(0.0).head<2>();
  const int samples = std::max(10, static_cast<int>(std::ceil(trajectory_duration / 0.05)));
  for (int i = 1; i <= samples; ++i)
  {
    const double t = trajectory_duration * static_cast<double>(i) / static_cast<double>(samples);
    const Eigen::Vector2d delta = trajectory[0].evaluateDeBoorT(t).head<2>() - start;
    if (delta.squaredNorm() >= 1e-4)
    {
      initial_yaw = std::atan2(delta.y(), delta.x());
      return true;
    }
  }
  return false;
}

void publishDesiredState(const rclcpp::Time &stamp, const Eigen::Vector3d &position,
                         const Eigen::Vector3d &velocity, const Eigen::Vector3d &acceleration,
                         double desired_yaw, double desired_yaw_rate)
{
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, desired_yaw);

  nav_msgs::msg::Odometry control_point;
  control_point.header.stamp = stamp;
  control_point.header.frame_id = world_frame;
  control_point.child_frame_id = base_frame;
  control_point.pose.pose.position.x = position.x();
  control_point.pose.pose.position.y = position.y();
  control_point.pose.pose.position.z = position.z();
  control_point.pose.pose.orientation = tf2::toMsg(orientation);
  control_point_state_pub->publish(control_point);

  quadrotor_msgs::msg::PositionCommand position_command;
  position_command.header.stamp = stamp;
  position_command.header.frame_id = world_frame;
  position_command.trajectory_flag =
      quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  position_command.trajectory_id = trajectory_id;
  position_command.position.x = position.x();
  position_command.position.y = position.y();
  position_command.position.z = position.z();
  position_command.velocity.x = velocity.x();
  position_command.velocity.y = velocity.y();
  position_command.velocity.z = velocity.z();
  position_command.acceleration.x = acceleration.x();
  position_command.acceleration.y = acceleration.y();
  position_command.acceleration.z = acceleration.z();
  position_command.yaw = desired_yaw;
  position_command.yaw_dot = desired_yaw_rate;
  pos_cmd_pub->publish(position_command);
}

void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  geometry_msgs::msg::PoseStamped pose_in;
  geometry_msgs::msg::PoseStamped pose_world;
  pose_in.header = msg->header;
  pose_in.pose = msg->pose.pose;

  try
  {
    if (msg->header.frame_id.empty() || msg->header.frame_id == world_frame)
    {
      pose_world = pose_in;
      pose_world.header.frame_id = world_frame;
    }
    else
    {
      const auto transform = tf_buffer->lookupTransform(
          world_frame, msg->header.frame_id, rclcpp::Time(msg->header.stamp),
          rclcpp::Duration::from_seconds(0.05));
      tf2::doTransform(pose_in, pose_world, transform);
    }
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                          "Odometry transform %s -> %s failed: %s",
                          msg->header.frame_id.c_str(), world_frame.c_str(), ex.what());
    return;
  }

  const auto &q = pose_world.pose.orientation;
  const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(q_norm) || q_norm < 1e-6)
  {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                          "Odometry contains an invalid orientation quaternion");
    return;
  }

  odom_position << pose_world.pose.position.x, pose_world.pose.position.y,
      pose_world.pose.position.z;
  odom_yaw = tf2::getYaw(pose_world.pose.orientation);
  have_odom = true;
}

void bsplineCallback(const traj_utils::msg::Bspline::ConstSharedPtr msg)
{
  if (have_trajectory && msg->traj_id <= trajectory_id)
  {
    RCLCPP_WARN(node->get_logger(), "Discard stale trajectory id=%ld, active id=%ld",
                static_cast<long>(msg->traj_id), static_cast<long>(trajectory_id));
    return;
  }

  std::string validation_error;
  if (!validateTrajectoryMessage(*msg, validation_error))
  {
    trajectory_id = msg->traj_id;
    have_trajectory = false;
    enterFault(validation_error);
    return;
  }
  if (!have_odom)
  {
    trajectory_id = msg->traj_id;
    have_trajectory = false;
    enterFault("trajectory received before valid odometry");
    return;
  }

  Eigen::MatrixXd control_points(3, msg->pos_pts.size());
  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    control_points(0, i) = msg->pos_pts[i].x;
    control_points(1, i) = msg->pos_pts[i].y;
    control_points(2, i) = msg->pos_pts[i].z;
  }
  for (size_t i = 0; i < msg->knots.size(); ++i)
    knots(i) = msg->knots[i];

  UniformBspline position_trajectory(control_points, msg->order, 0.1);
  position_trajectory.setKnot(knots);
  const double duration = position_trajectory.getTimeSum();
  if (!std::isfinite(duration) || duration <= 1e-4)
  {
    trajectory_id = msg->traj_id;
    have_trajectory = false;
    enterFault("trajectory duration is invalid");
    return;
  }

  trajectory.clear();
  trajectory.push_back(position_trajectory);
  trajectory.push_back(trajectory[0].getDerivative());
  trajectory.push_back(trajectory[1].getDerivative());
  trajectory_duration = duration;
  trajectory_id = msg->traj_id;
  execution_time = 0.0;
  have_trajectory = true;

  trajectory_is_stationary = !trajectoryHasPlanarMotion(last_desired_yaw);
  if (trajectory_is_stationary)
  {
    setControllerState(ExecutionState::HOLD_FINAL);
  }
  else
  {
    const double yaw_error = normalizeAngle(last_desired_yaw - odom_yaw);
    setControllerState(std::abs(yaw_error) > heading_error_threshold
                           ? ExecutionState::ALIGN_HEADING
                           : ExecutionState::TRACK_TRAJECTORY);
  }

  have_last_control_time = false;
  align_watch_abs_error = -1.0;
  force_track_after_align_stuck = false;
  RCLCPP_INFO(node->get_logger(), "Accepted trajectory id=%ld duration=%.3f",
              static_cast<long>(trajectory_id), trajectory_duration);
}

void controlCallback()
{
  const auto steady_now = SteadyClock::now();
  const rclcpp::Time ros_now = node->now();
  double dt = static_cast<double>(cmd_interval_ms) / 1000.0;
  if (have_last_control_time)
    dt = std::chrono::duration<double>(steady_now - last_control_steady).count();
  last_control_steady = steady_now;
  have_last_control_time = true;

  if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.2)
    dt = 0.0;
  if (controller_state == ExecutionState::WAIT_TRAJECTORY && !have_trajectory)
  {
    publishZeroCommand();
    publishExecutionState(ros_now);
    return;
  }
  if (!have_odom)
  {
    enterFault("odometry timeout");
  }

  if (controller_state == ExecutionState::WAIT_TRAJECTORY ||
      controller_state == ExecutionState::FAULT_STOP)
  {
    publishZeroCommand();
    publishExecutionState(ros_now);
    return;
  }

  if (!have_trajectory)
  {
    enterFault("controller state requires a trajectory but none is active");
    publishZeroCommand();
    publishExecutionState(ros_now);
    return;
  }

  if (controller_state == ExecutionState::HOLD_FINAL)
  {
    const Eigen::Vector3d final_position =
        trajectory[0].evaluateDeBoorT(trajectory_duration);
    const Eigen::Vector2d final_error =
        final_position.head<2>() - odom_position.head<2>();
    if (final_error.norm() <= finish_distance)
    {
      publishZeroCommand();
      publishDesiredState(ros_now, final_position, Eigen::Vector3d::Zero(),
                          Eigen::Vector3d::Zero(), odom_yaw, 0.0);
      publishExecutionState(ros_now);
      return;
    }
    setControllerState(ExecutionState::TRACK_TRAJECTORY);
  }

  double desired_yaw = last_desired_yaw;
  if (!desiredYawAt(execution_time, desired_yaw))
  {
    enterFault("moving trajectory has no valid planar heading");
    publishZeroCommand();
    publishExecutionState(ros_now);
    return;
  }
  last_desired_yaw = desired_yaw;
  const double yaw_error = normalizeAngle(desired_yaw - odom_yaw);

  const double abs_yaw_error = std::abs(yaw_error);
  // If odom yaw is frozen (common when LIO/sport yaw stall), ALIGN_HEADING would
  // otherwise loop forever: 3s spin-only -> brief track -> ALIGN again. Latch
  // TRACK for the rest of this trajectory once align is proven stuck.
  if (force_track_after_align_stuck && abs_yaw_error <= heading_error_threshold)
    force_track_after_align_stuck = false;

  const bool want_align =
      abs_yaw_error > heading_error_threshold && !force_track_after_align_stuck;

  if (want_align)
    setControllerState(ExecutionState::ALIGN_HEADING);
  else
  {
    align_watch_abs_error = -1.0;
    setControllerState(ExecutionState::TRACK_TRAJECTORY);
  }

  if (want_align)
  {
    if (align_stuck_timeout > 0.0)
    {
      if (align_watch_abs_error < 0.0)
      {
        align_watch_abs_error = abs_yaw_error;
        align_watch_steady = steady_now;
      }
      else
      {
        const double progressed = align_watch_abs_error - abs_yaw_error;
        const double waited =
            std::chrono::duration<double>(steady_now - align_watch_steady).count();
        if (progressed >= align_progress_min)
        {
          align_watch_abs_error = abs_yaw_error;
          align_watch_steady = steady_now;
        }
        else if (waited >= align_stuck_timeout)
        {
          RCLCPP_WARN(node->get_logger(),
                      "heading align stuck: odometry yaw not converging while rotating; "
                      "force TRACK for this trajectory (err=%.3f)",
                      abs_yaw_error);
          align_watch_abs_error = -1.0;
          force_track_after_align_stuck = true;
          setControllerState(ExecutionState::TRACK_TRAJECTORY);
        }
      }
    }

    if (!force_track_after_align_stuck)
    {
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = clamp(kp_yaw * yaw_error, -max_wz, max_wz);
      cmd_vel_pub->publish(cmd);
      last_cmd_vel = cmd;

      const Eigen::Vector3d position = trajectory[0].evaluateDeBoorT(execution_time);
      publishDesiredState(ros_now, position, Eigen::Vector3d::Zero(),
                          Eigen::Vector3d::Zero(), desired_yaw, cmd.angular.z);
      publishExecutionState(ros_now);
      return;
    }
  }

  execution_time = std::min(trajectory_duration, execution_time + dt);
  const Eigen::Vector3d desired_position = trajectory[0].evaluateDeBoorT(execution_time);
  Eigen::Vector3d desired_velocity = trajectory[1].evaluateDeBoorT(execution_time);
  Eigen::Vector3d desired_acceleration = trajectory[2].evaluateDeBoorT(execution_time);
  if (execution_time >= trajectory_duration - 1e-6)
  {
    desired_velocity.setZero();
    desired_acceleration.setZero();
  }
  const Eigen::Vector2d position_error =
      desired_position.head<2>() - odom_position.head<2>();

  const Eigen::Vector2d correction(kp_pos_x * position_error.x(),
                                   kp_pos_y * position_error.y());
  const Eigen::Vector2d velocity_world = clampNorm(
      desired_velocity.head<2>() + correction, std::max(max_vx, max_vy));

  const double c = std::cos(odom_yaw);
  const double s = std::sin(odom_yaw);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = clamp(c * velocity_world.x() + s * velocity_world.y(), -max_vx, max_vx);
  cmd.linear.y = clamp(-s * velocity_world.x() + c * velocity_world.y(), -max_vy, max_vy);
  cmd.angular.z = clamp(kp_yaw * yaw_error, -max_wz, max_wz);

  if (execution_time >= trajectory_duration - 1e-6 &&
      position_error.norm() <= finish_distance)
  {
    setControllerState(ExecutionState::HOLD_FINAL);
    publishZeroCommand();
    publishDesiredState(ros_now, desired_position, Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero(), desired_yaw, 0.0);
    publishExecutionState(ros_now);
    return;
  }

  cmd_vel_pub->publish(cmd);
  last_cmd_vel = cmd;

  publishDesiredState(ros_now, desired_position, desired_velocity, desired_acceleration,
                      desired_yaw, cmd.angular.z);

  publishExecutionState(ros_now);
}

bool loadAndValidateParameters()
{
  node->declare_parameter("traj_server/world_frame", world_frame);
  node->declare_parameter("traj_server/base_frame", base_frame);
  node->declare_parameter("traj_server/odometry_topic", odom_topic);
  node->declare_parameter("traj_server/cmd_topic", cmd_topic);
  node->declare_parameter("traj_server/control_point_topic", control_point_topic);
  node->declare_parameter("traj_server/execution_state_topic", execution_state_topic);
  node->declare_parameter("traj_server/kp_pos_x", kp_pos_x);
  node->declare_parameter("traj_server/kp_pos_y", kp_pos_y);
  node->declare_parameter("traj_server/kp_yaw", kp_yaw);
  node->declare_parameter("traj_server/finish_distance", finish_distance);
  node->declare_parameter("traj_server/yaw_lookahead_time", yaw_lookahead_time);
  node->declare_parameter("traj_server/heading_error_threshold", heading_error_threshold);
  node->declare_parameter("traj_server/align_stuck_timeout", align_stuck_timeout);
  node->declare_parameter("traj_server/align_progress_min", align_progress_min);
  node->declare_parameter("traj_server/max_vx", max_vx);
  node->declare_parameter("traj_server/max_vy", max_vy);
  node->declare_parameter("traj_server/max_wz", max_wz);
  node->declare_parameter("traj_server/cmd_interval_ms", cmd_interval_ms);

  node->get_parameter("traj_server/world_frame", world_frame);
  node->get_parameter("traj_server/base_frame", base_frame);
  node->get_parameter("traj_server/odometry_topic", odom_topic);
  node->get_parameter("traj_server/cmd_topic", cmd_topic);
  node->get_parameter("traj_server/control_point_topic", control_point_topic);
  node->get_parameter("traj_server/execution_state_topic", execution_state_topic);
  node->get_parameter("traj_server/kp_pos_x", kp_pos_x);
  node->get_parameter("traj_server/kp_pos_y", kp_pos_y);
  node->get_parameter("traj_server/kp_yaw", kp_yaw);
  node->get_parameter("traj_server/finish_distance", finish_distance);
  node->get_parameter("traj_server/yaw_lookahead_time", yaw_lookahead_time);
  node->get_parameter("traj_server/heading_error_threshold", heading_error_threshold);
  node->get_parameter("traj_server/align_stuck_timeout", align_stuck_timeout);
  node->get_parameter("traj_server/align_progress_min", align_progress_min);
  node->get_parameter("traj_server/max_vx", max_vx);
  node->get_parameter("traj_server/max_vy", max_vy);
  node->get_parameter("traj_server/max_wz", max_wz);
  node->get_parameter("traj_server/cmd_interval_ms", cmd_interval_ms);

  const bool valid =
      !world_frame.empty() && !base_frame.empty() && !odom_topic.empty() &&
      !cmd_topic.empty() && !execution_state_topic.empty() && kp_pos_x >= 0.0 &&
      kp_pos_y >= 0.0 && kp_yaw > 0.0 && finish_distance > 0.0 &&
      yaw_lookahead_time > 0.0 && heading_error_threshold > 0.0 &&
      align_stuck_timeout >= 0.0 && align_progress_min >= 0.0 &&
      max_vx > 0.0 && max_vy > 0.0 && max_wz > 0.0 && cmd_interval_ms >= 10;
  if (!valid)
    RCLCPP_FATAL(node->get_logger(), "Invalid trajectory controller parameters");
  return valid;
}
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("traj_server");

  if (!loadAndValidateParameters())
  {
    rclcpp::shutdown();
    return 1;
  }

  tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  auto bspline_sub = node->create_subscription<traj_utils::msg::Bspline>(
      "planning/bspline", 10, bsplineCallback);
  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 20, odometryCallback);

  pos_cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>(
      "position_cmd", 50);
  cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 50);
  control_point_state_pub = node->create_publisher<nav_msgs::msg::Odometry>(
      control_point_topic, 20);
  execution_state_pub = node->create_publisher<ExecutionState>(execution_state_topic, 20);

  auto control_timer = node->create_wall_timer(
      std::chrono::milliseconds(cmd_interval_ms), controlCallback);

  RCLCPP_WARN(node->get_logger(),
              "Trajectory controller ready: odom=%s cmd=%s state=%s rate=%.1f Hz world=%s base=%s",
              odom_topic.c_str(), cmd_topic.c_str(), execution_state_topic.c_str(),
              1000.0 / static_cast<double>(cmd_interval_ms), world_frame.c_str(),
              base_frame.c_str());

  rclcpp::spin(node);
  publishZeroCommand();
  rclcpp::shutdown();
  return 0;
}
