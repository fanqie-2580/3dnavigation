#include <mpc_critics/pure_pursuit_model.h>

#include <algorithm>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(mpc_critics::PurePursuitModel, mpc_critics::ScoringModel)

namespace mpc_critics
{

PurePursuitModel::PurePursuitModel()
{
  return;
}

void PurePursuitModel::onInitialize()
{
  node_->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".weight", weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "weight: %.2f", weight_);

  node_->declare_parameter(name_ + ".translation_weight", rclcpp::ParameterValue(0.5));
  node_->get_parameter(name_ + ".translation_weight", translation_weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "translation_weight: %.2f", translation_weight_);

  node_->declare_parameter(name_ + ".orientation_weight", rclcpp::ParameterValue(0.5));
  node_->get_parameter(name_ + ".orientation_weight", orientation_weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "orientation_weight: %.2f", orientation_weight_);

  // [GO2ReactiveBypassV5]
  node_->declare_parameter(name_ + ".allow_invalid_prune_plan", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".allow_invalid_prune_plan", allow_invalid_prune_plan_);

  node_->declare_parameter(name_ + ".invalid_prune_plan_cost", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".invalid_prune_plan_cost", invalid_prune_plan_cost_);

  node_->declare_parameter(name_ + ".min_required_prune_points", rclcpp::ParameterValue(1));
  node_->get_parameter(name_ + ".min_required_prune_points", min_required_prune_points_);

  node_->declare_parameter(name_ + ".min_required_traj_points", rclcpp::ParameterValue(2));
  node_->get_parameter(name_ + ".min_required_traj_points", min_required_traj_points_);

  RCLCPP_INFO(
    node_->get_logger().get_child(name_),
    "allow_invalid_prune_plan: %d, invalid_prune_plan_cost: %.2f, min_required_prune_points: %d, min_required_traj_points: %d",
    allow_invalid_prune_plan_, invalid_prune_plan_cost_, min_required_prune_points_, min_required_traj_points_);
}

double PurePursuitModel::scoreTrajectory(base_trajectory::Trajectory &traj)
{
  const int min_prune_points = std::max(1, min_required_prune_points_);
  const int min_traj_points = std::max(1, min_required_traj_points_);

  if (static_cast<int>(shared_data_->prune_plan_.poses.size()) < min_prune_points ||
      static_cast<int>(traj.getPointsSize()) < min_traj_points) {
    if (allow_invalid_prune_plan_) {
      return invalid_prune_plan_cost_;
    }
    return -4.0;
  }

  geometry_msgs::msg::PoseStamped last_traj_pose = traj.getPoint(traj.getPointsSize() - 1);
  geometry_msgs::msg::PoseStamped last_prune_plan_pose = shared_data_->prune_plan_.poses.back();

  geometry_msgs::msg::TransformStamped tf_last_traj_pose, tf_last_prune_plan_pose;
  tf_last_traj_pose.header.frame_id = shared_data_->global_frame_;
  tf_last_traj_pose.child_frame_id = shared_data_->base_frame_;
  tf_last_traj_pose.transform.translation.x = last_traj_pose.pose.position.x;
  tf_last_traj_pose.transform.translation.y = last_traj_pose.pose.position.y;
  tf_last_traj_pose.transform.translation.z = last_traj_pose.pose.position.z;
  tf_last_traj_pose.transform.rotation = last_traj_pose.pose.orientation;

  tf_last_prune_plan_pose.header.frame_id = shared_data_->global_frame_;
  tf_last_prune_plan_pose.child_frame_id = shared_data_->base_frame_;
  tf_last_prune_plan_pose.transform.translation.x = last_prune_plan_pose.pose.position.x;
  tf_last_prune_plan_pose.transform.translation.y = last_prune_plan_pose.pose.position.y;
  tf_last_prune_plan_pose.transform.translation.z = last_prune_plan_pose.pose.position.z;
  tf_last_prune_plan_pose.transform.rotation = last_prune_plan_pose.pose.orientation;

  Eigen::Affine3d last_traj_pose_af3 = tf2::transformToEigen(tf_last_traj_pose);
  last_traj_pose_af3 = last_traj_pose_af3.inverse();
  Eigen::Affine3d last_prune_plan_pose_af3 = tf2::transformToEigen(tf_last_prune_plan_pose);

  Eigen::Affine3d pose_difference_af3 = last_traj_pose_af3 * last_prune_plan_pose_af3;
  geometry_msgs::msg::TransformStamped tf_pose_difference = tf2::eigenToTransform(pose_difference_af3);

  double roll, yaw, pitch;
  tf2::Quaternion q;
  tf2::convert(tf_pose_difference.transform.rotation, q);
  tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

  yaw = std::fmod((yaw + 3.1416), 3.1416);
  double distance = std::sqrt(
    tf_pose_difference.transform.translation.x * tf_pose_difference.transform.translation.x +
    tf_pose_difference.transform.translation.y * tf_pose_difference.transform.translation.y +
    tf_pose_difference.transform.translation.z * tf_pose_difference.transform.translation.z);

  return (translation_weight_ * distance + orientation_weight_ * yaw);
}

}  // namespace mpc_critics
