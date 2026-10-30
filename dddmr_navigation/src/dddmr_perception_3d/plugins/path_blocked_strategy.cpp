/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <perception_3d/path_blocked_strategy.h>
#include <algorithm>

PLUGINLIB_EXPORT_CLASS(perception_3d::PathBlockedStrategy, perception_3d::Sensor)

namespace perception_3d
{

PathBlockedStrategy::PathBlockedStrategy(){
  current_lethal_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  blocked_since_set_ = false;
  blocked_points_count_ = 0;
  forward_points_count_ = 0;
  return;
}

void PathBlockedStrategy::onInitialize()
{ 

  node_->declare_parameter(name_ + ".check_radius", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".check_radius", check_radius_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "check_radius: %.2f", check_radius_);//5.0
  
  // [SmoothLaneChange] Split blocked handling:
  // - Mild blockage => request replanning while still allowing local bypass.
  // - Severe sustained blockage => wait/stop.
  node_->declare_parameter(name_ + ".blocked_ratio_replanning_threshold", rclcpp::ParameterValue(3.0));
  node_->get_parameter(name_ + ".blocked_ratio_replanning_threshold", blocked_ratio_replanning_threshold_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "blocked_ratio_replanning_threshold: %.2f", blocked_ratio_replanning_threshold_);

  node_->declare_parameter(name_ + ".blocked_ratio_wait_threshold", rclcpp::ParameterValue(20.0));
  node_->get_parameter(name_ + ".blocked_ratio_wait_threshold", blocked_ratio_wait_threshold_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "blocked_ratio_wait_threshold: %.2f", blocked_ratio_wait_threshold_);

  node_->declare_parameter(name_ + ".blocked_hold_seconds", rclcpp::ParameterValue(0.8));
  node_->get_parameter(name_ + ".blocked_hold_seconds", blocked_hold_seconds_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "blocked_hold_seconds: %.2f", blocked_hold_seconds_);
  // [GO2SmoothBypassV2] Guard against ratio spikes caused by too few prune points.
  node_->declare_parameter(name_ + ".max_forward_points_to_check", rclcpp::ParameterValue(30));
  node_->get_parameter(name_ + ".max_forward_points_to_check", max_forward_points_to_check_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_forward_points_to_check: %d", max_forward_points_to_check_);

  node_->declare_parameter(name_ + ".min_forward_points_for_ratio", rclcpp::ParameterValue(8));
  node_->get_parameter(name_ + ".min_forward_points_for_ratio", min_forward_points_for_ratio_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_forward_points_for_ratio: %d", min_forward_points_for_ratio_);

  node_->declare_parameter(name_ + ".min_blocked_points_replanning", rclcpp::ParameterValue(2));
  node_->get_parameter(name_ + ".min_blocked_points_replanning", min_blocked_points_replanning_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_blocked_points_replanning: %d", min_blocked_points_replanning_);

  node_->declare_parameter(name_ + ".min_blocked_points_wait", rclcpp::ParameterValue(4));
  node_->get_parameter(name_ + ".min_blocked_points_wait", min_blocked_points_wait_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_blocked_points_wait: %d", min_blocked_points_wait_);


  prune_plan_blocked_ratio_ = 0.0;
  blocked_points_count_ = 0;
  forward_points_count_ = 0;

}

void PathBlockedStrategy::updateLethalPointCloud(){
}

void PathBlockedStrategy::selfMark(){

  //@initial opinion as pass
  opinion_ = perception_3d::PASS;

  //@ there is no obstacle, so return ratio 0
  if(shared_data_->aggregate_observation_->points.size()<= 5 || shared_data_->pcl_prune_plan_.points.size()<=0){
    prune_plan_blocked_ratio_ = 0.0;
  }
  //@this method will return percent of point of pruneplan which conflict with obstacle  
  else{
    
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr pcl_perception_kdtree;
    pcl_perception_kdtree.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    pcl_perception_kdtree->setInputCloud(shared_data_->aggregate_observation_);        
    
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    //float prune_plan_orignal_size;
    float prune_plan_orignal_size = 0.0f;
    float prune_plan_blocked_size;
    std::vector<geometry_msgs::msg::PoseStamped> prune_plan_point_blocked; //it could be deployed to a global variable
    int checked_forward_points = 0;
    int blocked_forward_points = 0;
    for(int i=0;i<shared_data_->pcl_prune_plan_.points.size();i++){

      //@ intensity is the index of prune plan, negative is backward of robot,position is forward of robot
      if(shared_data_->pcl_prune_plan_.points[i].intensity<0)
        continue;
      if(max_forward_points_to_check_ > 0 && checked_forward_points >= max_forward_points_to_check_){
        break;
      }
      checked_forward_points++;  
      prune_plan_orignal_size += 1.0f;

      if (pcl_perception_kdtree->radiusSearch (shared_data_->pcl_prune_plan_.points[i], check_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
        blocked_forward_points++;
        // --- 新加打印 ---
        RCLCPP_ERROR_THROTTLE(node_->get_logger().get_child(name_), *node_->get_clock(), 1000,
        "pathblock[%d](%.2f, %.2f) radius: %.2f", 
        i, shared_data_->pcl_prune_plan_.points[i].x, shared_data_->pcl_prune_plan_.points[i].y, check_radius_);
        // ---------------
        geometry_msgs::msg::PoseStamped temp_point;
        temp_point.pose.position.x = shared_data_->pcl_prune_plan_.points[i].x;
        temp_point.pose.position.y = shared_data_->pcl_prune_plan_.points[i].y;
        temp_point.pose.position.z = shared_data_->pcl_prune_plan_.points[i].z;
        prune_plan_point_blocked.push_back(temp_point);
      }
    }
    //prune_plan_orignal_size = shared_data_->pcl_prune_plan_.points.size();
    forward_points_count_ = checked_forward_points;
    blocked_points_count_ = blocked_forward_points;
    prune_plan_blocked_size = prune_plan_point_blocked.size();
    if(prune_plan_orignal_size <= 0.0f){
      prune_plan_blocked_ratio_ = 0.0;
    }
    else{
      prune_plan_blocked_ratio_ = (prune_plan_blocked_size) / (prune_plan_orignal_size)*100.0;
    }
    //prune_plan_blocked_ratio_ = (prune_plan_blocked_size) / (prune_plan_orignal_size)*100.0;
  }

  //if(prune_plan_blocked_ratio_>0.0) //原来为0.0
    //opinion_ = perception_3d::PATH_BLOCKED_WAIT;
    // [SmoothLaneChange] Decision ladder:
  // 1) Severe and persistent blockage => wait.
  // 2) Mild/temporary blockage => request replanning.
  // 3) Otherwise pass.
  //if(prune_plan_blocked_ratio_ >= blocked_ratio_wait_threshold_){
  const bool can_use_ratio = forward_points_count_ >= std::max(1, min_forward_points_for_ratio_);
  const bool enough_blocked_for_replan = blocked_points_count_ >= std::max(1, min_blocked_points_replanning_);
  const bool enough_blocked_for_wait = blocked_points_count_ >= std::max(1, min_blocked_points_wait_);
  const bool replan_triggered = enough_blocked_for_replan &&
    (can_use_ratio ? (prune_plan_blocked_ratio_ >= blocked_ratio_replanning_threshold_) : true);
  const bool wait_triggered = enough_blocked_for_wait &&
    (can_use_ratio ? (prune_plan_blocked_ratio_ >= blocked_ratio_wait_threshold_) : true);

  if(wait_triggered){
    auto now = node_->get_clock()->now();
    if(!blocked_since_set_){
      blocked_since_ = now;
      blocked_since_set_ = true;
    }

    if(blocked_hold_seconds_ <= 0.0 || (now - blocked_since_).seconds() >= blocked_hold_seconds_){
      opinion_ = perception_3d::PATH_BLOCKED_WAIT;
    }
    else{
      opinion_ = perception_3d::PATH_BLOCKED_REPLANNING;
    }
  }
  else if(replan_triggered){
    blocked_since_set_ = false;
    opinion_ = perception_3d::PATH_BLOCKED_REPLANNING;
  }
  else{
    blocked_since_set_ = false;
    opinion_ = perception_3d::PASS;
  }
  //RCLCPP_WARN(node_->get_logger().get_child(name_), "%s: blocked_ratio:%.2f", name_.c_str(), prune_plan_blocked_ratio_ );
}

void PathBlockedStrategy::selfClear(){
  shared_data_->dgraph_update_request_[name_] = false;
}

void PathBlockedStrategy::resetdGraph(){}

double PathBlockedStrategy::get_dGraphValue(const unsigned int index){
  return 9999.0;
}

bool PathBlockedStrategy::isCurrent(){
  
  current_ = true;

  return current_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PathBlockedStrategy::getObservation(){
  return sensor_current_observation_;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr PathBlockedStrategy::getLethal(){
  return current_lethal_;
}
}//end of name space