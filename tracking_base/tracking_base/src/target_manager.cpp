// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <string>
#include <memory>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "mcr_msgs/action/target_tracking.hpp"
#include "tracking_base/target_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "Eigen/Dense"
#include "Eigen/QR"


namespace tracking_base
{

using std::placeholders::_1;

TargetManager::TargetManager(rclcpp_lifecycle::LifecycleNode::SharedPtr node){
  node_ = node;

  distance_ = 0.0;
  std::string goal_updater_topic;

  nav2_util::declare_parameter_if_not_declared(
    node_, "global_frame",
    rclcpp::ParameterValue("map"));
  node_->get_parameter("global_frame", global_frame_);

  nav2_util::declare_parameter_if_not_declared(
    node_, "goal_updater_topic",
    rclcpp::ParameterValue("tracking_pose"));
  node_->get_parameter("goal_updater_topic", goal_updater_topic);

  nav2_util::declare_parameter_if_not_declared(
    node_, "max_pose_inuse", rclcpp::ParameterValue(5));
  node_->get_parameter("max_pose_inuse", max_pose_inuse_);

  nav2_util::declare_parameter_if_not_declared(
    node_, "dist_throttle", rclcpp::ParameterValue(0.3));
  node_->get_parameter("dist_throttle", dist_sq_throttle_);
  dist_sq_throttle_ *= dist_sq_throttle_;

  nav2_util::declare_parameter_if_not_declared(
    node_, "keep_distance", rclcpp::ParameterValue(1.3));
  node_->get_parameter("keep_distance", keep_distance_);

  nav2_util::declare_parameter_if_not_declared(
    node_, "overtime", rclcpp::ParameterValue(6.0));
  node_->get_parameter("overtime", overtime_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node_->get_node_base_interface(),
    node_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_updater_topic, rclcpp::SensorDataQoS(),
    std::bind(&TargetManager::callback_updated_goal, this, _1));
}

bool TargetManager::getTarget(geometry_msgs::msg::PoseStamped& target)
{
  if (last_goal_received_.header.frame_id == "" || 
      (node_->now().seconds() - latest_timestamp_.seconds()) > overtime_) {
    RCLCPP_WARN(node_->get_logger(), "The target pose may be lost or invalid. %lf", overtime_);
    historical_poses_.clear();
    return false;
  }

  target = last_goal_transformed_;
  return true;
}

geometry_msgs::msg::PoseStamped
TargetManager::translatePoseByMode(const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::TransformStamped transform;
  geometry_msgs::msg::PoseStamped tpose = pose;
  transform.header = pose.header;
  transform.child_frame_id = pose.header.frame_id;
  double N = 4.0;

  unsigned char cur_mode = current_mode_;

  if(cur_mode == mcr_msgs::action::TargetTracking::Goal::AUTO){
    double yaw = tf2::getYaw(pose.pose.orientation);
    double x0 = pose.pose.position.x;
    double y0 = pose.pose.position.y;
    double x1 = x0 + cos(yaw);
    double y1 = y0 + sin(yaw);
    cur_mode = (x0 * y1 - x1 * y0) > 0 ? 
                mcr_msgs::action::TargetTracking::Goal::LEFT : 
                mcr_msgs::action::TargetTracking::Goal::RIGHT;
  }

  switch (cur_mode) {

    case mcr_msgs::action::TargetTracking::Goal::LEFT: {
        //左侧 1m
        double yaw = tf2::getYaw(pose.pose.orientation);
        transform.transform.translation.x = 2.0 * cos(yaw + 3.14 / N);
        transform.transform.translation.y = 2.0 * sin(yaw + 3.14 / N);
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        break;
      }
    case mcr_msgs::action::TargetTracking::Goal::RIGHT: {
        //右侧 1m
        double yaw = tf2::getYaw(pose.pose.orientation);
        transform.transform.translation.x = 2.0 * cos(yaw - 3.14 / N);
        transform.transform.translation.y = 2.0 * sin(yaw - 3.14 / N);
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        break;
      }
    case mcr_msgs::action::TargetTracking::Goal::BEHIND:
    default:
      {
        // 后方1m
        // double yaw = tf2::getYaw(pose.pose.orientation);
        double yaw = atan2(pose.pose.position.y, pose.pose.position.x);
        transform.transform.translation.x = -1.0 * keep_distance_ * cos(yaw);
        transform.transform.translation.y = -1.0 * keep_distance_ * sin(yaw);
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        tpose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
        
        break;
      }
  }
  //当前跟随模式对应的方位
  geometry_msgs::msg::PoseStamped transformed_pose;
  tf2::doTransform(tpose, transformed_pose, transform);

  //该模式下的跟随位姿对应在全局坐标系上的位姿
  geometry_msgs::msg::PoseStamped global_frame_pose;

  if (!nav2_util::transformPoseInTargetFrame(
      transformed_pose, global_frame_pose, *tf_buffer_,
      global_frame_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Faild transform target pose to %s", global_frame_.c_str());
    throw nav2_core::TFException("Transformed error in target updater node");
  }
  global_frame_pose.pose.position.z = 0.0;
  return global_frame_pose;
}



bool TargetManager::isValid(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if(msg->header.frame_id == ""){
    RCLCPP_WARN(node_->get_logger(), "invalid data for target's frame id is null.");
    return false;
  }

  geometry_msgs::msg::PoseStamped pose_based_on_global_frame;
  if(!nav2_util::transformPoseInTargetFrame(
    *msg, pose_based_on_global_frame, *tf_buffer_, global_frame_, 1.2)){
    RCLCPP_WARN(node_->get_logger(), "Failed to transform pose in target updater.");
    return false;
  }
  if (poseDistanceSq(last_goal_received_.pose, pose_based_on_global_frame.pose) <
    dist_sq_throttle_)
  {
    return false;
  }
  last_goal_received_ = pose_based_on_global_frame;
  return true;
}

void
TargetManager::callback_updated_goal(const geometry_msgs::msg::PoseStamped::SharedPtr m)
{
  
  distance_ = hypot(m->pose.position.x, m->pose.position.y);

  latest_timestamp_ = node_->now();
  if (!isValid(m)) {
    return;
  }

  geometry_msgs::msg::PoseStamped msg_with_orientation = *m;
  last_raw_goal_ = *m;

  if (distance_ > 8.0) {
    double scale = 8.0 / distance_;
    msg_with_orientation.pose.position.x *= scale;
    msg_with_orientation.pose.position.y *= scale;
  }


  last_goal_transformed_ = translatePoseByMode(msg_with_orientation);

}

}  // namespace tracking_base
