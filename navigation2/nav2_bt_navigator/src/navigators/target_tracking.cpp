// Copyright (c) 2023 Samsung Research
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

#include <vector>
#include <string>
#include <set>
#include <memory>
#include <limits>
#include "nav2_core/exceptions.hpp"
#include <nav2_util/geometry_utils.hpp>
#include "nav2_bt_navigator/navigators/target_tracking.hpp"

namespace nav2_bt_navigator
{

bool
TargetTrackingNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();
  if (!node->has_parameter("goal_blackboard_relative_pos")) {
    node->declare_parameter("goal_blackboard_relative_pos", std::string("relative_pos"));
  }
  goal_blackboard_relative_pos_ = node->get_parameter("goal_blackboard_relative_pos").as_string();

  if (!node->has_parameter("goal_blackboard_keep_dist")) {
    node->declare_parameter("goal_blackboard_keep_dist", std::string("keep_distance"));
  }
  goal_blackboard_keep_dist_ = node->get_parameter("goal_blackboard_keep_dist").as_string();

  if (!node->has_parameter("target_topic")) {
    node->declare_parameter("target_topic", std::string("tracking_pose"));
  }
  target_topic_ = node->get_parameter("target_topic").as_string();
  
  if (!node->has_parameter("maxwait")) {
    node->declare_parameter("maxwait", 0.0);
  }
  maxwait_ = node->get_parameter("maxwait").as_double();

  // Odometry smoother object for getting current speed
  odom_smoother_ = std::make_unique<nav2_util::OdomSmoother>(node, 0.3);

  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

  goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    target_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&TargetTrackingNavigator::onGoalPoseReceived, this, std::placeholders::_1));
  return true;
}

std::string
TargetTrackingNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();
  std::string pkg_share_dir =
    ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
  node->declare_parameter<std::string>(
    "default_target_tracking_bt_xml",
    pkg_share_dir +
    "/behavior_trees/follow_point.xml");
  node->get_parameter("default_target_tracking_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool
TargetTrackingNavigator::cleanup()
{
  goal_sub_.reset();
  self_client_.reset();
  return true;
}
using namespace std::chrono_literals;
bool
TargetTrackingNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  start_time_ = clock_->now();
  rclcpp::WallRate r(500ms);
  if(maxwait_ > 1.0){
    while(rclcpp::ok()){
      RCLCPP_INFO(logger_, "Waiting for the destiny...");
      if(clock_->now().seconds() - latest_goal_.header.stamp.sec < 2.0){
        RCLCPP_INFO(logger_, "The tracking target has appeared in the last 2s and can be tracked.");
        break;
      }
      r.sleep();
      if(clock_->now().seconds() - start_time_.seconds() > maxwait_){
        RCLCPP_ERROR(logger_, "No target input in last %lf secs, unable to execute follow task.", maxwait_);
        return false;
      }
    }
  }

  auto bt_xml_filename = goal->behavior_tree;

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "BT file not found: %s. Navigation canceled.",
      bt_xml_filename.c_str());
    return false;
  }

  initializeGoalPose(goal);

  return true;
}

bool TargetTrackingNavigator::onGoalUpdate(FollowPoses::SharedPtr msg)
{
  (void)msg;
  return true;
}

void
TargetTrackingNavigator::goalCompleted(typename ActionT::Result::SharedPtr /*result*/)
{
}

void
TargetTrackingNavigator::onLoop()
{
  // action server feedback (pose, duration of task,
  // number of recoveries, and distance remaining to goal)
  auto feedback_msg = std::make_shared<ActionT::Feedback>();

  geometry_msgs::msg::PoseStamped current_pose;
  nav2_util::getCurrentPose(
    current_pose, *feedback_utils_.tf,
    feedback_utils_.global_frame, feedback_utils_.robot_frame,
    feedback_utils_.transform_tolerance);


  auto blackboard = bt_action_server_->getBlackboard();


  blackboard->get<float>("distance", feedback_msg->current_distance);
  int recovery_count = 0;
  blackboard->get<int>("number_recoveries", recovery_count);
  feedback_msg->number_of_recoveries = recovery_count;
  feedback_msg->tracking_time = clock_->now() - start_time_;
  unsigned int exception_code = 0;
  blackboard->get<unsigned int>("exception_code", exception_code);
  feedback_msg->exception_code = exception_code;
  bt_action_server_->publishFeedback(feedback_msg);
}

void
TargetTrackingNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Received goal preemption request");

  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
    (goal->behavior_tree.empty() &&
    bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
  {
    // if pending goal requests the same BT as the current goal, accept the pending goal
    // if pending goal has an empty behavior_tree field, it requests the default BT file
    // accept the pending goal if the current goal is running the default BT file
    initializeGoalPose(bt_action_server_->acceptPendingGoal());
  } else {
    RCLCPP_WARN(
      logger_,
      "Preemption request was rejected since the requested BT XML file is not the same "
      "as the one that the current goal is executing. Preemption with a new BT is invalid "
      "since it would require cancellation of the previous goal instead of true preemption."
      "\nCancel the current goal and send a new action request if you want to use a "
      "different BT XML file. For now, continuing to track the last goal until completion.");
    bt_action_server_->terminatePendingGoal();
  }
}

std::string dir_analysis(unsigned char dir)
{
  std::vector<std::string> dirs{"", "Behind", "Left", "Right"};
  return dirs[dir];
}

void
TargetTrackingNavigator::initializeGoalPose(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(
    logger_, "Start tracking target with direction: %s.", dir_analysis(goal->relative_pos).c_str());

  // Reset state for new action feedback

  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<int>("number_recoveries", 0);  // NOLINT

  // Update the goal pose on the blackboard
  blackboard->set<unsigned char>(goal_blackboard_relative_pos_, goal->relative_pos);
  blackboard->set<unsigned char>(goal_blackboard_keep_dist_, goal->keep_distance);
}

void
TargetTrackingNavigator::onGoalPoseReceived(
  const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  if(pose->header.frame_id == "")
    return;
  latest_goal_ = *pose;
  latest_goal_.header.stamp = clock_->now();
}

}  // namespace nav2_bt_navigator
