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

#include "nav2_bt_navigator/navigators/automatic_recharge.hpp"

#include <limits>
#include <memory>
#include <nav2_util/geometry_utils.hpp>
#include <set>
#include <string>
#include <vector>

namespace nav2_bt_navigator
{

bool AutomaticRechargeNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  start_time_ = rclcpp::Time(0);
  RCLCPP_ERROR(logger_, "AutomaticRechargeNavigator configure:.");
  auto node = parent_node.lock();
  if (!node->has_parameter("charging_station")) {
    node->declare_parameter(
      "charging_station",
      std::string("charging_station"));
  }
  charging_station_ = node->get_parameter("charging_station").as_string();

  if (!node->has_parameter("goal_blackboard_keep_dist")) {
    node->declare_parameter(
      "goal_blackboard_keep_dist",
      std::string("keep_distance"));
  }
  goal_blackboard_keep_dist_ =
    node->get_parameter("goal_blackboard_keep_dist").as_string();

  // Odometry smoother object for getting current speed
  odom_smoother_ = std::make_unique<nav2_util::OdomSmoother>(node, 0.3);

  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

  goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "charging_station", rclcpp::SystemDefaultsQoS(),
    std::bind(
      &AutomaticRechargeNavigator::onGoalPoseReceived, this,
      std::placeholders::_1));
  return true;
}

std::string AutomaticRechargeNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();
  std::string pkg_share_dir =
    ament_index_cpp::get_package_share_directory("mcr_bringup");
  node->declare_parameter<std::string>(
    "default_automatic_recharge_bt_xml",
    pkg_share_dir + "/behavior_trees/automatic_recharge.xml");
  node->get_parameter(
    "default_automatic_recharge_bt_xml",
    default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool AutomaticRechargeNavigator::cleanup()
{
  goal_sub_.reset();
  self_client_.reset();
  return true;
}

bool AutomaticRechargeNavigator::goalReceived(
  ActionT::Goal::ConstSharedPtr goal)
{
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

bool AutomaticRechargeNavigator::onGoalUpdate(FollowPoses::SharedPtr msg)
{
  (void)msg;
  return true;
}

void AutomaticRechargeNavigator::goalCompleted(
  typename ActionT::Result::SharedPtr /*result*/) {}

void AutomaticRechargeNavigator::onLoop()
{
  // action server feedback (pose, duration of task,
  // number of recoveries, and distance remaining to goal)
  auto feedback_msg = std::make_shared<ActionT::Feedback>();

  geometry_msgs::msg::PoseStamped current_pose;
  nav2_util::getCurrentPose(
    current_pose, *feedback_utils_.tf, feedback_utils_.global_frame,
    feedback_utils_.robot_frame, feedback_utils_.transform_tolerance);

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

void AutomaticRechargeNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Received goal preemption request");

  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
    (goal->behavior_tree.empty() &&
    bt_action_server_->getCurrentBTFilename() ==
    bt_action_server_->getDefaultBTFilename()))
  {
    // if pending goal requests the same BT as the current goal, accept the
    // pending goal if pending goal has an empty behavior_tree field, it
    // requests the default BT file accept the pending goal if the current goal
    // is running the default BT file
    initializeGoalPose(bt_action_server_->acceptPendingGoal());
  } else {
    RCLCPP_WARN(
      logger_,
      "Preemption request was rejected since the requested BT XML "
      "file is not the same "
      "as the one that the current goal is executing. Preemption "
      "with a new BT is invalid "
      "since it would require cancellation of the previous goal "
      "instead of true preemption."
      "\nCancel the current goal and send a new action request if "
      "you want to use a "
      "different BT XML file. For now, continuing to track the last "
      "goal until completion.");
    bt_action_server_->terminatePendingGoal();
  }
}

void AutomaticRechargeNavigator::initializeGoalPose(
  ActionT::Goal::ConstSharedPtr /*goal*/)
{
  RCLCPP_INFO(logger_, "Start automatic recharge.");
  // Reset state for new action feedback
  start_time_ = clock_->now();
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<int>("number_recoveries", 0);  // NOLINT

  // Update the goal pose on the blackboard
  // blackboard->set<unsigned char>(charging_station_, goal->charging_station);
}

void AutomaticRechargeNavigator::onGoalPoseReceived(
  const geometry_msgs::msg::PoseStamped::SharedPtr /*pose*/)
{
  ActionT::Goal goal;
  // goal.keep_distance = 1.0;
  // goal.behavior_tree
  self_client_->async_send_goal(goal);
}

}  // namespace nav2_bt_navigator
