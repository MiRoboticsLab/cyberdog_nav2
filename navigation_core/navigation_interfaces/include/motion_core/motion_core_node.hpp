// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#ifndef MOTION_CORE__MOTION_CORE_NODE_HPP_
#define MOTION_CORE__MOTION_CORE_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "protocol/action/navigation.hpp"
#include "protocol/msg/follow_points.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

typedef enum
{
  ACTION_NONE,
  ACTION_NAVIGATION,
  ACTION_WAYPOINT,
  ACTION_THROUGH_POSE,
} ACTION_TYPE;

namespace CARPO_NAVIGATION
{
using std::placeholders::_1;
using std::placeholders::_2;
class NavigationCore : public rclcpp::Node
{
public:
  using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using WaypointFollowerGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
  using NavThroughPosesGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>;

  using GoalStatus = action_msgs::msg::GoalStatus;
  using Navigation = protocol::action::Navigation;
  using GoalHandleNavigation = rclcpp_action::ServerGoalHandle<Navigation>;

  NavigationCore();
  ~NavigationCore() = default;

public:
  void onInitialize();

// start a2b navigation
  uint8_t startNavigation(geometry_msgs::msg::PoseStamped pose);

// cancel a navigation request
  void onCancel();

// way point following
  uint8_t startWaypointFollowing(
    std::vector<geometry_msgs::msg::PoseStamped> poses);

// start through poses
  uint8_t startNavThroughPoses(
    std::vector<geometry_msgs::msg::PoseStamped> poses);

// get current navstatus.
  void getCurrentNavStatus();

private:
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
    navigation_action_client_;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr
    waypoint_follower_action_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr
    nav_through_poses_action_client_;

// Navigation action feedback subscribers
  rclcpp::Subscription<
    nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr
    navigation_feedback_sub_;

  rclcpp::Subscription<
    nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage>::SharedPtr
    nav_through_poses_feedback_sub_;

  rclcpp::Subscription<
    nav2_msgs::action::NavigateToPose::Impl::GoalStatusMessage>::SharedPtr
    navigation_goal_status_sub_;

  rclcpp::Subscription<
    nav2_msgs::action::NavigateThroughPoses::Impl::GoalStatusMessage>::
  SharedPtr nav_through_poses_goal_status_sub_;

// Goal-related state
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
  nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal_;
  nav2_msgs::action::NavigateThroughPoses::Goal nav_through_poses_goal_;

  rclcpp::Node::SharedPtr client_node_;
  std::chrono::milliseconds server_timeout_;

// Goal handlers
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  WaypointFollowerGoalHandle::SharedPtr waypoint_follower_goal_handle_;
  NavThroughPosesGoalHandle::SharedPtr nav_through_poses_goal_handle_;

// The client used to control the nav2 stack
  nav2_lifecycle_manager::LifecycleManagerClient client_nav_;
  nav2_lifecycle_manager::LifecycleManagerClient client_loc_;
  rclcpp::TimerBase::SharedPtr nav_timer_;
  rclcpp::TimerBase::SharedPtr waypoint_follow_timer_;
  rclcpp::TimerBase::SharedPtr through_pose_timer_;
  int status_;
  ACTION_TYPE action_type_;
  void getNavStatus(int & status, ACTION_TYPE & action_type);

  rclcpp_action::Server<Navigation>::SharedPtr navigation_server_;

  rclcpp_action::GoalResponse handle_navigation_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Navigation::Goal> goal);

  rclcpp_action::CancelResponse handle_navigation_cancel(
    const std::shared_ptr<GoalHandleNavigation> goal_handle);

  void handle_navigation_accepted(
    const std::shared_ptr<GoalHandleNavigation> goal_handle);

  void follow_execute(const std::shared_ptr<GoalHandleNavigation> goal_handle);

// save goal handle to local
  std::shared_ptr<GoalHandleNavigation> goal_handle_;
  void senResult();

  rclcpp::Publisher<protocol::msg::FollowPoints>::SharedPtr points_pub_;
  rclcpp::Subscription<protocol::msg::FollowPoints>::SharedPtr
    points_subscriber_;

  void follwPointCallback(const protocol::msg::FollowPoints::SharedPtr msg);
  std_msgs::msg::Header returnHeader();
};
}  // namespace CARPO_NAVIGATION
#endif  // MOTION_CORE__MOTION_CORE_NODE_HPP_
