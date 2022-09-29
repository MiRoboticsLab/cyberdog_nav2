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

#ifndef ALGORITHM_MANAGER__ALGORITHM_TASK_MANAGER_HPP_
#define ALGORITHM_MANAGER__ALGORITHM_TASK_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "mcr_msgs/action/target_tracking.hpp"
// #include "nav2_util/geometry_utils.hpp"
#include "protocol/action/navigation.hpp"
// #include "protocol/msg/follow_points.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "std_srvs/srv/set_bool.hpp"
// #include "std_msgs/msg/int32.hpp"
// #include "cyberdog_common/cyberdog_log.hpp"
#include "algorithm_manager/realsense_lifecycle_manager.hpp"
#include "algorithm_manager/executor_base.hpp"
#include "algorithm_manager/executor_ab_navigation.hpp"
// #include "algorithm_manager/executor_auto_dock.hpp"
// #include "algorithm_manager/executor_laser_localization.hpp"
// #include "algorithm_manager/executor_laser_mapping.hpp"
// #include "algorithm_manager/executor_uwb_tracking.hpp"
// #include "algorithm_manager/executor_vision_tracking.hpp"
namespace cyberdog
{
namespace algorithm
{

using NavigationGoalHandle =
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
using WaypointFollowerGoalHandle =
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
using NavThroughPosesGoalHandle =
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>;
using TargetTrackingGoalHandle =
  rclcpp_action::ClientGoalHandle<mcr_msgs::action::TargetTracking>;

using GoalStatus = action_msgs::msg::GoalStatus;
using Navigation = protocol::action::Navigation;
using GoalHandleNavigation = rclcpp_action::ServerGoalHandle<Navigation>;
using RealSenseClient = RealSenseLifecycleServiceClient;

class AlgorithmTaskManager : public rclcpp::Node
{

public:
  AlgorithmTaskManager();
  ~AlgorithmTaskManager(){};

private:
  rclcpp_action::GoalResponse HandleNavigationGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Navigation::Goal> goal);
  rclcpp_action::CancelResponse HandleNavigationCancel(
    const std::shared_ptr<GoalHandleNavigation> goal_handle);
  void HandleNavigationAccepted(
    const std::shared_ptr<GoalHandleNavigation> goal_handle);
  void TaskExecute();
  // void TaskExecute(const std::shared_ptr<GoalHandleNavigation> goal_handle);
  void PublishFeedback();
  // void PublishFeedback(const std::shared_ptr<GoalHandleNavigation> goal_handle);

  rclcpp_action::Server<Navigation>::SharedPtr navigation_server_;
  std::shared_ptr<GoalHandleNavigation> goal_handle_;
  std::shared_ptr<ExecutorBase> executor_;

  // std::shared_ptr<ExecutorAbNavition> executor_ab_navigation_;
  // std::shared_ptr<ExecutorAutoDock> executor_ab_navigation_;

  // rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
  //   navigation_action_client_;
  // rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr
  //   waypoint_follower_action_client_;
  // rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr
  //   nav_through_poses_action_client_;
  // rclcpp_action::Client<mcr_msgs::action::TargetTracking>::SharedPtr
  //   target_tracking_action_client_;
  // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr reloc_sub_;

  // // Navigation action feedback subscribers
  // rclcpp::Subscription<
  //   nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr
  //   navigation_feedback_sub_;

  // rclcpp::Subscription<
  //   nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage>::SharedPtr
  //   nav_through_poses_feedback_sub_;

  // rclcpp::Subscription<
  //   nav2_msgs::action::NavigateToPose::Impl::GoalStatusMessage>::SharedPtr
  //   navigation_goal_status_sub_;

  // rclcpp::Subscription<
  //   nav2_msgs::action::NavigateThroughPoses::Impl::GoalStatusMessage>::
  // SharedPtr nav_through_poses_goal_status_sub_;

  // // Goal-related state
  // nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
  // nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal_;
  // nav2_msgs::action::NavigateThroughPoses::Goal nav_through_poses_goal_;
  // mcr_msgs::action::TargetTracking::Goal target_tracking_goal_;

  // rclcpp::Node::SharedPtr client_node_;
  // std::chrono::milliseconds server_timeout_;

  // // Goal handlers
  // NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  // WaypointFollowerGoalHandle::SharedPtr waypoint_follower_goal_handle_;
  // NavThroughPosesGoalHandle::SharedPtr nav_through_poses_goal_handle_;
  // TargetTrackingGoalHandle::SharedPtr target_tracking_goal_handle_;

  // // The client used to control the nav2 stack
  // nav2_lifecycle_manager::LifecycleManagerClient client_nav_;
  // nav2_lifecycle_manager::LifecycleManagerClient client_loc_;
  // std::unique_ptr<RealSenseClient> client_realsense_{nullptr};

  // // nav2_lifecycle_manager::LifecycleManagerClient client_data_;
  // nav2_lifecycle_manager::LifecycleManagerClient client_mapping_;
  // nav2_lifecycle_manager::LifecycleManagerClient client_mcr_uwb_;
  // rclcpp::TimerBase::SharedPtr nav_timer_;
  // rclcpp::TimerBase::SharedPtr loc_timer_;
  // rclcpp::TimerBase::SharedPtr waypoint_follow_timer_;
  // rclcpp::TimerBase::SharedPtr through_pose_timer_;
  // int status_;
  // ActionType action_type_;

};  // class algorithm_manager
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__ALGORITHM_TASK_MANAGER_HPP_
