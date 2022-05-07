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

#include "motion_core/motion_core_node.hpp"
#include <memory>
#include <vector>
namespace CARPO_NAVIGATION
{
using namespace std::chrono_literals;
NavigationCore::NavigationCore()
: Node("NavigationCore"),
  server_timeout_(20),
  client_nav_("lifecycle_manager_navigation"),
  client_loc_(
    "lifecycle_manager_localization"),
  status_(GoalStatus::STATUS_UNKNOWN),
  action_type_(ACTION_NONE)
{
  auto options =
    rclcpp::NodeOptions().arguments(
    {"--ros-args --remap __node:=navigation_dialog_action_client"});
  client_node_ =
    std::make_shared<rclcpp::Node>(
    "_",
    options);

  navigation_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    client_node_, "navigate_to_pose");
  waypoint_follower_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    client_node_, "follow_waypoints");
  nav_through_poses_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
    client_node_,
    "navigate_through_poses");

  navigation_goal_ =
    nav2_msgs::action::NavigateToPose::Goal();
  waypoint_follower_goal_ =
    nav2_msgs::action::FollowWaypoints::Goal();
  nav_through_poses_goal_ =
    nav2_msgs::action::NavigateThroughPoses::
    Goal();

  navigation_server_ =
    rclcpp_action::create_server<Navigation>(
    this, "CyberdogNavigation",
    std::bind(
      &NavigationCore::
      handle_navigation_goal, this,
      _1, _2),
    std::bind(
      &NavigationCore::
      handle_navigation_cancel,
      this, _1),
    std::bind(
      &NavigationCore::
      handle_navigation_accepted,
      this, _1));

  onInitialize();
  points_pub_ =
    this->create_publisher<protocol::msg::FollowPoints>(
    "follow_points",
    rclcpp::SystemDefaultsQoS());

  points_subscriber_ =
    this->create_subscription<protocol::msg::FollowPoints>(
    "subsequent_follow_points",
    rclcpp::SystemDefaultsQoS(),
    std::bind(
      &NavigationCore::
      follwPointCallback, this,
      _1));
}

void NavigationCore::follwPointCallback(
  const protocol::msg::FollowPoints::SharedPtr msg)
{
  protocol::msg::FollowPoints msg_;
  int status;
  ACTION_TYPE action_type;
  getNavStatus(status, action_type);
  if ((action_type == ACTION_THROUGH_POSE) &&
    (msg->poses.size() > 0))
  {
    msg_.poses = msg->poses;
    msg_.token = "0xafic29casckdek";
    msg_.header = returnHeader();
    points_pub_->publish(msg_);
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "action type: %d error or follow points empty",
      action_type);
  }
}

std_msgs::msg::Header NavigationCore::returnHeader()
{
  std_msgs::msg::Header msg;
  msg.frame_id = "NavigationCore";
  msg.stamp = this->get_clock()->now();
  return msg;
}

rclcpp_action::GoalResponse NavigationCore::handle_navigation_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Navigation::Goal> goal)
{
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::
         ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigationCore::handle_navigation_cancel(
  const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Received request to cancel goal");
  (void)goal_handle;
  onCancel();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigationCore::handle_navigation_accepted(
  const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  goal_handle_ = goal_handle;
  std::thread{std::bind(
      &NavigationCore::follow_execute,
      this, _1), goal_handle}
  .detach();
}

void NavigationCore::follow_execute(
  const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result =
    std::make_shared<Navigation::Result>();
  RCLCPP_INFO(
    this->get_logger(), "Executing goal");
  switch (goal->nav_type) {
    case Navigation::Goal::
      NAVIGATION_GOAL_TYPE_AB: {
        if (goal->poses.empty()) {
          RCLCPP_INFO(
            this->get_logger(),
            "empty pose ");
          goal_handle->succeed(result);
        }

        uint8_t goal_result = startNavigation(
          goal->poses[0]);
        if (goal_result !=
          Navigation::Result::
          NAVIGATION_RESULT_TYPE_ACCEPT)
        {
          // goal process failed
          result->result = goal_result;
          goal_handle->succeed(result);
        }
      } break;

    case Navigation::Goal::
      NAVIGATION_GOAL_TYPE_FOLLOW: {
        uint8_t goal_result =
          startNavThroughPoses(goal->poses);
        if (goal_result !=
          Navigation::Result::
          NAVIGATION_RESULT_TYPE_ACCEPT)
        {
          goal_handle->succeed(result);
        }
      } break;

    default:
      result->result =
        Navigation::Result::
        NAVIGATION_RESULT_TYPE_REJECT;
      goal_handle->succeed(result);
      break;
  }
}

void NavigationCore::onInitialize()
{
  // create action feedback subscribers
  navigation_feedback_sub_ =
    this->create_subscription<
    nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
    "navigate_to_pose/_action/feedback",
    rclcpp::SystemDefaultsQoS(),
    [this](const nav2_msgs::action::
    NavigateToPose::Impl::
    FeedbackMessage::
    SharedPtr msg) {
      (void)msg;
      // navigation_feedback_indicator_->setText(getNavToPoseFeedbackLabel(msg->feedback));
    });
  nav_through_poses_feedback_sub_ =
    this->create_subscription<
    nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage>(
    "navigate_through_poses/_action/feedback",
    rclcpp::SystemDefaultsQoS(),
    [this](const nav2_msgs::action::
    NavigateThroughPoses::Impl::
    FeedbackMessage::SharedPtr msg) {
      (void)msg;
      // navigation_feedback_indicator_->setText(getNavThroughPosesFeedbackLabel(msg->feedback));
    });

  // create action goal status subscribers
  navigation_goal_status_sub_ =
    this->create_subscription<action_msgs::msg::GoalStatusArray>(
    "navigate_to_pose/_action/status",
    rclcpp::SystemDefaultsQoS(),
    [this](const action_msgs::msg::
    GoalStatusArray::SharedPtr msg) {
      (void)msg;
      // navigation_goal_status_indicator_->setText(
      //   getGoalStatusLabel(msg->status_list.back().status));
      if (msg->status_list.back().status !=
      action_msgs::msg::GoalStatus::
      STATUS_EXECUTING)
      {
        // navigation_feedback_indicator_->setText(getNavToPoseFeedbackLabel());
      }
    });
  nav_through_poses_goal_status_sub_ =
    this->create_subscription<action_msgs::msg::GoalStatusArray>(
    "navigate_through_poses/_action/status",
    rclcpp::SystemDefaultsQoS(),
    [this](const action_msgs::msg::
    GoalStatusArray::SharedPtr msg) {
      (void)msg;
      // navigation_goal_status_indicator_->setText(
      //   getGoalStatusLabel(msg->status_list.back().status));
      if (msg->status_list.back().status !=
      action_msgs::msg::GoalStatus::
      STATUS_EXECUTING)
      {
        // navigation_feedback_indicator_->setText(getNavThroughPosesFeedbackLabel());
      }
    });
}

uint8_t NavigationCore::startNavigation(
  geometry_msgs::msg::PoseStamped pose)
{
  auto is_action_server_ready =
    navigation_action_client_->
    wait_for_action_server(
    std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "navigate_to_pose action server is not available."
      " Is the initial pose set?");
    return Navigation::Result::
           NAVIGATION_RESULT_TYPE_UNAVALIBLE;
  }

  // Send the goal pose
  navigation_goal_.pose = pose;

  RCLCPP_INFO(
    client_node_->get_logger(),
    "NavigateToPose will be called using the BT Navigator's default "
    "behavior tree.");

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<
    nav2_msgs::action::NavigateToPose>::
    SendGoalOptions();
  send_goal_options.result_callback =
    [this](auto) {
      RCLCPP_ERROR(
        client_node_->get_logger(),
        "Get navigate to poses result");
      senResult();
      navigation_goal_handle_
      .reset();
    };

  auto future_goal_handle =
    navigation_action_client_->
    async_send_goal(
    navigation_goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(
      client_node_, future_goal_handle,
      server_timeout_)
    !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "Send goal call failed");
    return Navigation::Result::
           NAVIGATION_RESULT_TYPE_FAILED;
  }

  // Get the goal handle and save so that we can check on completion in the
  // timer callback
  navigation_goal_handle_ =
    future_goal_handle.get();
  if (!navigation_goal_handle_) {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "Goal was rejected by server");
    return Navigation::Result::
           NAVIGATION_RESULT_TYPE_REJECT;
  }

  nav_timer_ = this->create_wall_timer(
    200ms,
    std::bind(
      &NavigationCore::
      getCurrentNavStatus,
      this));
  return Navigation::Result::
         NAVIGATION_RESULT_TYPE_ACCEPT;
}

uint8_t NavigationCore::startNavThroughPoses(
  std::vector<geometry_msgs::msg::PoseStamped> poses)
{
  auto is_action_server_ready =
    nav_through_poses_action_client_->
    wait_for_action_server(
    std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "navigate_through_poses action server is not available."
      " Is the initial pose set?");
    return Navigation::Result::
           NAVIGATION_RESULT_TYPE_UNAVALIBLE;
  }

  nav_through_poses_goal_.poses = poses;
  RCLCPP_INFO(
    client_node_->get_logger(),
    "NavigateThroughPoses will be called using the BT Navigator's "
    "default behavior tree.");

  RCLCPP_DEBUG(
    client_node_->get_logger(), "Sending a path of %zu waypoints:",
    nav_through_poses_goal_.poses.size());
  for (auto waypoint :
    nav_through_poses_goal_.poses)
  {
    RCLCPP_DEBUG(
      client_node_->get_logger(), "\t(%lf, %lf)",
      waypoint.pose.position.x,
      waypoint.pose.position.y);
  }

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<
    nav2_msgs::action::NavigateThroughPoses>
    ::SendGoalOptions();
  send_goal_options.result_callback =
    [this](auto) {
      RCLCPP_ERROR(
        client_node_->get_logger(),
        "Get navigate_through_poses result");
      nav_through_poses_goal_handle_
      .reset();
      senResult();
    };

  auto future_goal_handle =
    nav_through_poses_action_client_->
    async_send_goal(
    nav_through_poses_goal_,
    send_goal_options);
  if (rclcpp::spin_until_future_complete(
      client_node_, future_goal_handle,
      server_timeout_)
    !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "Send goal call failed");
    return Navigation::Result::
           NAVIGATION_RESULT_TYPE_FAILED;
  }

  // Get the goal handle and save so that we can check on completion in the
  // timer callback
  nav_through_poses_goal_handle_ =
    future_goal_handle.get();
  if (!nav_through_poses_goal_handle_) {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "Goal was rejected by server");
    return Navigation::Result::
           NAVIGATION_RESULT_TYPE_REJECT;
  }

  through_pose_timer_ =
    this->create_wall_timer(
    200ms,
    std::bind(
      &NavigationCore::
      getCurrentNavStatus,
      this));
  return Navigation::Result::
         NAVIGATION_RESULT_TYPE_ACCEPT;
}

uint8_t NavigationCore::startWaypointFollowing(
  std::vector<geometry_msgs::msg::PoseStamped> poses)
{
  auto is_action_server_ready =
    waypoint_follower_action_client_->
    wait_for_action_server(
    std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "follow_waypoints action server is not available."
      " Is the initial pose set?");
    return Navigation::Result::
           NAVIGATION_RESULT_TYPE_UNAVALIBLE;
  }

  // Send the goal poses
  waypoint_follower_goal_.poses = poses;

  RCLCPP_DEBUG(
    client_node_->get_logger(), "Sending a path of %zu waypoints:",
    waypoint_follower_goal_.poses.size());
  for (auto waypoint :
    waypoint_follower_goal_.poses)
  {
    RCLCPP_DEBUG(
      client_node_->get_logger(), "\t(%lf, %lf)",
      waypoint.pose.position.x,
      waypoint.pose.position.y);
  }

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<
    nav2_msgs::action::FollowWaypoints>::
    SendGoalOptions();
  send_goal_options.result_callback =
    [this](auto) {
      RCLCPP_ERROR(
        client_node_->get_logger(),
        "Get follow waypoints result");
      senResult();
      waypoint_follower_goal_handle_
      .reset();
    };

  auto future_goal_handle =
    waypoint_follower_action_client_->
    async_send_goal(
    waypoint_follower_goal_,
    send_goal_options);
  if (rclcpp::spin_until_future_complete(
      client_node_, future_goal_handle,
      server_timeout_)
    !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "Send goal call failed");
    return Navigation::Result::
           NAVIGATION_RESULT_TYPE_FAILED;
  }

  // Get the goal handle and save so that we can check on completion in the
  // timer callback
  waypoint_follower_goal_handle_ =
    future_goal_handle.get();
  if (!waypoint_follower_goal_handle_) {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "Goal was rejected by server");
    return Navigation::Result::
           NAVIGATION_RESULT_TYPE_REJECT;
  }

  waypoint_follow_timer_ =
    this->create_wall_timer(
    200ms,
    std::bind(
      &NavigationCore::
      getCurrentNavStatus,
      this));
  return Navigation::Result::
         NAVIGATION_RESULT_TYPE_ACCEPT;
}

void NavigationCore::getNavStatus(
  int & status, ACTION_TYPE & action_type)
{
  status = status_;
  action_type = action_type_;
}

void NavigationCore::getCurrentNavStatus()
{
  RCLCPP_ERROR(
    client_node_->get_logger(),
    "getCurrentNavStatus ");

  if (!waypoint_follower_goal_handle_ &&
    !nav_through_poses_goal_handle_ &&
    !navigation_goal_handle_)
  {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "Waiting for Goal");
    // state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
    action_type_ = ACTION_NONE;
    return;
  } else if (waypoint_follower_goal_handle_) {
    rclcpp::spin_some(client_node_);
    status_ =
      waypoint_follower_goal_handle_->
      get_status();
    action_type_ = ACTION_WAYPOINT;
    // Check if the goal is still executing
    if (status_ ==
      GoalStatus::STATUS_ACCEPTED ||
      status_ ==
      GoalStatus::STATUS_EXECUTING)
    {
      // state_machine_.postEvent(new ROSActionQEvent(QActionState::ACTIVE));
    } else {
      // state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
      waypoint_follow_timer_->cancel();
      RCLCPP_ERROR(
        client_node_->get_logger(),
        "Way point follow finished");
      senResult();
    }
  } else if (nav_through_poses_goal_handle_) {
    rclcpp::spin_some(client_node_);
    status_ =
      nav_through_poses_goal_handle_->
      get_status();
    action_type_ = ACTION_THROUGH_POSE;
    // Check if the goal is still executing
    if (status_ ==
      GoalStatus::STATUS_ACCEPTED ||
      status_ ==
      GoalStatus::STATUS_EXECUTING)
    {
      // state_machine_.postEvent(new ROSActionQEvent(QActionState::ACTIVE));
    } else {
      // state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
      RCLCPP_ERROR(
        client_node_->get_logger(),
        "through poses finished");
      through_pose_timer_->cancel();
      senResult();
    }
  } else if (navigation_goal_handle_) {
    rclcpp::spin_some(client_node_);
    status_ =
      navigation_goal_handle_->get_status();
    action_type_ = ACTION_NAVIGATION;
    // Check if the goal is still executing
    if (status_ ==
      GoalStatus::STATUS_ACCEPTED ||
      status_ ==
      GoalStatus::STATUS_EXECUTING)
    {
      // state_machine_.postEvent(new ROSActionQEvent(QActionState::ACTIVE));
    } else {
      // state_machine_.postEvent(new ROSActionQEvent(QActionState::INACTIVE));
      RCLCPP_ERROR(
        client_node_->get_logger(),
        "navigation to pose finished");
      nav_timer_->cancel();
      senResult();
    }
  }
}
void NavigationCore::senResult()
{
  auto result =
    std::make_shared<Navigation::Result>();
  result->result =
    Navigation::Result::
    NAVIGATION_RESULT_TYPE_SUCCESS;
  goal_handle_->succeed(result);
  action_type_ = ACTION_NONE;
}
void NavigationCore::onCancel()
{
  if (!waypoint_follower_goal_handle_ &&
    !nav_through_poses_goal_handle_ &&
    !navigation_goal_handle_)
  {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "nothing to cancel");
  }
  if (navigation_goal_handle_) {
    auto future_cancel =
      navigation_action_client_->
      async_cancel_goal(
      navigation_goal_handle_);

    if (rclcpp::spin_until_future_complete(
        client_node_, future_cancel,
        server_timeout_)
      !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(
        client_node_->get_logger(),
        "Failed to cancel goal");
    } else {
      navigation_goal_handle_.reset();
      RCLCPP_ERROR(
        client_node_->get_logger(),
        "canceled navigation goal");
    }
    if (!nav_timer_->is_canceled()) {
      RCLCPP_ERROR(
        client_node_->get_logger(),
        "canceled navigation goal timer");
      nav_timer_->cancel();
    }
  }

  if (waypoint_follower_goal_handle_) {
    auto future_cancel =
      waypoint_follower_action_client_->
      async_cancel_goal(
      waypoint_follower_goal_handle_);

    if (rclcpp::spin_until_future_complete(
        client_node_, future_cancel,
        server_timeout_)
      !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(
        client_node_->get_logger(),
        "Failed to cancel waypoint follower");
    } else {
      waypoint_follower_goal_handle_.reset();
      RCLCPP_ERROR(
        client_node_->get_logger(),
        "canceled waypoint follower");
    }
    waypoint_follow_timer_->cancel();
  }

  if (nav_through_poses_goal_handle_) {
    auto future_cancel =
      nav_through_poses_action_client_->
      async_cancel_goal(
      nav_through_poses_goal_handle_);

    if (rclcpp::spin_until_future_complete(
        client_node_, future_cancel,
        server_timeout_)
      !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(
        client_node_->get_logger(),
        "Failed to cancel nav through pose action");
    } else {
      nav_through_poses_goal_handle_.reset();
      RCLCPP_ERROR(
        client_node_->get_logger(),
        "canceled nav through pose action");
    }
    through_pose_timer_->cancel();
  }
}
// a service to start ab point.

// a service to start follow mode

// a topic to send follow mode poses.

}  // namespace CARPO_NAVIGATION
