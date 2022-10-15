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

#include <memory>
#include <vector>
#include <string>
#include "algorithm_manager/executor_ab_navigation.hpp"

namespace cyberdog
{
namespace algorithm
{

ExecutorAbNavigation::ExecutorAbNavigation(std::string node_name)
: ExecutorBase(node_name)
{
  action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, "navigate_to_pose");

  // spin
  std::thread{[this]() {
    rclcpp::spin(this->get_node_base_interface());}
  }.detach();
}

void ExecutorAbNavigation::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  INFO("AB navigation started");
  ReportPreparationStatus();

  // Check all depends is ok
  bool ready = IsDependsReady();
  if (!ready) {
    ERROR("AB navigation lifecycle depend start up failed.");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    return;
  }

  // Check action client connect server
  bool connect = IsConnectServer();
  if (!connect) {
    ERROR("Connect navigation AB point server failed.");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    return;
  }

  // 结束激活进度的上报
  ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_SUCCESS);

  // Check input target goal is legal
  bool legal = IsLegal(goal);
  if (!legal) {
    ERROR("Current navigation AB point is not legal.");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    return;
  }

  // Print set target goal pose
  Debug2String(goal->poses[0]);

  // Send goal request
  if (!SendGoal(goal->poses[0])) {
    ERROR("Send navigation AB point send target goal request failed.");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    return;
  }

  INFO("Navigation AB point send target goal request success.");
  task_success_callback_();
}

void ExecutorAbNavigation::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  (void)request;
  INFO("Navigation AB will stop");
  if (nav_goal_handle_ != nullptr) {
    action_client_->async_cancel_goal(nav_goal_handle_);
  } else {
    task_abort_callback_();
  }

  StopReportPreparationThread();
  nav_goal_handle_.reset();
  response->result = OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause) ?
    StopTaskSrv::Response::SUCCESS :
    StopTaskSrv::Response::FAILED;
  INFO("Navigation AB Stoped");
  task_success_callback_();
}

void ExecutorAbNavigation::Cancel()
{
  INFO("Navigation AB Canceled");
}

void ExecutorAbNavigation::HandleGoalResponseCallback(
  NavigationGoalHandle::SharedPtr goal_handle)
{
  (void)goal_handle;
  INFO("Goal accepted");
}

void ExecutorAbNavigation::HandleFeedbackCallback(
    NavigationGoalHandle::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  // feedback_->feedback_code = feedback->exception_code;
  task_feedback_callback_(feedback_);
}

void ExecutorAbNavigation::HandleResultCallback(
  const NavigationGoalHandle::WrappedResult result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      INFO("Navigation AB run target goal succeeded");
      task_success_callback_();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      ERROR("Navigation AB run target goal aborted");
      task_abort_callback_();
      break;
    case rclcpp_action::ResultCode::CANCELED:
      ERROR("Navigation AB run target goal canceled");
      task_cancle_callback_();
      break;
    default:
      ERROR("Navigation AB run target goal unknown result code");
      task_abort_callback_();
      break;
  }
}

bool ExecutorAbNavigation::IsDependsReady()
{
  // Nav lifecycle
  if (!OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kStartUp)) {
    return false;
  }

  // Customize lifecycle and not depend Nav lifecycle
  // TODO(quan)
  return true;
}

bool ExecutorAbNavigation::IsConnectServer()
{
  std::chrono::seconds timeout(5);
  auto is_action_server_ready = action_client_->wait_for_action_server(timeout);
  
  if (!is_action_server_ready) {
    ERROR("navigation action server is not available.");
    OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
    return false;
  }
  return true;
}

bool ExecutorAbNavigation::IsLegal(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  return goal->poses.empty() ? false : true;
}

bool ExecutorAbNavigation::SendGoal(const geometry_msgs::msg::PoseStamped & pose)
{
  // Set orientation
  NormalizedGoal(pose);

  // Send the goal pose
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(
    &ExecutorAbNavigation::HandleGoalResponseCallback,
    this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(
    &ExecutorAbNavigation::HandleFeedbackCallback,
    this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(
    &ExecutorAbNavigation::HandleResultCallback,
    this, std::placeholders::_1);
  auto future_goal_handle = action_client_->async_send_goal(
    target_goal_, send_goal_options);

  if (future_goal_handle.wait_for(server_timeout_) != std::future_status::ready) {
    ERROR("Send Navigation AB goal failed");
    OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
    return false;
  }

  nav_goal_handle_ = future_goal_handle.get();
  if (!nav_goal_handle_) {
    ERROR("Navigation AB Goal was rejected by server");
    OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
    return false;
  }
  return true;
}

void ExecutorAbNavigation::NormalizedGoal(const geometry_msgs::msg::PoseStamped & pose)
{
  // Normalize the goal pose
  target_goal_.pose = pose;
  target_goal_.pose.header.frame_id = "map";
  target_goal_.pose.pose.orientation.w = 1;
}

void ExecutorAbNavigation::Debug2String(const geometry_msgs::msg::PoseStamped & pose)
{
  INFO("Nav target goal: [%f, %f]", pose.pose.position.x, pose.pose.position.y);
}

}  // namespace algorithm
}  // namespace cyberdog
