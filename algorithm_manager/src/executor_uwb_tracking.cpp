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
#include "algorithm_manager/executor_uwb_tracking.hpp"

namespace cyberdog
{
namespace algorithm
{

ExecutorUwbTracking::ExecutorUwbTracking(std::string node_name)
: ExecutorBase(node_name)
{
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args", "-r", std::string("__node:=") + get_name() + "_client", "--"});
  action_client_node_ = std::make_shared<rclcpp::Node>("_", options);
  target_tracking_action_client_ =
    rclcpp_action::create_client<mcr_msgs::action::TargetTracking>(
    action_client_node_, "tracking_target");
  GetBehaviorManager()->RegisterStateCallback(
    std::bind(&ExecutorUwbTracking::UpdateBehaviorStatus, this, std::placeholders::_1));
  std::thread{[this]() {rclcpp::spin(action_client_node_);}}.detach();
}

void ExecutorUwbTracking::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  mcr_msgs::action::TargetTracking_Goal target_tracking_goal;
  target_tracking_goal.keep_distance = goal->keep_distance;
  static uint8_t last_relative_pos = AlgorithmMGR::Goal::TRACING_AUTO;
  switch (goal->relative_pos) {
    case AlgorithmMGR::Goal::TRACING_AUTO:
      target_tracking_goal.relative_pos = McrTargetTracking::Goal::AUTO;
      break;

    case AlgorithmMGR::Goal::TRACING_LEFT:
      target_tracking_goal.relative_pos = McrTargetTracking::Goal::LEFT;
      break;

    case AlgorithmMGR::Goal::TRACING_RIGHT:
      target_tracking_goal.relative_pos = McrTargetTracking::Goal::RIGHT;
      break;

    case AlgorithmMGR::Goal::TRACING_BEHIND:
      target_tracking_goal.relative_pos = McrTargetTracking::Goal::BEHIND;
      break;

    default:
      INFO("Get Invalid tracking pos %d, will use last: %d", goal->relative_pos, last_relative_pos);
      target_tracking_goal.relative_pos = last_relative_pos;
      break;
  }
  last_relative_pos = goal->relative_pos;
  INFO("UWB Tracking will start");
  // 在激活依赖节点前需要开始上报激活进度
  // ReportPreparationStatus();
  UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_EXECUTING);

  if (!ActivateDepsLifecycleNodes(this->get_name())) {
    // ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    DeactivateDepsLifecycleNodes();
    task_abort_callback_();
    return;
  }
  // static bool first_run = true;
  // bool result = false;
  // if (first_run) {
  //   result = OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kStartUp);
  // } else {
  //   result = OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kResume);
  // }
  // if (!result) {
  //   ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
  //   DeactivateDepsLifecycleNodes();
  //   task_abort_callback_();
  //   return;
  // }
  // first_run = false;
  // TODO(Harvey): 当有Realsense的依赖时
  // TODO(Harvey): 当有其他没有继承Nav2Lifecycle的依赖节点时
  auto is_action_server_ready =
    target_tracking_action_client_->wait_for_action_server(
    std::chrono::seconds(5));
  if (!is_action_server_ready) {
    ERROR("TrackingTarget action server is not available.");
    OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
    DeactivateDepsLifecycleNodes();
    // ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    return;
  }
  // 结束激活进度的上报
  // ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_SUCCESS);
  UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_SUCCESS);
  // Send the goal pose
  auto send_goal_options =
    rclcpp_action::Client<McrTargetTracking>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(
    &ExecutorUwbTracking::HandleGoalResponseCallback,
    this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(
    &ExecutorUwbTracking::HandleFeedbackCallback,
    this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(
    &ExecutorUwbTracking::HandleResultCallback,
    this, std::placeholders::_1);
  auto future_goal_handle = target_tracking_action_client_->async_send_goal(
    target_tracking_goal, send_goal_options);
  if (future_goal_handle.wait_for(server_timeout_) != std::future_status::ready) {
    ERROR("Send TrackingTarget goal failed");
    OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
    DeactivateDepsLifecycleNodes();
    task_abort_callback_();
    return;
  }
  target_tracking_goal_handle_ = future_goal_handle.get();
  if (!target_tracking_goal_handle_) {
    ERROR("TrackingTarget Goal was rejected by server");
    OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
    DeactivateDepsLifecycleNodes();
    task_abort_callback_();
    return;
  }
  INFO("UWB Tracking started");
}

void ExecutorUwbTracking::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  (void)request;
  INFO("UWB Tracking will stop");
  OnCancel(response);
  INFO("UWB Tracking Stoped");
}

void ExecutorUwbTracking::Cancel()
{
  INFO("UWB Tracking will cancel");
  OnCancel();
  INFO("UWB Tracking Canceled");
}

void ExecutorUwbTracking::OnCancel(StopTaskSrv::Response::SharedPtr response)
{
  std::unique_lock<std::mutex> lk(target_tracking_server_mutex_);
  if (target_tracking_goal_handle_ != nullptr) {
    // 只有在向底层执行器发送目标后才需要发送取消指令
    target_tracking_action_client_->async_cancel_goal(target_tracking_goal_handle_);
    if (target_tracking_server_cv_.wait_for(lk, 5s) == std::cv_status::timeout) {
      cancel_tracking_result_ = false;
    } else {
      cancel_tracking_result_ = true;
    }
  } else {
    DeactivateDepsLifecycleNodes();
    OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
    task_abort_callback_();
  }
  StopReportPreparationThread();
  target_tracking_goal_handle_.reset();
  GetBehaviorManager()->Launch(false, false);
  GetBehaviorManager()->Reset();
  if (response == nullptr) {
    return;
  }
  response->result = cancel_tracking_result_ ?
    StopTaskSrv::Response::SUCCESS : StopTaskSrv::Response::FAILED;
}

void ExecutorUwbTracking::UpdateBehaviorStatus(const BehaviorManager::BehaviorStatus & status)
{
  behavior_status_ = status;
  int32_t feedback_code = -1;
  switch (behavior_status_) {
    case BehaviorManager::BehaviorStatus::kStairJumping:
      feedback_code =
        AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_BASE_TRACKING_STAIRJUMPING;
      break;

    case BehaviorManager::BehaviorStatus::kAutoTracking:
      feedback_code =
        AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_BASE_TRACKING_AUTOTRACKING;
      break;

    case BehaviorManager::BehaviorStatus::kAbnorm:
      feedback_code =
        AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_BASE_TRACKING_BEHAVIORABNORM;

    default:
      return;
  }
  // task_feedback_callback_(feedback_);
  UpdateFeedback(feedback_code);
}


void ExecutorUwbTracking::HandleFeedbackCallback(
  TargetTrackingGoalHandle::SharedPtr,
  const std::shared_ptr<const McrTargetTracking::Feedback> feedback)
{
  if (behavior_status_ != BehaviorManager::BehaviorStatus::kNormTracking) {
    return;
  }
  INFO_MILLSECONDS(1000, "Get TargetTracking Feedback: %d", feedback->exception_code);
  int32_t feedback_code = -1;
  switch (feedback->exception_code) {
    case 0:
      feedback_code =
        AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_BASE_TRACKING_NOEXCEPTION;
      break;

    case 1000:
      feedback_code =
        AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_BASE_TRACKING_DETECOTOREXCEPTION;
      break;

    case 2000:
      feedback_code =
        AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_BASE_TRACKING_TFEXCEPTION;
      break;

    case 3000:
      feedback_code =
        AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_BASE_TRACKING_PLANNNEREXCEPTION;
      break;

    case 4000:
      feedback_code =
        AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_BASE_TRACKING_CONTROLLEREXCEPTION;
      break;

    default:
      ERROR("Get unkown feedback %d from TargetTracking:", feedback->exception_code);
      return;
  }
  // task_feedback_callback_(feedback_);
  UpdateFeedback(feedback_code);
}

void ExecutorUwbTracking::HandleResultCallback(const TargetTrackingGoalHandle::WrappedResult result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      ERROR("UWB Tracking reported succeeded, this should never happened");
      // DeactivateDepsLifecycleNodes();
      // OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
      // task_abort_callback_();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      ERROR("UWB Tracking reported aborted, this should never happened");
      target_tracking_goal_handle_.reset();
      feedback_->feedback_code =
        AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_BASE_TRACKING_EMPTY_TARGET;
      task_feedback_callback_(feedback_);
      // DeactivateDepsLifecycleNodes();
      // OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
      // task_abort_callback_();
      break;
    case rclcpp_action::ResultCode::CANCELED:
      INFO("UWB Tracking reported canceled");
      DeactivateDepsLifecycleNodes();
      OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
      task_cancle_callback_();
      target_tracking_server_cv_.notify_one();
      break;
    default:
      ERROR("UWB Tracking reported unknown result code");
      // DeactivateDepsLifecycleNodes();
      // OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause);
      // task_abort_callback_();
      break;
  }
}
}  // namespace algorithm
}  // namespace cyberdog
