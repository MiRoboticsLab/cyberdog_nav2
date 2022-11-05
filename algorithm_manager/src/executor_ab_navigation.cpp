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

  // trigger slam
  stop_lidar_trigger_pub_ = create_publisher<std_msgs::msg::Bool>("stop_lidar_relocation", 10);
  stop_vision_trigger_pub_ = create_publisher<std_msgs::msg::Bool>("stop_vision_relocation", 10);

  // trigger navigation
  stop_nav_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "stop_nav_trigger",
    rclcpp::SystemDefaultsQoS(),
    std::bind(
      &ExecutorAbNavigation::HandleTriggerStopCallback, this,
      std::placeholders::_1));

  // localization_lifecycle_ = std::make_shared<LifecycleController>("localization_node");
  map_server_lifecycle_ = std::make_shared<LifecycleController>("map_server");

  nav_client_ = std::make_unique<nav2_lifecycle_manager::LifecycleManagerClient>(
    "lifecycle_manager_navigation");

  // spin
  std::thread{[this]() {
      rclcpp::spin(this->get_node_base_interface());
    }
  }.detach();
}

ExecutorAbNavigation::~ExecutorAbNavigation()
{
  nav_client_->reset();
  ReinitializeAndCleanup();
}

void ExecutorAbNavigation::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  INFO("AB navigation started");
  ReportPreparationStatus();

  // Set vision and lidar flag
  SetLocationType(goal->outdoor);

  // Check all depends is ok
  bool ready = IsDependsReady();
  if (!ready) {
    ERROR("AB navigation lifecycle depend start up failed.");
    ReportPreparationFinished(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
    task_abort_callback_();
    return;
  }

  // Check action client connect server
  bool connect = IsConnectServer();
  if (!connect) {
    ERROR("Connect navigation AB point server failed.");
    ReportPreparationFinished(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
    task_abort_callback_();
    return;
  }

  // Check input target goal is legal
  bool legal = IsLegal(goal);
  if (!legal) {
    ERROR("Current navigation AB point is not legal.");
    ReportPreparationFinished(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
    task_abort_callback_();
    return;
  }

  if (velocity_smoother_ == nullptr) {
    velocity_smoother_ = std::make_shared<nav2_util::ServiceClient<MotionServiceCommand>>(
      "velocity_adaptor_gait", shared_from_this());
  }

  // Smoother walk
  VelocitySmoother();

  // Print set target goal pose
  Debug2String(goal->poses[0]);

  // Send goal request
  if (!SendGoal(goal->poses[0])) {
    ERROR("Send navigation AB point send target goal request failed.");
    // Reset lifecycle nodes
    // LifecycleNodesReinitialize();

    ReportPreparationFinished(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
    task_abort_callback_();
    return;
  }

  // 结束激活进度的上报
  ReportPreparationFinished(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_SUCCESS);
  INFO("Navigation AB point send target goal request success.");
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
    SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
    task_abort_callback_();
    ERROR("Navigation AB will stop failed.");
  }

  nav_goal_handle_.reset();
  response->result = StopTaskSrv::Response::SUCCESS;

  StopReportPreparationThread();
  SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_SUCCESS);
  INFO("Navigation AB Stoped success");
  task_cancle_callback_();
}

void ExecutorAbNavigation::Cancel()
{
  // Reset lifecycle nodes
  INFO("Navigation AB Canceled");
}

void ExecutorAbNavigation::HandleGoalResponseCallback(
  NavigationGoalHandle::SharedPtr goal_handle)
{
  (void)goal_handle;
  INFO("Navigation AB Goal accepted");
}

void ExecutorAbNavigation::HandleFeedbackCallback(
  NavigationGoalHandle::SharedPtr,
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  INFO("Navigation feedback, distance_remaining : %f", feedback->distance_remaining);
  feedback_->feedback_code = AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_RUNNING;
  task_feedback_callback_(feedback_);
}

void ExecutorAbNavigation::HandleResultCallback(
  const NavigationGoalHandle::WrappedResult result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      INFO("Navigation AB point have arrived target goal success");
      SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_SUCCESS);
      task_success_callback_();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      ERROR("Navigation AB run target goal aborted");
      SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
      task_abort_callback_();
      break;
    case rclcpp_action::ResultCode::CANCELED:
      ERROR("Navigation AB run target goal canceled");
      SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
      task_cancle_callback_();
      break;
    default:
      ERROR("Navigation AB run target goal unknown result code");
      SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
      task_abort_callback_();
      break;
  }

  // Deactivate all nav2 lifecycle nodes
  // Reset lifecycle nodes
  // LifecycleNodesReinitialize();
}

void ExecutorAbNavigation::HandleTriggerStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  INFO("Reset location module and reset some sensor.");
  if (msg == nullptr) {
    return;
  }

  if (msg->data) {
    ReleaseSources();
  }
}

bool ExecutorAbNavigation::IsDependsReady()
{
  // Nav lifecycle
  // if (!OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kStartUp)) {
  //   return false;
  // }

  // bool success = LifecycleNodesConfigure();
  // if (!success) {
  //   return false;
  // }

  // success = LifecycleNodesStartup();
  //  if (!success) {
  //   return false;
  // }

  if (nav_client_->is_active() != nav2_lifecycle_manager::SystemStatus::ACTIVE) {
    if (!nav_client_->startup()) {
      WARN("Navigation client lifecycle startup failed.");
      return false;
    }
  }

  // map server lifecycle configure  and deactivate
  if (!map_server_lifecycle_->IsActivate()) {
    bool success = map_server_lifecycle_->Configure();
    if (!success) {
      ERROR("Configure map server lifecycle configure state failed.");
      return false;
    }

    success = map_server_lifecycle_->Startup();
    if (!success) {
      ERROR("Configure map server lifecycle activate state failed.");
      return false;
    }
  }
  return true;
}

bool ExecutorAbNavigation::IsConnectServer()
{
  std::chrono::seconds timeout(5);
  auto is_action_server_ready = action_client_->wait_for_action_server(timeout);

  if (!is_action_server_ready) {
    ERROR("Navigation action server is not available.");
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

  std::chrono::seconds timeout{5};
  if (future_goal_handle.wait_for(timeout) != std::future_status::ready) {
    ERROR("[Navigation AB] Wait navigation server timeout.");
    return false;
  }

  nav_goal_handle_ = future_goal_handle.get();
  if (!nav_goal_handle_) {
    ERROR("[Navigation AB] Navigation AB Goal was rejected by server");
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

bool ExecutorAbNavigation::ReinitializeAndCleanup()
{
  // return localization_lifecycle_->Pause() && localization_lifecycle_->Cleanup();
  return true;
}

bool ExecutorAbNavigation::LifecycleNodesReinitialize()
{
  // bool success = nav_client_->pause();
  // if (!success) {
  //   ERROR("Reset navigation lifecycle nodes deactivate state failed.");
  //   return false;
  // }

  // bool success = localization_lifecycle_->Pause();
  // if (!success) {
  //   ERROR("Reset localization lifecycle node deactivate state failed.");
  //   return false;
  // }
  return true;
}

bool ExecutorAbNavigation::VelocitySmoother()
{
  while (!velocity_smoother_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("[Navigation AB] Connect velocity adaptor service timeout");
      return false;
    }
  }

  // Set request data
  auto request = std::make_shared<MotionServiceCommand::Request>();
  std::vector<float> step_height{0.01, 0.01};
  request->motion_id = 303;
  request->value = 2;
  request->step_height = step_height;

  // Send request
  auto future_result = velocity_smoother_->invoke(request, std::chrono::seconds(5s));
  return future_result->result;
}

void ExecutorAbNavigation::Debug2String(const geometry_msgs::msg::PoseStamped & pose)
{
  INFO("Nav target goal: [%f, %f]", pose.pose.position.x, pose.pose.position.y);
}

void ExecutorAbNavigation::ReleaseSources()
{
  INFO("Release all sources and reset all lifecycle default state.");
  auto command = std::make_shared<std_msgs::msg::Bool>();
  command->data = true;
  stop_vision_trigger_pub_->publish(*command);

  // if (IsUseVisionLocation()) {
  //   stop_vision_trigger_pub_->publish(*command);
  // } else if (IsUseLidarLocation()) {
  //   stop_lidar_trigger_pub_->publish(*command);
  // }
  ResetDefaultValue();
}

bool ExecutorAbNavigation::IsUseVisionLocation()
{
  return use_vision_slam_;
}

bool ExecutorAbNavigation::IsUseLidarLocation()
{
  return use_lidar_slam_;
}

void ExecutorAbNavigation::SetLocationType(bool outdoor)
{
  if (outdoor) {
    INFO("Current location mode use vision slam.");
    use_vision_slam_ = true;
  } else {
    INFO("Current location mode use lidar slam.");
    use_lidar_slam_ = true;
  }
}

void ExecutorAbNavigation::ResetDefaultValue()
{
  use_vision_slam_ = false;
  use_lidar_slam_ = false;
}

}  // namespace algorithm
}  // namespace cyberdog
