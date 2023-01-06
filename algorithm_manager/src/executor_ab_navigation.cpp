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
#include <utility>

#include "algorithm_manager/executor_ab_navigation.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "filesystem/filesystem.hpp"

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

  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

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
  Timer timer_;
  timer_.Start();

  // Check current map exits
  bool exist = CheckMapAvailable();
  if (!exist) {
    ERROR("AB navigation can't start up, because current robot's map not exist");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
    task_cancle_callback_();
    return;
  }

  // Set vision and lidar flag
  bool outdoor = false;
  if (CheckUseOutdoor(outdoor)) {
    SetLocationType(outdoor);
  }

  // Check all depends is ok
  bool ready = IsDependsReady();
  if (!ready) {
    ERROR("AB navigation lifecycle depend start up failed.");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
    task_abort_callback_();
    return;
  }

  INFO("[Navigation AB] Depend sensors Elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());

  // Check action client connect server
  bool connect = IsConnectServer();
  if (!connect) {
    ERROR("Connect navigation AB point server failed.");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
    DeactivateDepsLifecycleNodes();
    task_abort_callback_();
    return;
  }

  // Check input target goal is legal
  bool legal = IsLegal(goal);
  if (!legal) {
    ERROR("Current navigation AB point is not legal.");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
    DeactivateDepsLifecycleNodes();
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
    DeactivateDepsLifecycleNodes();
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
    task_abort_callback_();
    return;
  }

  // 结束激活进度的上报
  UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_SUCCESS);
  INFO("Navigation AB point send target goal request success.");
  INFO("[Navigation AB] Elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());
}

void ExecutorAbNavigation::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  (void)request;
  Timer timer_;
  timer_.Start();

  INFO("Navigation AB will stop");
  bool cancel = ShouldCancelGoal();
  if (!cancel) {
    WARN("Current robot can't stop, due to navigation status is not available.");
    nav_goal_handle_.reset();
    // task_cancle_callback_();
    return;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  if (nav_goal_handle_ != nullptr) {
    auto server_ready = action_client_->wait_for_action_server(std::chrono::seconds(5));
    if (!server_ready) {
      ERROR("Navigation action server is not available.");
      return;
    }

    // async_cancel_goal will throw exceptions::UnknownGoalHandleError()
    try {
      auto future_cancel = action_client_->async_cancel_goal(nav_goal_handle_);
      PublishZeroPath();
    } catch (const std::exception & e) {
      ERROR("%s", e.what());
      return;
    }
  } else {
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
    task_abort_callback_();
    ERROR("Navigation AB will stop failed.");
    return;
  }

  nav_goal_handle_.reset();
  response->result = StopTaskSrv::Response::SUCCESS;
  UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_SUCCESS);
  INFO("Navigation AB Stoped success");
  INFO("[Navigation AB] Elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());
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
  (void)feedback;
  // INFO("Navigation feedback, distance_remaining : %f", feedback->distance_remaining);
  UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_RUNNING);
}

void ExecutorAbNavigation::HandleResultCallback(
  const NavigationGoalHandle::WrappedResult result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      INFO("Navigation AB point have arrived target goal success");
      UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_SUCCESS);
      task_success_callback_();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      ERROR("Navigation AB run target goal aborted");
      UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
      ResetPreprocessingValue();
      // task_abort_callback_();
      break;
    case rclcpp_action::ResultCode::CANCELED:
      ERROR("Navigation AB run target goal canceled");
      // UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
      // task_cancle_callback_();
      break;
    default:
      ERROR("Navigation AB run target goal unknown result code");
      // UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_NAVIGATING_AB_FAILURE);
      // task_abort_callback_();
      break;
  }
  // nav_goal_handle_.reset();
}

void ExecutorAbNavigation::HandleTriggerStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg == nullptr) {
    return;
  }

  // Set vision and lidar flag
  bool outdoor = false;
  if (CheckUseOutdoor(outdoor)) {
    SetLocationType(outdoor);
  }

  if (msg->data) {
    ReleaseSources();
  }

  if (!outdoor) {
    INFO("Trigger reset laser location module.");
  } else {
    INFO("Trigger reset vision location module.");
  }

  // stop current navigation ab
  auto request = std::make_shared<StopTaskSrv::Request>();
  auto response = std::make_shared<StopTaskSrv::Response>();
  Stop(request, response);
}

bool ExecutorAbNavigation::IsDependsReady()
{
  // Nav lifecycle
  if (!ActivateDepsLifecycleNodes(this->get_name())) {
    DeactivateDepsLifecycleNodes();
    return false;
  }

  // start_lifecycle_depend_finished_ = true;
  INFO("Call function IsDependsReady() finished.");
  return true;
}

bool ExecutorAbNavigation::IsConnectServer()
{
  if (connect_server_finished_) {
    INFO("Current server have connected.");
    return true;
  }

  std::chrono::seconds timeout(5);
  auto is_action_server_ready = action_client_->wait_for_action_server(timeout);

  if (!is_action_server_ready) {
    ERROR("Navigation action server is not available.");
    return false;
  }

  connect_server_finished_ = true;
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
  time_goal_sent_ = this->now();

  std::chrono::seconds timeout{5};
  if (future_goal_handle.wait_for(timeout) != std::future_status::ready) {
    ERROR("Wait navigation server timeout.");
    return false;
  }

  nav_goal_handle_ = future_goal_handle.get();
  if (!nav_goal_handle_) {
    ERROR("Navigation AB Goal was rejected by server");
    return false;
  }
  return true;
}

bool ExecutorAbNavigation::ShouldCancelGoal()
{
  // No need to cancel the goal if goal handle is invalid
  if (!nav_goal_handle_) {
    return false;
  }

  // Check if the goal is still executing
  auto status = nav_goal_handle_->get_status();
  NavigationStatus2String(status);

  return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
         status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
}


bool ExecutorAbNavigation::CheckTimeout()
{
  if (nav_goal_handle_) {
    auto elapsed = (this->now() - time_goal_sent_).to_chrono<std::chrono::milliseconds>();
    if (!IsFutureGoalHandleComplete(elapsed)) {
      // return RUNNING if there is still some time before timeout happens
      if (elapsed < server_timeout_) {
        return false;
      }
      // if server has taken more time than the specified timeout value return FAILURE
      WARN("Timed out while waiting for action server to acknowledge goal request.");
      nav_goal_handle_.reset();
      return true;
    }
  }
}

bool ExecutorAbNavigation::IsFutureGoalHandleComplete(std::chrono::milliseconds & elapsed)
{
  auto remaining = server_timeout_ - elapsed;
  // server has already timed out, no need to sleep
  if (remaining <= std::chrono::milliseconds(0)) {
    nav_goal_handle_.reset();
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
  return true;
}

bool ExecutorAbNavigation::VelocitySmoother()
{
  if (start_velocity_smoother_finished_) {
    INFO("Current start velocity smoother finished.");
    return true;
  }

  while (!velocity_smoother_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("Connect velocity adaptor service timeout");
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
  bool result = false;
  try {
    auto future_result = velocity_smoother_->invoke(request, std::chrono::seconds(5s));
    result = future_result->result;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }

  start_velocity_smoother_finished_ = true;
  return result;
}

void ExecutorAbNavigation::Debug2String(const geometry_msgs::msg::PoseStamped & pose)
{
  INFO("Nav target goal: [%f, %f]", pose.pose.position.x, pose.pose.position.y);
}

void ExecutorAbNavigation::NavigationStatus2String(int8_t status)
{
  if (status == action_msgs::msg::GoalStatus::STATUS_UNKNOWN) {
    INFO("Bt navigation return status : STATUS_UNKNOWN");
  } else if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED) {
    INFO("Bt navigation return status : STATUS_ACCEPTED");
  } else if (status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
    INFO("Bt navigation return status : STATUS_EXECUTING");
  } else if (status == action_msgs::msg::GoalStatus::STATUS_CANCELING) {
    INFO("Bt navigation return status : STATUS_CANCELING");
  } else if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    INFO("Bt navigation return status : STATUS_SUCCEEDED");
  } else if (status == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
    INFO("Bt navigation return status : STATUS_CANCELED");
  } else if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
    INFO("Bt navigation return status : STATUS_ABORTED");
  }
}

void ExecutorAbNavigation::ReleaseSources()
{
  INFO("Reset all lifecycle default state.");
  auto command = std::make_shared<std_msgs::msg::Bool>();
  command->data = true;

  if (IsUseVisionLocation()) {
    stop_vision_trigger_pub_->publish(*command);
  } else if (IsUseLidarLocation()) {
    stop_lidar_trigger_pub_->publish(*command);
  }

  LifecycleNodesReinitialize();
  ResetDefaultValue();
}

bool ExecutorAbNavigation::CheckMapAvailable(const std::string & map_name)
{
  std::string map_filename = "/home/mi/mapping/" + map_name;
  if (!filesystem::exists(map_filename)) {
    ERROR("Navigation's map file is not exist.");
    return false;
  }

  return true;
}

bool ExecutorAbNavigation::CheckUseOutdoor(bool & outdoor, const std::string & filename)
{
  std::string map_json_filename = "/home/mi/mapping/" + filename;
  if (!filesystem::exists(map_json_filename)) {
    ERROR("Navigation's map json file is not exist.");
    return false;
  }

  rapidjson::Document document(rapidjson::kObjectType);
  common::CyberdogJson::ReadJsonFromFile(map_json_filename, document);

  for (auto it = document.MemberBegin(); it != document.MemberEnd(); ++it) {
    INFO("it->name = %s", it->name.GetString());
    INFO("it->value = %d", it->value.GetBool());

    std::string key = it->name.GetString();
    if (key == "is_outdoor") {
      outdoor = it->value.GetBool();
      INFO("Function CheckUseOutdoor() get is_outdoor: %d", outdoor);
      break;
    }
  }
  return true;
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

void ExecutorAbNavigation::ResetPreprocessingValue()
{
  connect_server_finished_ = false;
  start_lifecycle_depend_finished_ = false;
  start_velocity_smoother_finished_ = false;
}

void ExecutorAbNavigation::PublishZeroPath()
{
  auto msg = std::make_unique<nav_msgs::msg::Path>();
  msg->poses.clear();
  plan_publisher_->publish(std::move(msg));
}

}  // namespace algorithm
}  // namespace cyberdog
