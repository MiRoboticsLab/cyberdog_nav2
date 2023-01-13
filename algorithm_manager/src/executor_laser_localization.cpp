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
#include <functional>

#include "algorithm_manager/executor_laser_localization.hpp"

namespace cyberdog
{
namespace algorithm
{

ExecutorLaserLocalization::ExecutorLaserLocalization(std::string node_name)
: ExecutorBase(node_name),
  location_status_(LocationStatus::Unknown)
{
  localization_lifecycle_ = std::make_shared<LifecycleController>("localization_node");

  location_status_service_ = create_service<std_srvs::srv::SetBool>(
    "slam_location_status",
    std::bind(
      &ExecutorLaserLocalization::HandleLocationServiceCallback, this, std::placeholders::_1,
      std::placeholders::_2));

  // Subscription Lidar relocalization result
  relocalization_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "laser_reloc_result",
    rclcpp::SystemDefaultsQoS(),
    std::bind(
      &ExecutorLaserLocalization::HandleRelocalizationCallback, this,
      std::placeholders::_1));

  stop_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "stop_lidar_relocation",
    rclcpp::SystemDefaultsQoS(),
    std::bind(
      &ExecutorLaserLocalization::HandleStopTriggerCommandMessages, this,
      std::placeholders::_1));

  // spin
  std::thread{[this]() {
      rclcpp::spin(this->get_node_base_interface());
    }
  }.detach();
}

void ExecutorLaserLocalization::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  (void)goal;
  INFO("Laser Localization started");

  if (location_stop_function_starting_) {
    WARN("Current robot localization have not stopped, Laser Localization not started.");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_RELOCATION_FAILURE);
    return;
  }

  Timer timer_;
  timer_.Start();

  // Control lidar relocalization turn on
  if (start_client_ == nullptr) {
    start_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "start_location", shared_from_this());
  }

  // Control lidar relocalization turn off
  if (stop_client_ == nullptr) {
    stop_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "stop_location", shared_from_this());
  }

  // Control lidar mapping report realtime pose turn on and turn off
  if (realtime_pose_client_ == nullptr) {
    realtime_pose_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "PoseEnable", shared_from_this());
  }

  // 1 正在激活依赖节点
  UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_EXECUTING);
  bool ready = IsDependsReady();
  if (!ready) {
    ERROR("Laser localization lifecycle depend start up failed.");
    // 2 激活依赖节点失败
    UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    ResetLifecycleDefaultValue();
    location_status_ = LocationStatus::FAILURE;
    return;
  }
  // 3 激活依赖节点成功
  UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_SUCCESS);

  // Enable Relocalization
  bool success = EnableRelocalization();
  if (!success) {
    ERROR("Turn on relocalization failed.");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_RELOCATION_FAILURE);
    task_abort_callback_();
    location_status_ = LocationStatus::FAILURE;
    return;
  }

  // Send request and wait relocalization result success
  success = WaitRelocalization(std::chrono::seconds(120s));
  if (!success) {
    ERROR("Laser localization wait timeout, stop socalization.");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_RELOCING_FAILED);
    location_status_ = LocationStatus::FAILURE;
    StopLocalization();
    return;
  }

  // Check relocalization success
  if (!relocalization_success_) {
    ERROR("Lidar relocalization failed.");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_RELOCING_FAILED);
    task_abort_callback_();
    location_status_ = LocationStatus::FAILURE;
    StopLocalization();
    return;
  }

  // Enable report realtime robot pose
  success = CheckPoseServerActivate();
  if (success) {
    INFO("Current pose server is activated.");
  } else {
    success = EnableReportRealtimePose(true);
    if (!success) {
      ERROR("Enable report realtime robot pose failed.");
      UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_RELOCATION_FAILURE);
      task_abort_callback_();
      location_status_ = LocationStatus::FAILURE;
      StopLocalization();
      return;
    }
  }

  // 结束激活进度的上报
  UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_RELOCATION_SUCCESS);

  location_status_ = LocationStatus::SUCCESS;
  INFO("Laser localization success.");
  INFO("[Lidar Localization] Elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());

  bool activate_success = ActivateAllNavigationLifecycleNodes();
  if (!activate_success) {
    WARN("Activate all navigation lifecycle nodes failed.");
  }

  task_success_callback_();
}

void ExecutorLaserLocalization::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  (void)request;
  INFO("Laser localization will stop");
  Timer timer_;
  timer_.Start();

  location_stop_function_starting_ = true;

  // Disenable Relocalization
  bool success = DisenableRelocalization();
  if (!success) {
    ERROR("Turn off Laser relocalization failed.");
    response->result = StopTaskSrv::Response::FAILED;
    task_cancle_callback_();
    location_stop_function_starting_ = false;
    return;
  }

  // RealSense camera lifecycle
  success = LifecycleNodeManager::GetSingleton()->Pause(LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    response->result = StopTaskSrv::Response::FAILED;
    task_cancle_callback_();
    location_stop_function_starting_ = false;
    return;
  }

  // Nav lifecycle
  response->result = localization_lifecycle_->Pause() ?
    StopTaskSrv::Response::SUCCESS :
    StopTaskSrv::Response::FAILED;

  // Disenable report realtime robot pose
  success = CheckPoseServerActivate();
  if (success) {
    success = EnableReportRealtimePose(false);
    if (!success) {
      ERROR("Disenable report realtime robot pose failed.");
      task_cancle_callback_();
    }
  }

  task_cancle_callback_();
  INFO("Laser localization stoped success");
  INFO("[Lidar Localization] Elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());
  location_status_ = LocationStatus::Unknown;
  is_activate_ = false;
  location_stop_function_starting_ = false;
}

void ExecutorLaserLocalization::Cancel()
{
  INFO("Laser Localization canceled");
}

void ExecutorLaserLocalization::HandleLocationServiceCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  response->success = location_status_ == LocationStatus::SUCCESS ? true : false;
}

void ExecutorLaserLocalization::HandleRelocalizationCallback(
  const std_msgs::msg::Int32::SharedPtr msg)
{
  INFO("Relocalization result: %d", msg->data);
  if (msg->data == 0) {
    relocalization_success_ = true;
    INFO("Relocalization success.");
  } else if (msg->data == 100) {
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_RELOCING_RETRYING);
    WARN("Relocalization retrying.");
  } else if (msg->data == 200) {
    relocalization_failure_ = true;
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_RELOCING_FAILED);
    WARN("Relocalization failed.");
  }
}

void ExecutorLaserLocalization::HandleStopTriggerCommandMessages(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  INFO("Handle stop laser relocalization module.");
  if (msg == nullptr) {
    return;
  }

  if (!is_activate_) {
    INFO("Current laser localization not in activate, not need stop.");
    return;
  }

  if (msg->data) {
    auto request = std::make_shared<StopTaskSrv::Request>();
    auto response = std::make_shared<StopTaskSrv::Response>();
    Stop(request, response);
  }
}

bool ExecutorLaserLocalization::IsDependsReady()
{
  Timer timer_;
  timer_.Start();

  // RealSense camera lifecycle(configure state)
  bool success = LifecycleNodeManager::GetSingleton()->Configure(
    LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    return false;
  }

  // RealSense camera lifecycle(activate state)
  success = LifecycleNodeManager::GetSingleton()->Startup(
    LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    return false;
  }

  INFO(
    "[Laser Localization] RealSense camera elapsed time: %.5f [seconds]",
    timer_.ElapsedSeconds());

  // localization_node lifecycle(configure state)
  if (!localization_lifecycle_->Configure()) {
    return false;
  }

  // localization_node lifecycle(activate state)
  if (!localization_lifecycle_->Startup()) {
    return false;
  }

  is_activate_ = true;
  INFO(
    "[Laser Localization] localization_node elapsed time: %.5f [seconds]",
    timer_.ElapsedSeconds());
  return true;
}

bool ExecutorLaserLocalization::WaitRelocalization(std::chrono::seconds timeout)
{
  auto end = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && !relocalization_success_) {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {
      WARN("Wait relocalization result timeout.");
      return false;
    }

    if (relocalization_failure_) {
      ERROR("Relocalization result failure.");
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return true;
}

bool ExecutorLaserLocalization::EnableRelocalization()
{
  // Wait service
  while (!start_client_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("Waiting for the service. but cannot connect the service.");
      return false;
    }
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  // Send request
  // return start_->invoke(request, response);
  bool result = false;
  try {
    auto future_result = start_client_->invoke(request, std::chrono::seconds(50s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }
  return result;
}

bool ExecutorLaserLocalization::DisenableRelocalization()
{
  // Wait service
  while (!stop_client_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("Waiting for the service. but cannot connect the service.");
      return false;
    }
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  // Send request
  // return start_->invoke(request, response);
  bool result = false;
  try {
    auto future_result = stop_client_->invoke(request, std::chrono::seconds(10s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }
  return result;
}

bool ExecutorLaserLocalization::EnableReportRealtimePose(bool enable)
{
  while (!realtime_pose_client_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("Waiting for the service. but cannot connect the service.");
      return false;
    }
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enable;

  // Print enable and disenable message
  if (enable) {
    INFO("Start report robot's realtime pose");
  } else {
    INFO("Stop report robot's realtime pose.");
  }

  // Send request
  // return start_->invoke(request, response);
  bool result = false;
  try {
    auto future_result = realtime_pose_client_->invoke(request, std::chrono::seconds(5s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }
  return result;
}

void ExecutorLaserLocalization::StopLocalization()
{
  auto request = std::make_shared<StopTaskSrv::Request>();
  auto response = std::make_shared<StopTaskSrv::Response>();
  Stop(request, response);
}

bool ExecutorLaserLocalization::ActivateAllNavigationLifecycleNodes()
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

bool ExecutorLaserLocalization::CheckPoseServerActivate()
{
  if (pose_server_client_ == nullptr) {
    pose_server_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "pose_server_state", shared_from_this());
  }

  while (!pose_server_client_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("Waiting for the service. but cannot connect the service.");
      return false;
    }
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  // Send request
  bool result = false;
  try {
    auto future_result = pose_server_client_->invoke(request, std::chrono::seconds(5s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }
  return result;
}

bool ExecutorLaserLocalization::ResetLifecycleDefaultValue()
{
  bool success = LifecycleNodeManager::GetSingleton()->Pause(
    LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    ERROR("Release RealSense failed.");
  }

  return success;
}

}  // namespace algorithm
}  // namespace cyberdog
