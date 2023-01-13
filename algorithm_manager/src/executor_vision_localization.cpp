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

#include "algorithm_manager/executor_vision_localization.hpp"

namespace cyberdog
{
namespace algorithm
{

ExecutorVisionLocalization::ExecutorVisionLocalization(std::string node_name)
: ExecutorBase(node_name)
{
  localization_lifecycle_ = std::make_shared<LifecycleController>("mivinslocalization");
  // localization_client_ = std::make_unique<nav2_lifecycle_manager::LifecycleManagerClient>(
  //   "lifecycle_manager_localization");

  // Subscription Vision relocalization result
  relocalization_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "reloc_result",
    rclcpp::SystemDefaultsQoS(),
    std::bind(
      &ExecutorVisionLocalization::HandleRelocalizationCallback, this,
      std::placeholders::_1));

  stop_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "stop_vision_relocation",
    rclcpp::SystemDefaultsQoS(),
    std::bind(
      &ExecutorVisionLocalization::HandleStopTriggerCommandMessages, this,
      std::placeholders::_1));

  // // Control vision relocalization turn on
  // start_client_ = create_client<std_srvs::srv::SetBool>(
  //   "start_vins_location", rmw_qos_profile_services_default);

  // // Control vision relocalization turn off
  // stop_client_ = create_client<std_srvs::srv::SetBool>(
  //   "stop_vins_location", rmw_qos_profile_services_default);

  // // Control vision mapping report realtime pose turn on and turn off
  // realtime_pose_client_ = create_client<std_srvs::srv::SetBool>(
  //   "PoseEnable", rmw_qos_profile_services_default);

  // spin
  std::thread{[this]() {
      rclcpp::spin(this->get_node_base_interface());
    }
  }.detach();
}

void ExecutorVisionLocalization::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  (void)goal;
  INFO("Vision Localization started");
  // ReportPreparationStatus();
  Timer timer_;
  timer_.Start();

  // Check current map available
  bool available = CheckMapAvailable();
  if (!available) {
    ERROR("Vision build map file not available.");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_RELOCATION_FAILURE);
    task_abort_callback_();
    return;
  }

  // 1 正在激活依赖节点
  UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_EXECUTING);
  bool ready = IsDependsReady();
  if (!ready) {
    ResetLifecycleDefaultValue();
    ERROR("Vision localization lifecycle depend start up failed.");
    // 2 激活依赖节点失败
    UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    return;
  }

  // 3 激活依赖节点成功
  UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_SUCCESS);

  if (start_client_ == nullptr) {
    start_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "start_vins_location", shared_from_this());
  }

  if (stop_client_ == nullptr) {
    stop_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "stop_vins_location", shared_from_this());
  }

  if (realtime_pose_client_ == nullptr) {
    realtime_pose_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "PoseEnable", shared_from_this());
  }

  // Enable Relocalization
  bool success = EnableRelocalization();
  if (!success) {
    ERROR("Turn on relocalization failed.");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_RELOCATION_FAILURE);
    ResetLifecycleDefaultValue();
    task_abort_callback_();
    return;
  }

  // Send request and wait relocalization result success
  success = WaitRelocalization(std::chrono::seconds(60s));
  if (!success) {
    ERROR("Vision localization failed.");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_RELOCATION_FAILURE);
    ResetLifecycleDefaultValue();
    task_abort_callback_();
    return;
  }

  // Check relocalization success
  if (!relocalization_success_) {
    ERROR("Vision relocalization failed.");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_RELOCATION_FAILURE);
    ResetLifecycleDefaultValue();
    task_abort_callback_();
    return;
  }

  // Enable report realtime robot pose
  auto pose_thread = std::make_shared<std::thread>(
    [&]() {
      int try_count = 0;
      while (true) {
        if (relocalization_failure_) {
          break;
        }

        if (!relocalization_success_) {
          std::this_thread::sleep_for(std::chrono::seconds(1));
          continue;
        }

        try_count++;
        success = EnableReportRealtimePose(true);

        if (success) {
          INFO("Enable report realtime robot pose success.");
          try_count = 0;
          break;
        }

        if (try_count >= 3 && !success) {
          ERROR("Enable report realtime robot pose failed.");
          UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_RELOCATION_FAILURE);
          ResetLifecycleDefaultValue();
          task_abort_callback_();
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    });
  pose_thread->detach();

  // 结束激活进度的上报
  UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_RELOCATION_SUCCESS);
  INFO("Vision localization success.");
  INFO("[Vision Localization] Elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());
  task_success_callback_();
}

void ExecutorVisionLocalization::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  (void)request;
  INFO("Vision localization will stop");
  // StopReportPreparationThread();

  Timer timer_;
  timer_.Start();

  // Disenable Relocalization
  bool success = DisenableRelocalization();
  if (!success) {
    ERROR("Turn off Vision relocalization failed.");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_RELOCATION_FAILURE);
    task_abort_callback_();
    ResetLifecycleDefaultValue();
    return;
  }

  // Disenable report realtime robot pose
  success = EnableReportRealtimePose(false);
  if (!success) {
    ERROR("Disenable report realtime robot pose failed.");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_RELOCATION_FAILURE);
    task_abort_callback_();
    return;
  }

  // RealSense camera lifecycle
  success = LifecycleNodeManager::GetSingleton()->Pause(
    LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    response->result = StopTaskSrv::Response::FAILED;
    task_abort_callback_();
    return;
  }

  // RGB-D camera lifecycle(dectivate state)
  success = LifecycleNodeManager::GetSingleton()->Pause(
    LifeCycleNodeType::RGBCameraSensor);
  if (!success) {
    response->result = StopTaskSrv::Response::FAILED;
    task_abort_callback_();
    return;
  }

  response->result = localization_lifecycle_->Pause() ?
    StopTaskSrv::Response::SUCCESS :
    StopTaskSrv::Response::FAILED;

  INFO("Vision Localization stoped success");
  INFO("[Vision Localization] Elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());
  UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_RELOCATION_SUCCESS);

  is_activate_ = false;
  task_success_callback_();
}

void ExecutorVisionLocalization::Cancel()
{
  INFO("Vision Localization canceled");
}

void ExecutorVisionLocalization::HandleRelocalizationCallback(
  const std_msgs::msg::Int32::SharedPtr msg)
{
  INFO("Relocalization result: %d", msg->data);
  if (msg->data == 0) {
    relocalization_success_ = true;
    INFO("Relocalization success.");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_RELOCING_SUCCESS);
  } else if (msg->data == 100) {
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_RELOCING_RETRYING);
    WARN("Relocalization retrying.");
  } else if (msg->data == 200) {
    relocalization_failure_ = true;
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_RELOCING_FAILED);
    WARN("Relocalization failed.");
  }
}

void ExecutorVisionLocalization::HandleStopTriggerCommandMessages(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  INFO("Handle stop vision relocalization module.");
  if (msg == nullptr) {
    return;
  }

  if (!is_activate_) {
    INFO("Current vision localization not in activate, not need stop.");
    return;
  }

  if (msg->data) {
    auto request = std::make_shared<StopTaskSrv::Request>();
    auto response = std::make_shared<StopTaskSrv::Response>();
    Stop(request, response);
  }
}

bool ExecutorVisionLocalization::IsDependsReady()
{
  Timer timer_;
  timer_.Start();

  bool success = LifecycleNodeManager::GetSingleton()->IsActivate(
    LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    // RealSense camera lifecycle(configure state)
    success = LifecycleNodeManager::GetSingleton()->Configure(
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
  } else {
    INFO("RealSense Camera Sensor is activate.");
  }

  INFO("[Vision Mapping] RealSense camera elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());

  success = LifecycleNodeManager::GetSingleton()->IsActivate(
    LifeCycleNodeType::RGBCameraSensor);
  if (!success) {
    // RGB-D camera lifecycle(configure state)
    success = LifecycleNodeManager::GetSingleton()->Configure(
      LifeCycleNodeType::RGBCameraSensor);
    if (!success) {
      return false;
    }

    // RGB-D camera lifecycle(activate state)
    success = LifecycleNodeManager::GetSingleton()->Startup(
      LifeCycleNodeType::RGBCameraSensor);
    if (!success) {
      return false;
    }
  } else {
    INFO("RGB-D Camera Sensor is activate.");
  }

  INFO("[Vision Mapping] RGB-G camera elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());
  success = localization_lifecycle_->IsActivate();
  if (!success) {
    // localization_node lifecycle(configure state)
    if (!localization_lifecycle_->Configure()) {
      return false;
    }

    // localization_node lifecycle(activate state)
    if (!localization_lifecycle_->Startup()) {
      return false;
    }
  }

  is_activate_ = true;
  INFO("[Vision Mapping] mivinslocalization elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());
  return true;
}

bool ExecutorVisionLocalization::WaitRelocalization(std::chrono::seconds timeout)
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

bool ExecutorVisionLocalization::EnableRelocalization()
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

bool ExecutorVisionLocalization::DisenableRelocalization()
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

bool ExecutorVisionLocalization::EnableReportRealtimePose(bool enable)
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

bool ExecutorVisionLocalization::CheckMapAvailable()
{
  if (map_result_client_ == nullptr) {
    map_result_client_ = std::make_shared<nav2_util::ServiceClient<MapAvailableResult>>(
      "get_miloc_status", shared_from_this());
  }

  while (!map_result_client_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("Waiting for miloc map handler the service. but cannot connect the service.");
      return false;
    }
  }

  // Set request data
  auto request = std::make_shared<MapAvailableResult::Request>();
  // request->map_id = 0;

  // Send request
  // bool success = map_result_client_->invoke(request, response);

  bool result = false;
  try {
    auto future_result = map_result_client_->invoke(request, std::chrono::seconds(10s));
    result = future_result->code == 0;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }

  return result;
}

bool ExecutorVisionLocalization::ResetLifecycleDefaultValue()
{
  bool success = LifecycleNodeManager::GetSingleton()->Pause(
    LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    ERROR("Release RealSense failed.");
  }

  LifecycleNodeManager::GetSingleton()->Pause(
    LifeCycleNodeType::RGBCameraSensor);
  if (!success) {
    ERROR("Release RGBCamera failed.");
  }

  localization_lifecycle_->Pause();
  if (!success) {
    ERROR("Release localization failed.");
  }
  return success;
}

}  // namespace algorithm
}  // namespace cyberdog
