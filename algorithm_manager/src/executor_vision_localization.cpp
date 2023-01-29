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
#include "algorithm_manager/feedcode_type.hpp"

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

  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  stop_running_server_ = this->create_service<std_srvs::srv::SetBool>(
    "reset_stop_vision_localization", std::bind(
      &ExecutorVisionLocalization::HandleStopCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
    rmw_qos_profile_default, callback_group_);

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
  UpdateFeedback(relocalization::kMapChecking);
  bool available = CheckMapAvailable();
  if (!available) {
    ERROR("Vision build map file not available.");
    UpdateFeedback(relocalization::kMapCheckingError);
    task_abort_callback_();
    return;
  }
  UpdateFeedback(relocalization::kMapCheckingSuccess);

  // 1 正在激活依赖节点
  UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_EXECUTING);
  bool ready = IsDependsReady();
  if (!ready) {
    ResetAllLifecyceNodes();
    ERROR("Vision localization lifecycle depend start up failed.");
    // 2 激活依赖节点失败
    UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    return;
  }

  // 3 激活依赖节点成功
  UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_SUCCESS);

  // Realtime response user stop operation
  if (CheckExit()) {
    WARN("Vision localization is stop, not need enable relocalization.");
    return;
  }

  // Enable Relocalization
  UpdateFeedback(relocalization::kServiceStarting);
  bool success = EnableRelocalization();
  if (!success) {
    ERROR("Turn on relocalization failed.");
    UpdateFeedback(relocalization::kServiceStartingError);
    ResetAllLifecyceNodes();
    task_abort_callback_();
    return;
  }
  UpdateFeedback(relocalization::kServiceStartingSuccess);

  // Realtime response user stop operation
  if (CheckExit()) {
    WARN("Laser localization is stop, not need wait relocalization.");
    return;
  }

  // Send request and wait relocalization result success
  success = WaitRelocalization(std::chrono::seconds(120s));
  if (!success) {
    ERROR("Vision localization failed.");
    UpdateFeedback(relocalization::kSLAMTimeout);
    ResetAllLifecyceNodes();
    task_abort_callback_();
    return;
  }

  // Check relocalization success
  if (!relocalization_success_) {
    ERROR("Vision relocalization failed.");
    UpdateFeedback(relocalization::kSLAMError);
    ResetAllLifecyceNodes();
    task_abort_callback_();
    return;
  }

  // Realtime response user stop operation
  if (CheckExit()) {
    WARN("Laser localization is stop, not need enable report realtime pose.");
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

        if (CheckExit()) {
          break;
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
          ResetAllLifecyceNodes();
          task_abort_callback_();
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    });
  pose_thread->detach();

  UpdateFeedback(relocalization::kSLAMSuccess);
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
  response->result = StopTaskSrv::Response::SUCCESS;

  Timer timer_;
  timer_.Start();

  // exit flag
  is_exit_ = true;

  // Stop current running navigation robot.
  bool stop_navigation_running = IfRobotNavigationRunningAndStop();
  if (!stop_navigation_running) {
    ERROR("Stop robot navigation running failed.");
    response->result = StopTaskSrv::Response::FAILED;
  }

  // Disenable Relocalization
  bool success = DisableRelocalization();
  if (!success) {
    ERROR("Turn off Vision relocalization failed.");
    response->result = StopTaskSrv::Response::FAILED;
  }

  // Disenable report realtime robot pose
  success = EnableReportRealtimePose(false);
  if (!success) {
    ERROR("Disenable report realtime robot pose failed.");
    response->result = StopTaskSrv::Response::FAILED;
  }

  ResetFlags();
  success = ResetAllLifecyceNodes();
  if (!success) {
    response->result = StopTaskSrv::Response::FAILED;
    task_abort_callback_();
    return;
  }
  task_success_callback_();

  INFO("Vision Localization stoped success");
  INFO("[Vision Localization] Elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());
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
  } else if (msg->data == 100) {
    UpdateFeedback(relocalization::kSLAMFailedContinueTrying);
    WARN("Relocalization retrying.");
  } else if (msg->data == 200) {
    relocalization_failure_ = true;
    UpdateFeedback(relocalization::kSLAMError);
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
  INFO("ExecutorVisionLocalization::IsDependsReady() function call mutex before");
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  INFO("ExecutorVisionLocalization::IsDependsReady() function call mutex after");
  bool acivate_success = ActivateDepsLifecycleNodes(this->get_name());
  if (!acivate_success) {
    return false;
  }

  is_activate_ = true;
  return true;
}

bool ExecutorVisionLocalization::WaitRelocalization(std::chrono::seconds timeout)
{
  auto end = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && !relocalization_success_) {
    if (is_exit_) {
      return false;
    }

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
  if (start_client_ == nullptr) {
    start_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "start_vins_location", shared_from_this());
  }

  // Wait service
  bool connect = start_client_->wait_for_service(std::chrono::seconds(5s));
  if (!connect) {
    ERROR("Waiting for the service. but cannot connect the service.");
    return false;
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  // Send request
  // return start_->invoke(request, response);
  bool result = false;
  try {
    std::lock_guard<std::mutex> lock(service_mutex_);
    auto future_result = start_client_->invoke(request, std::chrono::seconds(50s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }
  return result;
}

bool ExecutorVisionLocalization::DisableRelocalization()
{
  if (stop_client_ == nullptr) {
    stop_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "stop_vins_location", shared_from_this());
  }

  // Wait service
  bool connect = stop_client_->wait_for_service(std::chrono::seconds(5s));
  if (!connect) {
    ERROR("Waiting for the service. but cannot connect the service.");
    return false;
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  // Send request
  // return start_->invoke(request, response);
  bool result = false;
  try {
    INFO("ExecutorVisionLocalization::DisableRelocalization() function call mutex before");
    std::lock_guard<std::mutex> lock(service_mutex_);
    INFO("ExecutorVisionLocalization::DisableRelocalization() function call mutex after");
    auto future_result = stop_client_->invoke(request, std::chrono::seconds(10s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }
  return result;
}

bool ExecutorVisionLocalization::EnableReportRealtimePose(bool enable)
{
  if (realtime_pose_client_ == nullptr) {
    realtime_pose_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "PoseEnable", shared_from_this());
  }

  bool connect = realtime_pose_client_->wait_for_service(std::chrono::seconds(5s));
  if (!connect) {
    ERROR("Waiting for the service. but cannot connect the service.");
    return false;
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

  bool connect = map_result_client_->wait_for_service(std::chrono::seconds(5s));
  if (!connect) {
    ERROR("Waiting for miloc map handler the service. but cannot connect the service.");
    return false;
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

bool ExecutorVisionLocalization::ResetAllLifecyceNodes()
{
  INFO("ExecutorVisionLocalization::ResetAllLifecyceNodes() function call mutex before");
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  INFO("ExecutorVisionLocalization::ResetAllLifecyceNodes() function call mutex after");
  return DeactivateDepsLifecycleNodes();
}

bool ExecutorVisionLocalization::SendServerRequest(
  const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client,
  const std_srvs::srv::SetBool::Request::SharedPtr & request,
  std_srvs::srv::SetBool::Response::SharedPtr & response)
{
  auto future = client->async_send_request(request);
  if (future.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
    ERROR("Cannot get response from service(%s) in 2s.", client->get_service_name());
    return false;
  }

  if (future.get()->success) {
    INFO("Success to call stop service : %s.", client->get_service_name());
  } else {
    ERROR("Get error when call stop service : %s.", client->get_service_name());
  }
  return true;
}

void ExecutorVisionLocalization::ResetFlags()
{
  relocalization_success_ = false;
  relocalization_failure_ = false;
  is_activate_ = false;
  is_exit_ = false; 
}

bool ExecutorVisionLocalization::IfRobotNavigationRunningAndStop()
{
  // bool is_connect = stop_robot_nav_client_->wait_for_service(std::chrono::seconds(2));
  // if (!is_connect) {
  //   ERROR("Connect stop robot navigation server timeout.");
  //   return false;
  // }

  // auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  // auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
  // request->data = true;

  // bool success = SendServerRequest(stop_robot_nav_client_, request, response);
  // return success;

  return true;
}

void ExecutorVisionLocalization::HandleStopCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> respose)
{
  if (!request->data) {
    return;
  }

  if (!is_activate_) {
    respose->success = true;
    WARN("Vision localization not activate.");
    return;
  }

  respose->success = StopLocalizationFunctions();
}

bool ExecutorVisionLocalization::StopLocalizationFunctions()
{
  INFO("Vision localization will stop without task callback");

  Timer timer_;
  timer_.Start();

  // exit flag
  is_exit_ = true;

  // Disenable Relocalization
  bool success = DisableRelocalization();
  if (!success) {
    ERROR("Turn off Vision relocalization failed.");
  }

  // Disenable report realtime robot pose
  success = EnableReportRealtimePose(false);
  if (!success) {
    ERROR("Disenable report realtime robot pose failed.");
  }

  ResetFlags();
  success = ResetAllLifecyceNodes();
  if (!success) {
    ERROR("Reset all lifecyce nodes failed.");
  }

  INFO("Vision Localization stoped success");
  INFO("[Vision Localization] Elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());
  return success;
}

bool ExecutorVisionLocalization::CheckExit()
{
  return is_exit_;
}

}  // namespace algorithm
}  // namespace cyberdog
