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
  // Subscription Vision relocalization result
  relocalization_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "reloc_result",
    rclcpp::SystemDefaultsQoS(),
    std::bind(
      &ExecutorVisionLocalization::HandleRelocalizationCallback, this,
      std::placeholders::_1));

  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  stop_running_server_ = this->create_service<std_srvs::srv::SetBool>(
    "reset_stop_vision_localization", std::bind(
      &ExecutorVisionLocalization::HandleStopCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
    rmw_qos_profile_default, callback_group_);

  pose_publisher_ = PosePublisher::make_shared(this);
  pose_publisher_->Stop();

  // spin
  std::thread{[this]() {
      rclcpp::spin(this->get_node_base_interface());
    }
  }.detach();
}

ExecutorVisionLocalization::~ExecutorVisionLocalization()
{
  pose_publisher_->Stop();
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
    ERROR("Vision build map file not available, you must wait a while or map file error");
    UpdateFeedback(relocalization::kMapCheckingError);
    ResetFlags();
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
    ResetFlags();
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

  INFO("Calling enable relocalization service.");
  bool success = EnableRelocalization();
  if (!success) {
    ERROR("Enable relocalization service failed.");
    UpdateFeedback(relocalization::kServiceStartingError);
    ResetAllLifecyceNodes();
    ResetFlags();
    task_abort_callback_();
    return;
  }
  INFO("Call enable relocalization service success.");

  UpdateFeedback(relocalization::kServiceStartingSuccess);

  // Realtime response user stop operation
  if (CheckExit()) {
    WARN("Vision localization is stop, not need wait relocalization.");
    return;
  }

  // Send request and wait relocalization result success
  INFO("Waiting relocalization");
  bool force_quit = false;
  success = WaitRelocalization(std::chrono::seconds(120s), force_quit);
  if (!success) {
    UpdateFeedback(relocalization::kSLAMTimeout);

    if (!force_quit) {
      INFO("Start: Trying call disable relocalization service.");
      bool ret = DisableRelocalization();
      if (!ret) {
        ERROR("Start: Trying call disable relocalization service failed");
      } else {
        INFO("Start: Trying call disable relocalization service success");
      }

      INFO("Start: Trying call reset all lifecyce nodes");
      ret = ResetAllLifecyceNodes();
      if (!ret) {
        ERROR("Start: Trying call reset all lifecyce nodes failed");
      } else {
        INFO("Start: Trying call reset all lifecyce nodes success");
      }

      ResetFlags();
      task_abort_callback_();
    }

    return;
  }
  INFO("Waiting relocalization success");

  // Realtime response user stop operation
  if (CheckExit()) {
    WARN("Vision localization is stop, not need enable report realtime pose.");
    return;
  }

  // Enable report realtime robot pose
  if (pose_publisher_->IsStop()) {
    pose_publisher_->Start();
    success = pose_publisher_->IsStart();
    if (!success) {
      ERROR("Enable report realtime robot pose failed.");
      UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
      if (is_slam_service_activate_) {
        DisableRelocalization();
      }
      ResetAllLifecyceNodes();
      ResetFlags();
      task_abort_callback_();
      return;
    } else {
      INFO("Enable report realtime robot pose success.");
    }
  }

  UpdateFeedback(relocalization::kSLAMSuccess);
  INFO("Vision localization success.");
  INFO("Elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());
  task_success_callback_();
}

void ExecutorVisionLocalization::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  (void)request;
  WARN("Vision localization Executor Stop() is called, this should never happen");
  response->result = StopTaskSrv::Response::SUCCESS;
}

void ExecutorVisionLocalization::Cancel()
{
  INFO("Vision Localization canceled");
}

void ExecutorVisionLocalization::HandleRelocalizationCallback(
  const std_msgs::msg::Int32::SharedPtr msg)
{
  if (!is_activate_) {
    return;
  }

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

bool ExecutorVisionLocalization::IsDependsReady()
{
  INFO("IsDependsReady(): Trying to get lifecycle_mutex");
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  INFO("IsDependsReady(): Success to get lifecycle_mutex");
  bool acivate_success = ActivateDepsLifecycleNodes(this->get_name());
  if (!acivate_success) {
    return false;
  }

  is_activate_ = true;
  return true;
}

bool ExecutorVisionLocalization::WaitRelocalization(std::chrono::seconds timeout, bool & force_quit)
{
  auto end = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && !relocalization_success_) {
    if (is_exit_) {
      WARN("Relocalization force quit");
      force_quit = true;
      return false;
    }

    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {
      WARN("Wait relocalization result timeout.");
      return false;
    }

    if (relocalization_failure_) {
      ERROR("Relocalization result failed.");
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
  bool connect = start_client_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Waiting for the service(start_vins_location) timeout");
    return false;
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  // Send request
  // return start_->invoke(request, response);
  bool result = false;

  try {
    INFO("EnableRelocalization(): Trying to get service mutex");
    std::lock_guard<std::mutex> lock(service_mutex_);
    is_slam_service_activate_ = true;
    INFO("EnableRelocalization(): Success to get service mutex");
    auto future_result = start_client_->invoke(request, std::chrono::seconds(50));
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
  bool connect = stop_client_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Waiting for the service(stop_vins_location) timeout");
    return false;
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  // Send request
  // return start_->invoke(request, response);
  bool result = false;

  try {
    INFO("DisableRelocalization(): Trying to get service mutex");
    std::lock_guard<std::mutex> lock(service_mutex_);
    is_slam_service_activate_ = false;
    INFO("DisableRelocalization(): Success to get service mutex");
    auto future_result = stop_client_->invoke(request, std::chrono::seconds(10));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }
  return result;
}

bool ExecutorVisionLocalization::CheckMapAvailable()
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  if (map_result_client_ == nullptr) {
    map_result_client_ = std::make_shared<nav2_util::ServiceClient<MapAvailableResult>>(
      "get_miloc_status", shared_from_this());
  }

  bool connect = map_result_client_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Waiting for miloc map handler the service(get_miloc_status) timeout");
    return false;
  }

  // Set request data
  auto request = std::make_shared<MapAvailableResult::Request>();
  // request->map_id = 0;

  // Send request
  // bool success = map_result_client_->invoke(request, response);

  bool result = false;
  try {
    auto future_result = map_result_client_->invoke(request, std::chrono::seconds(10));
    result = future_result->code == 0;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }

  return result;
}

bool ExecutorVisionLocalization::ResetAllLifecyceNodes()
{
  INFO("ResetAllLifecyceNodes():Trying to get lifecycle_mutex");
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  INFO("ResetAllLifecyceNodes():Success to get lifecycle_mutex");
  return DeactivateDepsLifecycleNodes();
}

bool ExecutorVisionLocalization::SendServerRequest(
  const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client,
  const std_srvs::srv::SetBool::Request::SharedPtr & request,
  std_srvs::srv::SetBool::Response::SharedPtr & response)
{
  (void)response;
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

void ExecutorVisionLocalization::HandleStopCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> respose)
{
  (void)request_header;
  if (!request->data) {
    return;
  }

  std::lock_guard<std::mutex> lock(task_mutex_);
  respose->success = StopLocalizationFunctions();
}

bool ExecutorVisionLocalization::StopLocalizationFunctions()
{
  INFO("Vision localization will stop");

  Timer timer_;
  timer_.Start();

  // exit flag
  is_exit_ = true;
  bool success = true;

  if (is_slam_service_activate_) {
    // Disenable Relocalization
    INFO("Stop: Trying close vision relocalization service(stop_vins_location)");
    success = DisableRelocalization();
    if (!success) {
      ERROR("Stop: Close vision relocalization service(stop_vins_location) failed.");
    } else {
      INFO("Stop: Close vision relocalization service(stop_vins_location) success");
    }
  }

  if (pose_publisher_->IsStart()) {
    // Disenable report realtime robot pose
    INFO("Stop: Trying stop report realtime pose");
    pose_publisher_->Stop();
    success = pose_publisher_->IsStop();
    if (!success) {
      ERROR("Stop: Robot stop report realtime pose failed");
    } else {
      INFO("Stop: Robot stop report realtime pose success");
    }
  }

  INFO("Stop: Trying close all lifecycle nodes");
  success = ResetAllLifecyceNodes();
  if (!success) {
    ERROR("Stop: Close all lifecyce nodes failed.");
  } else {
    INFO("Stop: Close all lifecyce nodes success.");
  }

  // Reset all flags for localization
  ResetFlags();

  INFO("Vision Localization stoped success");
  INFO("Elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());
  return success;
}

bool ExecutorVisionLocalization::CheckExit()
{
  return is_exit_;
}

}  // namespace algorithm
}  // namespace cyberdog
