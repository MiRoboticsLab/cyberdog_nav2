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
#include "algorithm_manager/feedcode_type.hpp"

namespace cyberdog
{
namespace algorithm
{

ExecutorLaserLocalization::ExecutorLaserLocalization(std::string node_name)
: ExecutorBase(node_name),
  location_status_(LocationStatus::Unknown)
{
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

  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  stop_running_server_ = this->create_service<std_srvs::srv::SetBool>(
    "reset_stop_lidar_localization", std::bind(
      &ExecutorLaserLocalization::HandleStopCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
    rmw_qos_profile_default, callback_group_);

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

  Timer timer_;
  timer_.Start();

  // 1 正在激活依赖节点
  UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_EXECUTING);
  bool ready = IsDependsReady();
  if (!ready) {
    ERROR("Laser localization lifecycle depend start up failed.");
    // 2 激活依赖节点失败
    UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    ResetAllLifecyceNodes();
    task_abort_callback_();
    location_status_ = LocationStatus::FAILURE;
    return;
  }
  // 3 激活依赖节点成功
  UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_SUCCESS);

  // Realtime response user stop operation
  if (CheckExit()) {
    WARN("Laser localization is stop, not need enable relocalization.");
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
    location_status_ = LocationStatus::FAILURE;
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
    ERROR("Laser localization wait timeout, stop socalization.");
    UpdateFeedback(relocalization::kSLAMTimeout);
    DisableRelocalization();
    ResetAllLifecyceNodes();
    task_abort_callback_();
    location_status_ = LocationStatus::FAILURE;
    return;
  }

  // Realtime response user stop operation
  if (CheckExit()) {
    WARN("Laser localization is stop, not need enable report realtime pose.");
    return;
  }

  // Enable report realtime robot pose
  success = EnableReportRealtimePose(true);
  if (!success) {
    ERROR("Enable report realtime robot pose failed.");
    UpdateFeedback(relocalization::kSLAMError);
    DisableRelocalization();
    ResetAllLifecyceNodes();
    task_abort_callback_();
    location_status_ = LocationStatus::FAILURE;
    return;
  }

  // 结束激活进度的上报
  UpdateFeedback(relocalization::kSLAMSuccess);

  location_status_ = LocationStatus::SUCCESS;
  INFO("Laser localization success.");
  INFO("[Lidar Localization] Elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());
  task_success_callback_();
}

void ExecutorLaserLocalization::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  (void)request;
  INFO("Laser localization will stop");
  response->result = StopTaskSrv::Response::SUCCESS;

  Timer timer_;
  timer_.Start();

  // exit flag
  is_exit_ = true;

  // Disenable Relocalization
  bool success = DisableRelocalization();
  if (!success) {
    ERROR("Turn off Laser relocalization failed.");
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

  location_status_ = LocationStatus::Unknown;

  // Set manager status
  task_cancle_callback_();

  INFO("Laser localization stoped success");
  INFO("[Lidar Localization] Elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());
}

void ExecutorLaserLocalization::Cancel()
{
  INFO("Laser Localization canceled");
}

void ExecutorLaserLocalization::HandleLocationServiceCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void)request;
  response->success = location_status_ == LocationStatus::SUCCESS ? true : false;
}

void ExecutorLaserLocalization::HandleRelocalizationCallback(
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

bool ExecutorLaserLocalization::IsDependsReady()
{
  INFO("[Laser Loc] IsDependsReady(): Trying to get lifecycle_mutex_");
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  INFO("[Laser Loc] IsDependsReady(): Success to get lifecycle_mutex_");
  bool acivate_success = ActivateDepsLifecycleNodes(this->get_name());
  if (!acivate_success) {
    return false;
  }

  is_activate_ = true;
  return true;
}

bool ExecutorLaserLocalization::WaitRelocalization(std::chrono::seconds timeout)
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

bool ExecutorLaserLocalization::EnableRelocalization()
{
  // Control lidar relocalization turn on
  if (start_client_ == nullptr) {
    start_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "start_location", shared_from_this());
  }

  // Wait service
  bool connect = start_client_->wait_for_service(std::chrono::seconds(5s));
  if (!connect) {
    ERROR("Waiting for the service(start_location). but cannot connect the service.");
    return false;
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  // Send request
  // return start_->invoke(request, response);
  bool result = false;
  try {
    INFO("[Laser Loc] EnableRelocalization(): Trying to get service_mutex_");
    std::lock_guard<std::mutex> lock(service_mutex_);
    INFO("[Laser Loc] EnableRelocalization(): Success to get service_mutex_");
    auto future_result = start_client_->invoke(request, std::chrono::seconds(50s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }
  return result;
}

bool ExecutorLaserLocalization::DisableRelocalization()
{
  // Control lidar relocalization turn off
  if (stop_client_ == nullptr) {
    stop_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "stop_location", shared_from_this());
  }

  // Wait service
  bool connect = stop_client_->wait_for_service(std::chrono::seconds(5s));
  if (!connect) {
    ERROR("Waiting for the service(stop_location). but cannot connect the service.");
    return false;
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  // Send request
  // return start_->invoke(request, response);
  bool result = false;
  try {
    INFO("[Laser Loc] DisableRelocalization(), Trying to get service_mutex_");
    std::lock_guard<std::mutex> lock(service_mutex_);
    INFO("[Laser Loc] DisableRelocalization() Success to get service_mutex_");
    auto future_result = stop_client_->invoke(request, std::chrono::seconds(10s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }
  return result;
}

bool ExecutorLaserLocalization::EnableReportRealtimePose(bool enable)
{
  // Control lidar mapping report realtime pose turn on and turn off
  if (realtime_pose_client_ == nullptr) {
    realtime_pose_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "PoseEnable", shared_from_this());
  }

  bool is_connect = realtime_pose_client_->wait_for_service(std::chrono::seconds(2));
  if (!is_connect) {
    ERROR("Waiting for the service(PoseEnable). but cannot connect the service.");
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

bool ExecutorLaserLocalization::ResetLifecycleDefaultValue()
{
  bool success = LifecycleNodeManager::GetSingleton()->Pause(
    LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    ERROR("Release RealSense failed.");
  }

  return success;
}

void ExecutorLaserLocalization::ResetFlags()
{
  relocalization_success_ = false;
  relocalization_failure_ = false;
  is_activate_ = false;
  is_exit_ = false;
}

bool ExecutorLaserLocalization::ResetAllLifecyceNodes()
{
  INFO("[Laser Loc] ResetAllLifecyceNodes(): Trying to get lifecycle_mutex_");
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  INFO("[Laser Loc] ResetAllLifecyceNodes(): Success to get lifecycle_mutex_");
  return DeactivateDepsLifecycleNodes();
}

bool ExecutorLaserLocalization::SendServerRequest(
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

bool ExecutorLaserLocalization::IfRobotNavigationRunningAndStop()
{
  bool is_connect = stop_robot_nav_client_->wait_for_service(std::chrono::seconds(2));
  if (!is_connect) {
    ERROR("Connect stop robot navigation server timeout.");
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
  request->data = true;

  bool success = SendServerRequest(stop_robot_nav_client_, request, response);
  return success;
}

void ExecutorLaserLocalization::HandleStopCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> respose)
{
  (void)request_header;
  if (!request->data) {
    return;
  }

  if (!is_activate_) {
    respose->success = true;
    WARN("Laser localization not activate.");
    return;
  }

  respose->success = StopLocalizationFunctions();
}

bool ExecutorLaserLocalization::StopLocalizationFunctions()
{
  INFO("Laser localization will stop without task callback");

  Timer timer_;
  timer_.Start();

  // Trigger stop localization exit flag
  is_exit_ = true;

  // Disenable Relocalization
  bool success = DisableRelocalization();
  if (!success) {
    ERROR("Turn off Laser relocalization failed.");
  }

  // Disenable report realtime robot pose
  success = EnableReportRealtimePose(false);
  if (!success) {
    ERROR("Disenable report realtime robot pose failed.");
  }

  // Reset all flags for localization
  ResetFlags();
  success = ResetAllLifecyceNodes();
  if (!success) {
    ERROR("Reset all lifecyce nodes failed.");
  }

  INFO("Laser localization stoped success");
  INFO("[Lidar Localization] Elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());
  return success;
}

bool ExecutorLaserLocalization::CheckExit()
{
  return is_exit_;
}

}  // namespace algorithm
}  // namespace cyberdog
