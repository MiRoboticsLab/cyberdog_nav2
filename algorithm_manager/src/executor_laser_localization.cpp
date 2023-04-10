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

  Timer timer, total_timer;
  timer.Start();
  total_timer.Start();

  // Check current map available
  UpdateFeedback(relocalization::kMapChecking);
  bool available = CheckMapAvailable();
  if (!available) {
    ERROR("Map file not available");
    UpdateFeedback(relocalization::kMapCheckingError);
    ResetFlags();
    task_abort_callback_();
    return;
  }
  UpdateFeedback(relocalization::kMapCheckingSuccess);
  INFO("[0] Checking map available Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());
  timer.Start();
  // 1 正在激活依赖节点
  UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_EXECUTING);
  bool ready = IsDependsReady();
  if (!ready) {
    ERROR("Laser localization lifecycle depend start up failed.");
    // 2 激活依赖节点失败
    UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    ResetAllLifecyceNodes();
    ResetFlags();
    task_abort_callback_();
    location_status_ = LocationStatus::FAILURE;
    return;
  }
  // 3 激活依赖节点成功
  UpdateFeedback(AlgorithmMGR::Feedback::TASK_PREPARATION_SUCCESS);
  INFO("[1] Activate lifecycle nodes Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());
  timer.Start();
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
    ResetFlags();
    task_abort_callback_();
    location_status_ = LocationStatus::FAILURE;
    return;
  }

  UpdateFeedback(relocalization::kServiceStartingSuccess);
  INFO("[2] Enable relocalization service Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());

  // Realtime response user stop operation
  if (CheckExit()) {
    WARN("Laser localization is stop, not need wait relocalization.");
    return;
  }

  // Send request and wait relocalization result success
  bool force_quit = false;
  timer.Start();
  success = WaitRelocalization(std::chrono::seconds(120s), force_quit);
  if (!success) {
    // Close handle localizaiton reuslt
    is_activate_ = false;

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

    location_status_ = LocationStatus::FAILURE;
    return;
  }
  INFO("Waiting relocalization success");
  INFO("[3] Waiting relocalization result Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());

  // Realtime response user stop operation
  if (CheckExit()) {
    WARN("Laser localization is stop, not need enable report realtime pose.");
    return;
  }
  timer.Start();
  // Enable report realtime robot pose
  success = EnableReportRealtimePose(true);
  if (!success) {
    ERROR("Enable report realtime robot pose failed.");
    UpdateFeedback(relocalization::kSLAMError);

    if (is_slam_service_activate_) {
      DisableRelocalization();
    }

    ResetAllLifecyceNodes();
    ResetFlags();
    task_abort_callback_();
    location_status_ = LocationStatus::FAILURE;
    return;
  }
  INFO("[4] Enable report realtime pose Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());
  UpdateFeedback(relocalization::kSLAMSuccess);

  location_status_ = LocationStatus::SUCCESS;
  INFO("Laser localization success.");
  INFO(
    "[Total] Start laser localization Elapsed time: %.5f [seconds]",
    total_timer.ElapsedSeconds());
  task_success_callback_();
}

void ExecutorLaserLocalization::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  (void)request;
  WARN("Laser localization Executor Stop() is called, this should never happen");
  response->result = StopTaskSrv::Response::SUCCESS;

  // Timer timer;
  // timer.Start();

  // // exit flag
  // is_exit_ = true;

  // // Disenable Relocalization
  // bool success = DisableRelocalization();
  // if (!success) {
  //   response->result = StopTaskSrv::Response::FAILED;
  // }

  // // Disenable report realtime robot pose
  // success = EnableReportRealtimePose(false);
  // if (!success) {
  //   response->result = StopTaskSrv::Response::FAILED;
  // }

  // ResetFlags();
  // success = ResetAllLifecyceNodes();
  // if (!success) {
  //   response->result = StopTaskSrv::Response::FAILED;
  //   task_abort_callback_();
  //   return;
  // }

  // location_status_ = LocationStatus::Unknown;

  // // Set manager status
  // task_cancle_callback_();

  // INFO("Laser localization stoped success");
  // INFO("Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());
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
    if (relocalization_timeout_) {
      UpdateFeedback(relocalization::kSLAMTimeout);
      is_activate_ = false;
    } else {
      UpdateFeedback(relocalization::kSLAMFailedContinueTrying);
    }
    WARN("Relocalization retrying.");
  } else if (msg->data == 200) {
    relocalization_failure_ = true;
    UpdateFeedback(relocalization::kSLAMError);
    WARN("Relocalization failed.");
  }
}

bool ExecutorLaserLocalization::CheckMapAvailable()
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  if (map_result_client_ == nullptr) {
    map_result_client_ = std::make_shared<nav2_util::ServiceClient<MapAvailableResult>>(
      "get_label", shared_from_this());
  }

  bool connect = map_result_client_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Waiting for the service(get_label) timeout");
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
    result = future_result->success == MapAvailableResult::Response::RESULT_SUCCESS;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }

  return result;
}

bool ExecutorLaserLocalization::IsDependsReady()
{
  INFO("IsDependsReady(): Trying to get lifecycle_mutex_");
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  INFO("[IsDependsReady(): Success to get lifecycle_mutex_");
  bool acivate_success = ActivateDepsLifecycleNodes(this->get_name());
  if (!acivate_success) {
    return false;
  }

  is_activate_ = true;
  return true;
}

bool ExecutorLaserLocalization::WaitRelocalization(std::chrono::seconds timeout, bool & force_quit)
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
      relocalization_timeout_ = true;
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
  bool connect = start_client_->wait_for_service(std::chrono::seconds(2s));
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
    INFO("EnableRelocalization(): Trying to get service_mutex_");
    std::lock_guard<std::mutex> lock(service_mutex_);
    is_slam_service_activate_ = true;
    INFO("EnableRelocalization(): Success to get service_mutex_");
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
  bool connect = stop_client_->wait_for_service(std::chrono::seconds(2s));
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
    INFO("DisableRelocalization(): Trying to get service_mutex_");
    std::lock_guard<std::mutex> lock(service_mutex_);
    is_slam_service_activate_ = false;
    INFO("DisableRelocalization(): Success to get service_mutex_");
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
    INFO("Robot starting report realtime pose");
  } else {
    INFO("Robot stopping report realtime pose.");
  }

  // Send request
  // return start_->invoke(request, response);
  bool result = false;
  try {
    INFO("EnableReportRealtimePose(): Trying to get realtime_pose_mutex_");
    std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
    is_realtime_pose_service_activate_ = enable;
    INFO("EnableReportRealtimePose(): Success to get realtime_pose_mutex_");

    auto future_result = realtime_pose_client_->invoke(request, std::chrono::seconds(5s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }


  return result;
}

void ExecutorLaserLocalization::ResetFlags()
{
  relocalization_success_ = false;
  relocalization_failure_ = false;
  is_activate_ = false;
  is_exit_ = false;
  relocalization_timeout_ = false;
}

bool ExecutorLaserLocalization::ResetAllLifecyceNodes()
{
  INFO("ResetAllLifecyceNodes(): Trying to get lifecycle_mutex");
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  INFO("ResetAllLifecyceNodes(): Success to get lifecycle_mutex");
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
    WARN("Laser localization not activate, not need disable relocalization and reset lifecycles");
    return;
  }

  std::lock_guard<std::mutex> lock(task_mutex_);
  respose->success = StopLocalizationFunctions();
}

bool ExecutorLaserLocalization::StopLocalizationFunctions()
{
  INFO("Laser localization will stop");

  Timer timer, total_timer;
  timer.Start();
  total_timer.Start();

  // Trigger stop localization exit flag
  is_exit_ = true;
  bool success = true;

  if (is_slam_service_activate_) {
    // Disenable Relocalization
    INFO("Stop: Trying close laser relocalization service(stop_location)");
    success = DisableRelocalization();
    if (!success) {
      ERROR("Stop: Close laser relocalization service(stop_location) failed.");
    } else {
      INFO("Stop: Close laser relocalization service(stop_location) success");
    }
  }
  INFO("[0] Disable relocalization service Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());
  timer.Start();
  if (is_realtime_pose_service_activate_) {
    // Disenable report realtime robot pose
    INFO("Stop: Trying stop report realtime pose");
    success = EnableReportRealtimePose(false);
    if (!success) {
      ERROR("Stop: Robot stop report realtime pose failed");
    } else {
      INFO("Stop: Robot stop report realtime pose success");
    }
  }
  INFO("[1] Disable report realtime pose Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());
  timer.Start();
  INFO("Stop: Trying close all lifecycle nodes");
  success = ResetAllLifecyceNodes();
  if (!success) {
    ERROR("Stop: Close all lifecyce nodes failed.");
  } else {
    INFO("Stop: Close all lifecyce nodes success.");
  }
  INFO("[2] Deactivate lifecycle nodes Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());
  // Reset all flags for localization
  ResetFlags();

  INFO("Laser Localization stoped success");
  INFO(
    "[Total] Stop laser localization Elapsed time: %.5f [seconds]",
    total_timer.ElapsedSeconds());
  return success;
}

bool ExecutorLaserLocalization::CheckExit()
{
  return is_exit_;
}

}  // namespace algorithm
}  // namespace cyberdog
