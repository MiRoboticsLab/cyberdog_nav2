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

#include "algorithm_manager/executor_laser_mapping.hpp"

namespace cyberdog
{
namespace algorithm
{

ExecutorLaserMapping::ExecutorLaserMapping(std::string node_name)
: ExecutorBase(node_name)
{
  // // Control realsense sensor startup and down
  // realsense_lifecycle_ = std::make_shared<LifecycleNodeManager>("camera/camera");
  mapping_client_ = std::make_unique<nav2_lifecycle_manager::LifecycleManagerClient>(
    "lifecycle_manager_laser_mapping");

  // Mutex mapping and localization lifecycle
  localization_client_ = std::make_unique<nav2_lifecycle_manager::LifecycleManagerClient>(
    "lifecycle_manager_localization");

  // ontrol lidar mapping turn on
  // start_client_ = create_client<std_srvs::srv::SetBool>(
  //   "start_mapping", rmw_qos_profile_services_default);

  // // Control lidar mapping turn off
  // stop_client_ = create_client<visualization::srv::Stop>(
  //   "stop_mapping", rmw_qos_profile_services_default);

  // Control lidar mapping report realtime pose turn on and turn off
  realtime_pose_client_ = create_client<std_srvs::srv::SetBool>(
    "PoseEnable", rmw_qos_profile_services_default);

  // stop_ = std::make_shared<nav2_util::ServiceClient<visualization::srv::Stop>>(
  //   "stop_mapping", shared_from_this());

  // spin
  std::thread{[this]() {rclcpp::spin(this->get_node_base_interface());}}.detach();
}

void ExecutorLaserMapping::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  (void)goal;
  INFO("Laser Mapping started");
  ReportPreparationStatus();

  // Set Laser Localization in deactivate state 
  bool ready = CheckAvailable();
  if (!ready) {
    ERROR("Laser Localization is running, Laser Mapping is not available.");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    return;
  }

  ready = IsDependsReady();
  if (!ready) {
    ERROR("Laser Mapping lifecycle depend start up failed.");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    return;
  }

  if (start_ == nullptr) {
    start_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "start_mapping", shared_from_this());
  }

  // Start build mapping
  bool success = StartBuildMapping();
  if (!success) {
    ERROR("Start laser mapping failed.");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    return;
  }

  // Enable report realtime robot pose
  success = EnableReportRealtimePose(true);
  if (!success) {
    ERROR("Enable report realtime robot pose failed.");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    return;
  }

  // 结束激活进度的上报
  ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_SUCCESS);
  INFO("Laser Mapping success.");
}

void ExecutorLaserMapping::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  INFO("Laser Mapping will stop");
  StopReportPreparationThread();

  // Disenable report realtime robot pose
  bool success = EnableReportRealtimePose(false);
  if (!success) {
    ERROR("Disenable report realtime robot pose failed.");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    return;
  }

  if (stop_ == nullptr) {
    stop_ = std::make_shared<nav2_util::ServiceClient<visualization::srv::Stop>>(
      "stop_mapping", shared_from_this());
  }
  
  // MapServer
  success = StopBuildMapping(request->map_name);
  if (!success) {
    ERROR("Laser Mapping stop failed.");
    response->result = StopTaskSrv::Response::FAILED;
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    return;
  }

  // RealSense camera lifecycle
  // if (!realsense_lifecycle_->Pause()) {
  //   response->result = StopTaskSrv::Response::FAILED;
  //   return;
  // }

  // RealSense camera lifecycle
  success = LifecycleNodeManager::GetSingleton()->Pause(LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    response->result = StopTaskSrv::Response::FAILED;
    ERROR("Laser Mapping stop failed.");
    task_abort_callback_();
    return;
  }

  // Nav lifecycle
  // response->result = OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause) ?
  //   StopTaskSrv::Response::SUCCESS :
  //   StopTaskSrv::Response::FAILED;

  response->result = mapping_client_->pause() ? 
    StopTaskSrv::Response::SUCCESS :
    StopTaskSrv::Response::FAILED;

  INFO("Laser Mapping stoped success");
  task_success_callback_();
}

void ExecutorLaserMapping::Cancel()
{
  INFO("Laser Mapping will cancel");
  StopReportPreparationThread();

  // Nav2 lifecycle
  if (!OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause)) {
    task_abort_callback_();
    return;
  }

  // RealSense camera lifecycle
  // if (!realsense_lifecycle_->Pause()) {
  //   task_abort_callback_();
  //   return;
  // }

  // RealSense camera lifecycle
  bool success = LifecycleNodeManager::GetSingleton()->Pause(
    LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    task_abort_callback_();
    return;
  }

  INFO("Laser Mapping Canceled");
}

bool ExecutorLaserMapping::IsDependsReady()
{
  // RealSense camera lifecycle(configure state)
  // if (!realsense_lifecycle_->Configure()) {
  //   ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
  //   task_abort_callback_();
  //   return false;
  // }

  // RealSense camera lifecycle(activate state)
  // if (!realsense_lifecycle_->Startup()) {
  //   ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
  //   task_abort_callback_();
  //   return false;
  // }

  // RealSense camera lifecycle(configure state)
  bool success = LifecycleNodeManager::GetSingleton()->Configure(
    LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    return false;
  }

  // RealSense camera lifecycle(activate state)
  success = LifecycleNodeManager::GetSingleton()->Startup(LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    return false;
  }

  // Nav lifecycle
  if (mapping_client_->is_active() != nav2_lifecycle_manager::SystemStatus::ACTIVE) {
    if (!mapping_client_->startup()) {
      return false;
    }
  }

  // // Nav lifecycle
  // if (!OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kStartUp)) {
  //   return false;
  // }

  INFO("Start all depends lifecycle nodes success.");
  return true;
}

bool ExecutorLaserMapping::StartBuildMapping()
{
  // // Wait service
  // while (!start_client_->wait_for_service(std::chrono::seconds(5s))) {
  //   if (!rclcpp::ok()) {
  //     ERROR("Waiting for the service. but cannot connect the service.");
  //     return false;
  //   }
  // }

  // // Set request data
  // auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  // request->data = true;

  // // Send request
  // auto future = start_client_->async_send_request(request);
  // if (future.wait_for(std::chrono::seconds(5s)) == std::future_status::timeout) {
  //   ERROR("Connect lidar mapping start service timeout");
  //   return false;
  // }
  // return future.get()->success;

  std::chrono::nanoseconds timeout(5 * 1000 * 1000 * 1000);  // 5s
  // Wait service
  while (!start_->wait_for_service(timeout)) {
    if (!rclcpp::ok()) {
      ERROR("Waiting for the service. but cannot connect the service.");
      return false;
    }
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
  request->data = true;

  // Send request
  return start_->invoke(request, response);
}

bool ExecutorLaserMapping::StopBuildMapping(const std::string & map_filename)
{
  // // Wait service
  // while (!stop_client_->wait_for_service(std::chrono::seconds(5s))) {
  //   if (!rclcpp::ok()) {
  //     ERROR("Waiting for the service. but cannot connect the service.");
  //     return false;
  //   }
  // }

  // // Set request data
  // auto request = std::make_shared<visualization::srv::Stop::Request>();
  // request->finish = true;
  // // request->map_name = map_filename;
  // request->map_name = "map";
  // INFO("Saved lidar map building filename: %s", map_filename.c_str());

  // // Send request
  // auto future = stop_client_->async_send_request(request);

  // if (future.wait_for(std::chrono::seconds(5s)) == std::future_status::timeout) {
  //   ERROR("Connect lidar mapping stop service timeout");
  //   return false;
  // }

  // return future.get()->success;

  std::chrono::nanoseconds timeout(10 * 1000 * 1000 * 1000);  // 10s
  // Wait service
  while (!stop_->wait_for_service(timeout)) {
    if (!rclcpp::ok()) {
      ERROR("Waiting for the service. but cannot connect the service.");
      return false;
    }
  }

  // Set request data
  auto request = std::make_shared<visualization::srv::Stop::Request>();
  auto response = std::make_shared<visualization::srv::Stop::Response>();
  request->finish = true;
  // request->map_name = map_filename;
  request->map_name = "map";
  INFO("Saved lidar map building filename: %s", map_filename.c_str());

  // Send request
  return stop_->invoke(request, response);
}

bool ExecutorLaserMapping::EnableReportRealtimePose(bool enable)
{
  // Wait service
  while (!realtime_pose_client_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("Waiting for the service. but cannot connect the service.");
      return false;
    }
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool_Request>();
  request->data = enable;

  // Print enable and disenable message
  if (enable) {
    INFO("Start report robot's realtime pose");
  } else {
    INFO("Stop report robot's realtime pose.");
  }

  // Send request
  auto future = realtime_pose_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5s)) == std::future_status::timeout) {
    ERROR("Connect position checker service timeout");
    return false;
  }

  start_report_realtime_pose_ = true;
  return future.get()->success;
}

bool ExecutorLaserMapping::CheckAvailable()
{
  std::chrono::nanoseconds timeout(5 * 1000 * 1000 * 1000);  // 5s
  INFO("Check ExecutorLaserMapping::CheckAvailable().");
  if (nav2_lifecycle_manager::SystemStatus::ACTIVE != localization_client_->is_active(timeout)) {
    return true;
  }

  if (!localization_client_->pause(timeout)) {
    return false;
  }

  return true;
}

}  // namespace algorithm
}  // namespace cyberdog
