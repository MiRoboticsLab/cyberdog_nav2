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
#include "algorithm_manager/executor_vision_mapping.hpp"

namespace cyberdog
{
namespace algorithm
{

ExecutorVisionMapping::ExecutorVisionMapping(std::string node_name)
: ExecutorBase(node_name)
{
  // Control lidar mapping turn on
  start_client_ = create_client<std_srvs::srv::SetBool>(
    "start_vins_mapping", rmw_qos_profile_services_default);

  // Control lidar mapping turn off
  stop_client_ = create_client<std_srvs::srv::SetBool>(
    "stop_vins_mapping", rmw_qos_profile_services_default);

  // Control lidar mapping report realtime pose turn on and turn off
  realtime_pose_client_ = create_client<std_srvs::srv::SetBool>(
    "PoseEnable", rmw_qos_profile_services_default);

  // spin
  std::thread{[this]() {rclcpp::spin(this->get_node_base_interface());}}.detach();
}

void ExecutorVisionMapping::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  (void)goal;
  INFO("Vision Mapping started");
  ReportPreparationStatus();

  bool ready = IsDependsReady();
  if (!ready) {
    ERROR("Vision Mapping lifecycle depend start up failed.");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    return;
  }

  // Start build mapping
  bool success = StartBuildMapping();
  if (!success) {
    ERROR("Start Vision Mapping failed.");
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
  INFO("Vision Mapping success.");
}

void ExecutorVisionMapping::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  INFO("Vision Mapping will stop");
  StopReportPreparationThread();

  // Disenable report realtime robot pose
  bool success = EnableReportRealtimePose(false);
  if (!success) {
    ERROR("Disenable report realtime robot pose failed.");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    task_abort_callback_();
    return;
  }

  // MapServer
  success = StopBuildMapping(request->map_name);
  if (!success) {
    response->result = StopTaskSrv::Response::FAILED;
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    ERROR("Vision Mapping stop failed.");
    task_abort_callback_();
    return;
  }

  // RGB-G camera lifecycle
  success = LifecycleNodeManager::GetSingleton()->Pause(LifeCycleNodeType::RGBCameraSensor);
  if (!success) {
    response->result = StopTaskSrv::Response::FAILED;
    ERROR("Vision Mapping stop failed.");
    task_abort_callback_();
    return;
  }

  // Nav lifecycle
  response->result = OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause) ?
    StopTaskSrv::Response::SUCCESS :
    StopTaskSrv::Response::FAILED;

  INFO("Vision Mapping stoped success");
  task_success_callback_();
}

void ExecutorVisionMapping::Cancel()
{
  INFO("Vision Mapping will cancel");
  StopReportPreparationThread();

  // Nav2 lifecycle
  if (!OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause)) {
    task_abort_callback_();
    return;
  }

  // RGB-G camera lifecycle
  bool success = LifecycleNodeManager::GetSingleton()->Pause(
    LifeCycleNodeType::RGBCameraSensor);
  if (!success) {
    task_abort_callback_();
    return;
  }

  // RealSense camera lifecycle
  success = LifecycleNodeManager::GetSingleton()->Pause(
    LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    task_abort_callback_();
    return;
  }

  INFO("Vision Mapping Canceled");
}

bool ExecutorVisionMapping::IsDependsReady()
{
  // RGB-G camera lifecycle(configure state)
  bool success = LifecycleNodeManager::GetSingleton()->Configure(
    LifeCycleNodeType::RGBCameraSensor);
  if (!success) {
    return false;
  }

  // RGB-G camera lifecycle(activate state)
  success = LifecycleNodeManager::GetSingleton()->Startup(
    LifeCycleNodeType::RGBCameraSensor);
  if (!success) {
    return false;
  }

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

  // Nav lifecycle
  if (!OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kStartUp)) {
    return false;
  }
  return true;
}

bool ExecutorVisionMapping::StartBuildMapping()
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
  auto future = start_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5s)) == std::future_status::timeout) {
    ERROR("Connect Vision Mapping start service timeout");
    return false;
  }
  return future.get()->success;
}

bool ExecutorVisionMapping::StopBuildMapping(const std::string & map_filename)
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
  auto future = stop_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5s)) == std::future_status::timeout) {
    ERROR("Connect Vision mapping stop service timeout");
    return false;
  }

  return future.get()->success;
}

bool ExecutorVisionMapping::EnableReportRealtimePose(bool enable)
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
  return future.get()->success;
}

}  // namespace algorithm
}  // namespace cyberdog
