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
  // Mapping build type
  vision_mapping_trigger_pub_ = create_publisher<std_msgs::msg::Bool>("vision_mapping_alive", 10);

  // Control `mivinsmapping` lifecycle turn on and turn off
  mapping_client_ = std::make_shared<LifecycleController>("mivinsmapping");

  // // Control lidar mapping turn on
  // start_client_ = create_client<std_srvs::srv::SetBool>(
  //   "start_vins_mapping", rmw_qos_profile_services_default);

  // // Control lidar mapping turn off
  // stop_client_ = create_client<std_srvs::srv::SetBool>(
  //   "stop_vins_mapping", rmw_qos_profile_services_default);

  // // Control lidar mapping report realtime pose turn on and turn off
  // realtime_pose_client_ = create_client<std_srvs::srv::SetBool>(
  //   "PoseEnable", rmw_qos_profile_services_default);

  // spin
  std::thread{[this]() {rclcpp::spin(this->get_node_base_interface());}}.detach();
}

void ExecutorVisionMapping::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  (void)goal;
  INFO("[Vision Mapping] Vision Mapping started");
  ReportPreparationStatus();

  // If current slam mapping in background, it's not available build mapping now
  bool available = CheckBuildMappingAvailable();
  if (!available) {
    ERROR("[Vision Mapping] Vision Mapping can't start, due to miloc creating map data.");
    SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    task_cancle_callback_();
    return;
  }

  // Check all sensors turn on
  bool ready = IsDependsReady();
  if (!ready) {
    ERROR("[Vision Mapping] Vision Mapping lifecycle depend start up failed.");
    ReportPreparationFinished(
      AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    task_abort_callback_();
    return;
  }

  if (start_client_ == nullptr) {
    start_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "start_vins_mapping", shared_from_this());
  }

  if (stop_client_ == nullptr) {
    stop_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "stop_vins_mapping", shared_from_this());
  }

  // Start build mapping
  bool success = StartBuildMapping();
  if (!success) {
    ERROR("[Vision Mapping] Start Vision Mapping failed.");
    ReportPreparationFinished(
      AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    task_abort_callback_();
    return;
  }

  // Smoother walk
  VelocitySmoother();

  // Enable report realtime robot pose
  success = EnableReportRealtimePose(true);
  if (!success) {
    ERROR("[Vision Mapping] Enable report realtime robot pose failed.");
    ReportPreparationFinished(
      AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    task_abort_callback_();
    return;
  }

  // 结束激活进度的上报
  ReportPreparationFinished(
    AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_SUCCESS);
  INFO("[Vision Mapping] Vision Mapping success.");
}

void ExecutorVisionMapping::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  INFO("[Vision Mapping] Vision Mapping will stop");

  // Disenable report realtime robot pose
  bool success = EnableReportRealtimePose(false);
  if (!success) {
    ERROR("[Vision Mapping] Disenable report realtime robot pose failed.");
    // ReportPreparationFinished(
    //   AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    task_abort_callback_();
    return;
  }

  // MapServer
  success = StopBuildMapping(request->map_name);
  if (!success) {
    response->result = StopTaskSrv::Response::FAILED;
    // ReportPreparationFinished(
    //   AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    ERROR("[Vision Mapping] Vision Mapping stop failed.");
    SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    task_abort_callback_();
    return;
  }

  // RGB-G camera lifecycle
  success = LifecycleNodeManager::GetSingleton()->Pause(
    LifeCycleNodeType::RGBCameraSensor);
  if (!success) {
    response->result = StopTaskSrv::Response::FAILED;
    ERROR("[Vision Mapping] Vision Mapping stop failed, deactivate RGB-D sensor failed");
    // ReportPreparationFinished(
    //   AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    task_abort_callback_();
    return;
  }

  // realsense camera lifecycle
  success = LifecycleNodeManager::GetSingleton()->Pause(
    LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    response->result = StopTaskSrv::Response::FAILED;
    ERROR("[Vision Mapping] Vision Mapping stop failed, deactivate realsense sensor failed.");
    // ReportPreparationFinished(
    //   AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    task_abort_callback_();
    return;
  }

  INFO("-----> 0 <-------");
  // mivins lifecycle
  success = mapping_client_->Pause();
  if (!success) {
    response->result = StopTaskSrv::Response::FAILED;
    ERROR("[Vision Mapping] Vision Mapping stop failed.");
    // ReportPreparationFinished(
    //   AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    task_abort_callback_();
    return;
  }

  // INFO("-----> 1 <-------");
  // StopReportPreparationThread();
  // INFO("-----> 2 <-------");
  // ReportPreparationFinished(
  //   AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_SUCCESS);
  // INFO("-----> 3 <-------");

  INFO("-----> 1 <-------");
  SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_SUCCESS);
  INFO("-----> 2 <-------");
  task_cancle_callback_();
  INFO("-----> 3 <-------");
  INFO("[Vision Mapping] Vision Mapping stoped success");
}

void ExecutorVisionMapping::Cancel()
{
  INFO("[Vision Mapping] Vision Mapping will cancel");
}

bool ExecutorVisionMapping::IsDependsReady()
{
  // RealSense camera
  bool success = LifecycleNodeManager::GetSingleton()->IsActivate(
    LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    // RealSense camera lifecycle(configure state)
    success = LifecycleNodeManager::GetSingleton()->Configure(
      LifeCycleNodeType::RealSenseCameraSensor);
    if (!success) {
      ERROR("[Vision Mapping] RealSense camera set configure state failed.");
      return false;
    }

    // RealSense camera lifecycle(activate state)
    success = LifecycleNodeManager::GetSingleton()->Startup(
      LifeCycleNodeType::RealSenseCameraSensor);
    if (!success) {
      ERROR("[Vision Mapping] RealSense camera set activate state failed.");
      return false;
    }
  }

  // RGB-G camera
  success = LifecycleNodeManager::GetSingleton()->IsActivate(
    LifeCycleNodeType::RGBCameraSensor);
  if (!success) {
    // RGB-G camera lifecycle(configure state)
    success = LifecycleNodeManager::GetSingleton()->Configure(
      LifeCycleNodeType::RGBCameraSensor);
    if (!success) {
      ERROR("[Vision Mapping] RGB-G camera set configure state failed.");
      return false;
    }

    // RGB-G camera lifecycle(activate state)
    success = LifecycleNodeManager::GetSingleton()->Startup(
      LifeCycleNodeType::RGBCameraSensor);
    if (!success) {
      ERROR("[Vision Mapping] RGB-G camera set activate state failed.");
      return false;
    }
  }

  // // Nav lifecycle
  // if (!OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kStartUp)) {
  //   ERROR("[Vision Mapping] lifecycle manager vis_mapping set activate state failed.");
  //   return false;
  // }

  if (!mapping_client_->IsActivate()) {
    success = mapping_client_->Configure() && mapping_client_->Startup();
    if (!success) {
      ERROR("[Vision Mapping] lifecycle manager mivinsmapping set activate state failed.");
      return false;
    }
  }

  INFO("[Vision Mapping] Start all depends lifecycle nodes success.");
  return true;
}

bool ExecutorVisionMapping::StartBuildMapping()
{
  // // Wait service
  // while (!start_client_->wait_for_service(std::chrono::seconds(5s))) {
  //   if (!rclcpp::ok()) {
  //     ERROR("[Vision Mapping] Waiting for the service. but cannot connect the service.");
  //     return false;
  //   }
  // }

  // // Set request data
  // auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  // request->data = true;

  // // Send request
  // auto future = start_client_->async_send_request(request);
  // if (future.wait_for(std::chrono::seconds(5s)) == std::future_status::timeout) {
  //   ERROR("[Vision Mapping] Connect Vision Mapping start service timeout");
  //   return false;
  // }
  // return future.get()->success;

  // Wait service
  while (!start_client_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("[Vision Mapping] Waiting for the service. but cannot connect the service.");
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
    auto future_result = start_client_->invoke(request, std::chrono::seconds(5s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }

  if (result) {
    PublishBuildMapType();
  }
  return result;
}

bool ExecutorVisionMapping::StopBuildMapping(const std::string & map_filename)
{
  // // Wait service
  // while (!stop_client_->wait_for_service(std::chrono::seconds(5s))) {
  //   if (!rclcpp::ok()) {
  //     ERROR("[Vision Mapping] Waiting for the service. but cannot connect the service.");
  //     return false;
  //   }
  // }

  // // Set request data
  // auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  // request->data = true;

  // // Send request
  // auto future = stop_client_->async_send_request(request);
  // if (future.wait_for(std::chrono::seconds(5s)) == std::future_status::timeout) {
  //   ERROR("[Vision Mapping] Connect Vision mapping stop service timeout");
  //   return false;
  // }

  // Wait service
  while (!stop_client_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("[Vision Mapping] Waiting for the service. but cannot connect the service.");
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
    auto future_result = stop_client_->invoke(request, std::chrono::seconds(5s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }

  if (result) {
    PublishBuildMapType();
  }
  return result;
}

bool ExecutorVisionMapping::EnableReportRealtimePose(bool enable)
{
  // // Wait service
  // while (!realtime_pose_client_->wait_for_service(std::chrono::seconds(5s))) {
  //   if (!rclcpp::ok()) {
  //     ERROR("[Vision Mapping] Waiting for the service. but cannot connect the service.");
  //     return false;
  //   }
  // }

  // // Set request data
  // auto request = std::make_shared<std_srvs::srv::SetBool_Request>();
  // request->data = enable;

  // // Print enable and disenable message
  // if (enable) {
  //   INFO("[Vision Mapping] Start report robot's realtime pose");
  // } else {
  //   INFO("[Vision Mapping] Stop report robot's realtime pose.");
  // }

  // // Send request
  // auto future = realtime_pose_client_->async_send_request(request);
  // if (future.wait_for(std::chrono::seconds(5s)) == std::future_status::timeout) {
  //   ERROR("[Vision Mapping] Connect position checker service timeout");
  //   return false;
  // }
  // return future.get()->success;

  if (realtime_pose_client_ == nullptr) {
    realtime_pose_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "PoseEnable", shared_from_this());
  }

  while (!realtime_pose_client_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("[Vision Mapping] Waiting for the service. but cannot connect the service.");
      return false;
    }
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enable;

  // Print enable and disenable message
  if (enable) {
    INFO("[Vision Mapping] Start report robot's realtime pose");
  } else {
    INFO("[Vision Mapping] Stop report robot's realtime pose.");
  }

  // Send request
  // return start_->invoke(request, response);
  bool result = false;
  try {
    auto future_result = realtime_pose_client_->invoke(request, std::chrono::seconds(10s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }
  return result;
}

bool ExecutorVisionMapping::CheckBuildMappingAvailable()
{
  if (mapping_available_client_ == nullptr) {
    mapping_available_client_ = std::make_shared<nav2_util::ServiceClient<MapAvailableResult>>(
      "get_miloc_status", shared_from_this());
  }

  while (!mapping_available_client_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("Waiting for miloc map handler the service. but cannot connect the service.");
      return false;
    }
  }

  // Set request data
  auto request = std::make_shared<MapAvailableResult::Request>();
  request->map_id = 1;

  // Send request
  // bool success = mapping_available_client_->invoke(request, response);
  bool result = false;
  try {
    auto future_result = mapping_available_client_->invoke(request, std::chrono::seconds(5s));
    result = future_result->code != 300;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }

  return result;
}

bool ExecutorVisionMapping::VelocitySmoother()
{
  if (velocity_smoother_ == nullptr) {
    velocity_smoother_ = std::make_shared<nav2_util::ServiceClient<MotionServiceCommand>>(
      "velocity_adaptor_gait", shared_from_this());
  }

  while (!velocity_smoother_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("[Laser Mapping] Connect velocity adaptor service timeout");
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
  return result;
}

void ExecutorVisionMapping::PublishBuildMapType()
{
  std_msgs::msg::Bool state;
  state.data = true;
  vision_mapping_trigger_pub_->publish(state);
}

}  // namespace algorithm
}  // namespace cyberdog
