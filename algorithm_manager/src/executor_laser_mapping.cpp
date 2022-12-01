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
  // mapping_client_ = std::make_unique<nav2_lifecycle_manager::LifecycleManagerClient>(
  //   "lifecycle_manager_laser_mapping");

  // Mutex mapping and localization lifecycle
  // localization_client_ = std::make_unique<nav2_lifecycle_manager::LifecycleManagerClient>(
  //   "lifecycle_manager_localization");

  // Initialize all ros parameters
  DeclareParameters();

  localization_client_ = std::make_unique<LifecycleController>("localization_node");
  mapping_client_ = std::make_unique<LifecycleController>("map_builder");

  // mapping build type
  lidar_mapping_trigger_pub_ = create_publisher<std_msgs::msg::Bool>("lidar_mapping_alive", 10);

  // ontrol lidar mapping turn on
  // start_client_ = create_client<std_srvs::srv::SetBool>(
  //   "start_mapping", rmw_qos_profile_services_default);

  // // Control lidar mapping turn off
  // stop_client_ = create_client<visualization::srv::Stop>(
  //   "stop_mapping", rmw_qos_profile_services_default);

  // Control lidar relocalization turn off
  stop_client_ = create_client<std_srvs::srv::SetBool>(
    "stop_location", rmw_qos_profile_services_default);

  // Control lidar mapping report realtime pose turn on and turn off
  realtime_pose_client_ = create_client<std_srvs::srv::SetBool>(
    "PoseEnable", rmw_qos_profile_services_default);

  // stop_ = std::make_shared<nav2_util::ServiceClient<visualization::srv::Stop>>(
  //   "stop_mapping", shared_from_this());

  // spin
  std::thread{[this]() {rclcpp::spin(this->get_node_base_interface());}}.detach();
}

ExecutorLaserMapping::~ExecutorLaserMapping()
{
  INFO("ExecutorLaserMapping shutdown() call.");
  EnableReportRealtimePose(false);
}

void ExecutorLaserMapping::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  (void)goal;
  INFO("[Laser Mapping] Laser Mapping started");
  ReportPreparationStatus();

  // Set Laser Localization in deactivate state
  bool ready = CheckAvailable();
  if (!ready) {
    ERROR("[Laser Mapping] Laser Localization is running, Laser Mapping is not available.");
    // ReportPreparationFinished(
    //   AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    task_abort_callback_();
    return;
  }

  ready = IsDependsReady();
  if (!ready) {
    ERROR("[Laser Mapping] Laser Mapping lifecycle depend start up failed.");
    // ReportPreparationFinished(
    //   AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    task_abort_callback_();
    return;
  }

  if (start_ == nullptr) {
    start_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "start_mapping", shared_from_this());
  }

  // Get all ros parameters
  GetParameters();

  // Start build mapping
  bool success = StartBuildMapping();
  if (!success) {
    ERROR("[Laser Mapping] Start laser mapping failed.");
    // ReportPreparationFinished(
    //   AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    task_abort_callback_();
    return;
  }

  if (velocity_smoother_ == nullptr) {
    velocity_smoother_ = std::make_shared<nav2_util::ServiceClient<MotionServiceCommand>>(
      "velocity_adaptor_gait", shared_from_this());
  }

  // Smoother walk
  VelocitySmoother();

  // Enable report realtime robot pose
  success = EnableReportRealtimePose(true);
  if (!success) {
    ERROR("[Laser Mapping] Enable report realtime robot pose failed.");
    // ReportPreparationFinished(
    //   AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    task_abort_callback_();
    return;
  }

  // 结束激活进度的上报
  // ReportPreparationFinished(
  //   AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_SUCCESS);
  SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_SUCCESS);
  INFO("[Laser Mapping] Laser Mapping success.");
}

void ExecutorLaserMapping::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  INFO("[Laser Mapping] Laser Mapping will stop");
  StopReportPreparationThread();

  // Disenable report realtime robot pose
  bool success = EnableReportRealtimePose(false);
  if (!success) {
    ERROR("[Laser Mapping] Disenable report realtime robot pose failed.");
    ReportPreparationFinished(
      AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
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
    ERROR("[Laser Mapping] Laser Mapping stop failed.");
    response->result = StopTaskSrv::Response::FAILED;
    // ReportPreparationFinished(
    //   AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
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
    ERROR("[Laser Mapping] Laser Mapping stop failed.");
    // ReportPreparationFinished(
    //   AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    task_abort_callback_();
    return;
  }

  // Nav lifecycle
  // response->result = OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause) ?
  //   StopTaskSrv::Response::SUCCESS :
  //   StopTaskSrv::Response::FAILED;

  response->result = mapping_client_->Pause() ?
    StopTaskSrv::Response::SUCCESS :
    StopTaskSrv::Response::FAILED;

  // ReportPreparationFinished(
  //   AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_SUCCESS);
  SetFeedbackCode(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_SUCCESS);
  task_success_callback_();
  INFO("[Laser Mapping] Laser Mapping stoped success");
}

void ExecutorLaserMapping::Cancel()
{
  INFO("[Laser Mapping] Laser Mapping will cancel");
  // StopReportPreparationThread();

  // Nav2 lifecycle
  // if (!OperateDepsNav2LifecycleNodes(this->get_name(), Nav2LifecycleMode::kPause)) {
  //   task_abort_callback_();
  //   return;
  // }

  // RealSense camera lifecycle
  // if (!realsense_lifecycle_->Pause()) {
  //   task_abort_callback_();
  //   return;
  // }

  // bool success = StopBuildMapping(request->map_name);
  // if (!success) {
  //   ERROR("[Laser Mapping] Laser Mapping stop failed.");
  //   response->result = StopTaskSrv::Response::FAILED;
  //   ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
  //   task_abort_callback_();
  //   return;
  // }

  // // RealSense camera lifecycle
  // bool success = LifecycleNodeManager::GetSingleton()->Pause(
  //   LifeCycleNodeType::RealSenseCameraSensor);
  // if (!success) {
  //   task_abort_callback_();
  //   return;
  // }
}

void ExecutorLaserMapping::DeclareParameters()
{
  INFO("DeclareParameters.");
  declare_parameter("localization_service_timeout", rclcpp::ParameterValue(5));
  declare_parameter("mapping_start_service_timeout", rclcpp::ParameterValue(5));
  declare_parameter("mapping_stop_service_timeout", rclcpp::ParameterValue(5));
  declare_parameter("pose_report_service_timeout", rclcpp::ParameterValue(5));
  declare_parameter("velocity_smoother_service_timeout", rclcpp::ParameterValue(5));
}

void ExecutorLaserMapping::GetParameters()
{
  INFO("GetParameters.");
  get_parameter("localization_service_timeout", localization_service_timeout_);
  get_parameter("mapping_start_service_timeout", mapping_start_service_timeout_);
  get_parameter("mapping_stop_service_timeout", mapping_stop_service_timeout_);
  get_parameter("pose_report_service_timeout", pose_report_service_timeout_);
  get_parameter("velocity_smoother_service_timeout", velocity_smoother_service_timeout_);
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

  // RealSense camera
  // bool success = LifecycleNodeManager::GetSingleton()->IsActivate(
  //   LifeCycleNodeType::RealSenseCameraSensor);
  // if (!success) {
  //   // RealSense camera lifecycle(configure state)
  //   success = LifecycleNodeManager::GetSingleton()->Configure(
  //     LifeCycleNodeType::RealSenseCameraSensor);
  //   if (!success) {
  //     return false;
  //   }

  //   // RealSense camera lifecycle(activate state)
  //   success = LifecycleNodeManager::GetSingleton()->Startup(
  //     LifeCycleNodeType::RealSenseCameraSensor);
  //   if (!success) {
  //     return false;
  //   }
  // }

  // // Laser mapping  lifecycle
  // if (!mapping_client_->IsActivate()) {
  //   bool ok = mapping_client_->Configure();
  //   if (!ok) {
  //     return false;
  //   }
  //   ok = mapping_client_->Startup();
  //   if (!ok) {
  //     return false;
  //   }
  // }

  if (!ActivateDepsLifecycleNodes(this->get_name())) {
    DeactivateDepsLifecycleNodes();
    task_abort_callback_();
    return false;
  }

  INFO("[Laser Mapping] Start all depends lifecycle nodes success.");
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

  // Wait service
  while (!start_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("[Laser Mapping] Waiting for the service. but cannot connect the service.");
      return false;
    }
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
  request->data = true;

  // Send request
  // return start_->invoke(request, response);
  bool result = false;
  try {
    auto future_result = start_->invoke(request, std::chrono::seconds(5s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }
  return result;
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

  // Wait service
  while (!stop_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("[Laser Mapping] Waiting for the service. but cannot connect the service.");
      return false;
    }
  }

  // Set request data
  auto request = std::make_shared<visualization::srv::Stop::Request>();
  request->finish = true;
  // request->map_name = map_filename;
  request->map_name = "map";
  INFO("Saved lidar map building filename: %s", map_filename.c_str());

  // Send request
  // return stop_->invoke(request, response);
  bool result = false;
  try {
    auto future_result = stop_->invoke(request, std::chrono::seconds(15s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }

  if (result) {
    PublishBuildMapType();
  }
  return result;
}

bool ExecutorLaserMapping::EnableReportRealtimePose(bool enable)
{
  // Wait service
  while (!realtime_pose_client_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("[Laser Mapping] Waiting for the service. but cannot connect the service.");
      return false;
    }
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool_Request>();
  request->data = enable;

  // Print enable and disenable message
  if (enable) {
    INFO("[Laser Mapping] Start report robot's realtime pose");
  } else {
    INFO("[Laser Mapping] Stop report robot's realtime pose.");
  }

  // Send request
  auto future = realtime_pose_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(10s)) == std::future_status::timeout) {
    ERROR("[Laser Mapping] Connect position checker service timeout");
    return false;
  }

  start_report_realtime_pose_ = true;
  return future.get()->success;
}

bool ExecutorLaserMapping::CheckAvailable()
{
  // if (nav2_lifecycle_manager::SystemStatus::ACTIVE != localization_client_->is_active()) {
  //   INFO("[Laser Mapping] Laser localization lifecycle is not activate state.");
  //   return true;
  // }

  INFO("Check Laser localization is activating ?");
  if (!localization_client_->IsActivate()) {
    INFO("[Laser Mapping] Laser localization lifecycle is not activate state.");
    return true;
  }

  // bool success = DisenableLocalization();
  // if (!success) {
  //   ERROR("Disenable Localization when laser mapping.");
  //   return false;
  // }

  if (!localization_client_->Pause()) {
    return false;
  }

  INFO("[Laser Mapping] Laser localization lifecycle set deactivate state success.");
  return true;
}

bool ExecutorLaserMapping::DisenableLocalization()
{
  while (!stop_client_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("Waiting for Localization stop the service. but cannot connect the service.");
      return false;
    }
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  // Send request
  auto future = stop_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5s)) == std::future_status::timeout) {
    ERROR("Connect Localization stop service timeout");
    return false;
  }

  return future.get()->success;
}

bool ExecutorLaserMapping::VelocitySmoother()
{
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
  // velocity_smoother_->invoke(request, std::chrono::seconds(5s))

  // return velocity_smoother_->invoke(request, response);
  bool result = false;
  try {
    auto future_result = velocity_smoother_->invoke(request, std::chrono::seconds(5s));
    result = future_result->result;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }
  return result;
}

void ExecutorLaserMapping::PublishBuildMapType()
{
  std_msgs::msg::Bool state;
  state.data = true;
  lidar_mapping_trigger_pub_->publish(state);
}

}  // namespace algorithm
}  // namespace cyberdog
