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
  // Initialize all ros parameters
  DeclareParameters();

  outdoor_client_ = create_client<LabelParam>(
    "outdoor", rmw_qos_profile_services_default);

  // Control lidar mapping report realtime pose turn on and turn off
  pose_publisher_ = PosePublisher::make_shared(this);
  pose_publisher_->Stop();

  // TF2 checker
  // tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  // auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
  //   get_node_base_interface(), get_node_timers_interface());
  // tf_buffer_->setCreateTimerInterface(timer_interface);
  // tf_buffer_->setUsingDedicatedThread(true);
  // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // spin
  std::thread{[this]() {rclcpp::spin(this->get_node_base_interface());}}.detach();
}

ExecutorLaserMapping::~ExecutorLaserMapping()
{
  INFO("ExecutorLaserMapping shutdown() call.");
  pose_publisher_->Stop();
}

void ExecutorLaserMapping::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  (void)goal;
  INFO("Laser Mapping started");

  Timer timer_;
  timer_.Start();

  // Check current from map to base_link tf exist, if exit `Laser Localization`
  // in activate, so that this error case
  // bool tf_exist = CanTransform("map", "base_link");
  // if (tf_exist) {
  //   ERROR("Check current from map to base_link tf exist, should never happen");
  //   UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
  //   task_abort_callback_();
  //   return;
  // }

  INFO("Trying start up all lifecycle nodes");
  bool ready = IsDependsReady();
  if (!ready) {
    ERROR("Start up all lifecycle nodes failed.");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    ResetAllLifecyceNodes();
    ResetFlags();
    task_abort_callback_();
    return;
  }
  INFO("Start up all lifecycle nodes success");

  // Realtime response user stop operation
  if (CheckExit()) {
    WARN("Laser mapping is stop, not need start mapping service.");
    return;
  }

  // Start build mapping
  INFO("Trying start laser mapping service(start_mapping)");
  bool success = StartBuildMapping();
  if (!success) {
    ERROR("Start laser mapping service(start_mapping) failed");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    ResetAllLifecyceNodes();
    ResetFlags();
    task_abort_callback_();
    return;
  }
  INFO("Start laser mapping service(start_mapping) success");

  // Realtime response user stop operation
  if (CheckExit()) {
    WARN("Laser mapping is stop, not need start report realtime pose service.");
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
        CloseMappingService();
      }
      ResetAllLifecyceNodes();
      ResetFlags();
      task_abort_callback_();
      return;
    } else {
      INFO("Enable report realtime robot pose success.");
    }
  }


  UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_SUCCESS);
  INFO("Elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());
  INFO("Laser Mapping success.");
}

void ExecutorLaserMapping::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  INFO("Laser Mapping will stop");
  response->result = StopTaskSrv::Response::SUCCESS;

  Timer timer_;
  timer_.Start();

  is_exit_ = true;
  bool success = true;

  // Disenable report realtime robot pose
  if (pose_publisher_->IsStart()) {
    INFO("Trying close report realtime robot pose service(PoseEnable)");
    pose_publisher_->Stop();
    success = pose_publisher_->IsStop();
    if (!success) {
      ERROR("Close report realtime robot pose failed.");
      response->result = StopTaskSrv::Response::FAILED;
    } else {
      INFO("Close report realtime robot pose success");
    }
  }

  // MapServer
  if (is_slam_service_activate_) {
    INFO("Trying close laser mapping service(stop_mapping)");
    success = StopBuildMapping(request->map_name);
    if (!success) {
      ERROR("Close laser mapping service(stop_mapping) failed");
      response->result = StopTaskSrv::Response::FAILED;
    } else {
      INFO("Close laser mapping service(stop_mapping) success");
    }
  }

  INFO("Trying close all lifecycle nodes");
  success = ResetAllLifecyceNodes();
  if (!success) {
    ERROR("Close all lifecycle nodes failed");
    response->result = StopTaskSrv::Response::FAILED;
  } else {
    INFO("Close all lifecycle nodes success");
  }

  ResetFlags();
  task_cancle_callback_();
  INFO("Elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());
  INFO("Laser Mapping stoped success");
}

void ExecutorLaserMapping::Cancel()
{
  INFO("Laser Mapping will cancel");
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
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  bool acivate_success = ActivateDepsLifecycleNodes(this->get_name());
  if (!acivate_success) {
    return false;
  }

  return true;
}

bool ExecutorLaserMapping::StartBuildMapping()
{
  if (start_ == nullptr) {
    start_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "start_mapping", shared_from_this());
  }

  // Wait service
  bool connect = start_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Waiting for the service(start_mapping). but cannot connect the service.");
    return false;
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
  request->data = true;

  // Send request
  // return start_->invoke(request, response);
  bool result = false;
  try {
    std::lock_guard<std::mutex> lock(service_mutex_);
    is_slam_service_activate_ = true;
    auto future_result = start_->invoke(request, std::chrono::seconds(5s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }
  return result;
}

bool ExecutorLaserMapping::StopBuildMapping(const std::string & map_filename)
{
  if (stop_ == nullptr) {
    stop_ = std::make_shared<nav2_util::ServiceClient<visualization::srv::Stop>>(
      "stop_mapping", shared_from_this());
  }

  // Wait service
  bool connect = stop_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Waiting for the service(stop_mapping). but cannot connect the service.");
    return false;
  }

  // Set request data
  auto request = std::make_shared<visualization::srv::Stop::Request>();
  if (map_filename.empty()) {
    request->finish = false;
    WARN("User set map name is empty");
  } else {
    request->finish = true;
    request->map_name = map_filename;
    INFO("Saved map building filename: %s", map_filename.c_str());
  }
  
  // Send request
  // return stop_->invoke(request, response);
  bool result = false;
  try {
    std::lock_guard<std::mutex> lock(service_mutex_);
    is_slam_service_activate_ = false;
    auto future_result = stop_->invoke(request, std::chrono::seconds(15s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }

  if (result && !map_filename.empty()) {
    // PublishBuildMapType();
    INFO("Trying start lidar mapping outdoor flag service");
    bool ok = InvokeOutdoorFlag(map_filename);
    if (!ok) {
      ERROR("Start lidar mapping outdoor flag service failed");
    } else {
      INFO("Start lidar mapping outdoor flag service success");
    }
    return ok;
  }
  return result;
}

bool ExecutorLaserMapping::CheckAvailable()
{
  return true;
}

bool ExecutorLaserMapping::InvokeOutdoorFlag(const std::string & mapname)
{
  bool connect = outdoor_client_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Waiting for service(%s) timeout", outdoor_client_->get_service_name());
    return false;
  }

  // Set request data
  auto request = std::make_shared<LabelParam::Request>();
  request->label.is_outdoor = false;
  request->label.map_name = mapname;

  // Send request
  auto future = outdoor_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5s)) == std::future_status::timeout) {
    ERROR("Send request service(%s) timeout", outdoor_client_->get_service_name());
    return false;
  }

  return future.get()->success;
}

bool ExecutorLaserMapping::ResetAllLifecyceNodes()
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  return DeactivateDepsLifecycleNodes();
}

bool ExecutorLaserMapping::CheckExit()
{
  return is_exit_;
}

bool ExecutorLaserMapping::CloseMappingService()
{
  if (stop_ == nullptr) {
    stop_ = std::make_shared<nav2_util::ServiceClient<visualization::srv::Stop>>(
      "stop_mapping", shared_from_this());
  }

  // Wait service
  bool connect = stop_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Waiting for the service(stop_mapping). but cannot connect the service.");
    return false;
  }

  // Set request data
  auto request = std::make_shared<visualization::srv::Stop::Request>();
  request->finish = false;

  // Send request
  // return stop_->invoke(request, response);
  bool result = false;
  try {
    std::lock_guard<std::mutex> lock(service_mutex_);
    auto future_result = stop_->invoke(request, std::chrono::seconds(15s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }
  return result;
}

bool ExecutorLaserMapping::CanTransform(
  const std::string & parent_link,
  const std::string & clild_link)
{
  // Look up for the transformation between parent_link and clild_link frames
  // return tf_buffer_->canTransform(parent_link, clild_link, tf2::get_now(), tf2::durationFromSec(1));
  return true;
}

void ExecutorLaserMapping::ResetFlags()
{
  is_exit_ = false;
}

}  // namespace algorithm
}  // namespace cyberdog
