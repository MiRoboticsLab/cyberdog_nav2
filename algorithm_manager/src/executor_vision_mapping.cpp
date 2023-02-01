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

  outdoor_client_ = create_client<std_srvs::srv::SetBool>(
    "vision_outdoor", rmw_qos_profile_services_default);

  // TF2 checker
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // spin
  std::thread{[this]() {rclcpp::spin(this->get_node_base_interface());}}.detach();
}

void ExecutorVisionMapping::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  Timer timer_;
  timer_.Start();
  (void)goal;
  INFO("Vision Mapping started");

  // TODO(dyq): This is bug
  //    when tf transform not exit from A to B frame, tf still check tf exit
  // Check current from map to base_link tf exist, if exit `Vision Localization` 
  // in activate, so that this error case
  // bool tf_exist = CanTransform("map", "base_link");
  // if (tf_exist) {
  //   ERROR("Check current from map to base_link tf exist, should never happen");
  //   UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
  //   task_abort_callback_();
  //   return;
  // }

  // Check all sensors turn on
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
    WARN("Vision mapping is stop, not need start mapping service.");
    return;
  }

  // Start build mapping
  INFO("Trying start vision mapping service(start_vins_mapping)");
  bool success = StartBuildMapping();
  if (!success) {
    ERROR("Start vision mapping service(start_vins_mapping) failed");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    ResetAllLifecyceNodes();
    ResetFlags();
    task_abort_callback_();
    return;
  }
  INFO("Start vision mapping service(start_vins_mapping) success");

  // Realtime response user stop operation
  if (CheckExit()) {
    WARN("Vision mapping is stop, not need start velocity smoother service.");
    return;
  }

  // Smoother walk
  VelocitySmoother();

  // Realtime response user stop operation
  if (CheckExit()) {
    WARN("Laser mapping is stop, not need start report realtime pose service.");
    return;
  }

  // Enable report realtime robot pose
  auto pose_thread = std::make_shared<std::thread>(
    [&]() {
      int try_count = 0;
      while (true) {
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

          if (is_slam_service_activate_) {
            CloseMappingService();
          }
          ResetAllLifecyceNodes();
          ResetFlags();
          task_abort_callback_();
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    });
  pose_thread->detach();

  // 结束激活进度的上报
  UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_SUCCESS);
  INFO("Vision Mapping success.");
  INFO("Elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());
}

void ExecutorVisionMapping::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  INFO("Vision Mapping will stop");
  response->result = StopTaskSrv::Response::SUCCESS;

  Timer timer_;
  timer_.Start();

  is_exit_ = true;
  bool success = true;

  // Disenable report realtime robot pose
  if (is_realtime_pose_service_activate_) {
    success = EnableReportRealtimePose(false);
    if (!success) {
      ERROR("Close report realtime robot pose service(PoseEnable) failed.");
      response->result = StopTaskSrv::Response::FAILED;
    } else {
      INFO("Close report realtime robot pose service(PoseEnable) success");
    }
  }
  
  // MapServer
  if (is_slam_service_activate_) {
    INFO("Trying stop vision mapping service(stop_vins_mapping)");
    success = StopBuildMapping(request->map_name);
    if (!success) {
      ERROR("Stop vision mapping service(stop_vins_mapping) failed");
      response->result = StopTaskSrv::Response::FAILED;
    } else {
      INFO("Stop vision mapping service(stop_vins_mapping) success");
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
  INFO("Vision Mapping stoped success");
  INFO("Elapsed time: %.5f [mircoseconds]", timer_.ElapsedMicroSeconds());
}

void ExecutorVisionMapping::Cancel()
{
  INFO("Vision Mapping will cancel");
}

bool ExecutorVisionMapping::IsDependsReady()
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  bool acivate_success = ActivateDepsLifecycleNodes(this->get_name());
  if (!acivate_success) {
    return false;
  }

  return true;
}

bool ExecutorVisionMapping::StartBuildMapping()
{
  if (start_client_ == nullptr) {
    start_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "start_vins_mapping", shared_from_this());
  }

  // Wait service
  bool connect = start_client_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Waiting for the service(start_vins_mapping). but cannot connect the service.");
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
    is_slam_service_activate_ = true;
    auto future_result = start_client_->invoke(request, std::chrono::seconds(5s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }

  if (result) {
    // PublishBuildMapType();
    INFO("Trying start vision mapping outdoor flag service");
    bool ok = InvokeOutdoorFlag();
    if (!ok) {
      ERROR("Start vision mapping outdoor flag service failed");
    } else {
      INFO("Start vision mapping outdoor flag service success");
    }
  }
  return result;
}

bool ExecutorVisionMapping::StopBuildMapping(const std::string & map_filename)
{
  if (stop_client_ == nullptr) {
    stop_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "stop_vins_mapping", shared_from_this());
  }

  // Wait service
  bool connect = stop_client_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Waiting for the service(stop_vins_mapping). but cannot connect the service.");
    return false;
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  // if (map_filename.empty()) {
  //   request->finish = false;
  // } else {
  //   request->finish = true;
  //   request->map_name = map_filename;
  // }

  // Send request
  // return start_->invoke(request, response);
  bool result = false;
  try {
    std::lock_guard<std::mutex> lock(service_mutex_);
    is_slam_service_activate_ = false;
    auto future_result = stop_client_->invoke(request, std::chrono::seconds(5s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }

  if (map_filename.empty()) {
    return DeleteMap();
  }

  if (result) {
    PublishBuildMapType();
  }
  return result;
}

bool ExecutorVisionMapping::EnableReportRealtimePose(bool enable)
{
  // // Wait service
  if (realtime_pose_client_ == nullptr) {
    realtime_pose_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "PoseEnable", shared_from_this());
  }

  bool connect = realtime_pose_client_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
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
    INFO("EnableReportRealtimePose(): Trying to get realtime_pose_mutex");
    std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
    is_realtime_pose_service_activate_ = enable;
    INFO("EnableReportRealtimePose(): Success to get realtime_pose_mutex");
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

  bool connect = mapping_available_client_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Waiting for miloc map handler the service. but cannot connect the service.");
    return false;
  }

  // Set request data
  auto request = std::make_shared<MapAvailableResult::Request>();
  request->map_id = 1;

  // Send request
  // bool success = mapping_available_client_->invoke(request, response);
  bool result = false;
  try {
    auto future_result = mapping_available_client_->invoke(request, std::chrono::seconds(5s));

    if (future_result->code == 0 || future_result->code == 302) {
      INFO("Relocation map is available.");
      return true;
    } else if (future_result->code == 300) {
      INFO("Relocation map not available, under construction.");
      return false;
    } else if (future_result->code == 301) {
      INFO(
        "There was an error in the last offline map building, and the map needs to be rebuilt"); // NOLINT
      return false;
    }
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }

  return result;
}

bool ExecutorVisionMapping::DeleteMap()
{
  if (map_delete_client_ == nullptr) {
    map_delete_client_ = std::make_shared<nav2_util::ServiceClient<MapAvailableResult>>(
      "delete_reloc_map", shared_from_this());
  }

  bool connect = map_delete_client_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Waiting for miloc map handler the service. but cannot connect the service.");
    return false;
  }

  // Set request data
  auto request = std::make_shared<MapAvailableResult::Request>();
  request->map_id = 1;

  // Send request
  bool result = false;
  try {
    auto future_result = map_delete_client_->invoke(request, std::chrono::seconds(10s));
    if (future_result->code == 0) {
      INFO("Delete map success");
      return true;
    } else if (future_result->code == 100) {
      ERROR("Delete map exception");
      return false;
    }
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

  bool connect = velocity_smoother_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Connect velocity adaptor service timeout.");
    return false;
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

bool ExecutorVisionMapping::InvokeOutdoorFlag()
{
  bool connect = outdoor_client_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Waiting for service(%s) timeout", outdoor_client_->get_service_name());
    return false;
  }

  // Set request data
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  // Send request
  auto future = outdoor_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5s)) == std::future_status::timeout) {
    ERROR("Send request service(%s) timeout", outdoor_client_->get_service_name());
    return false;
  }

  return future.get()->success;
}

bool ExecutorVisionMapping::ResetAllLifecyceNodes()
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  return DeactivateDepsLifecycleNodes();
}

bool ExecutorVisionMapping::CheckExit()
{
  return is_exit_;
}

bool ExecutorVisionMapping::CloseMappingService()
{
  if (stop_client_ == nullptr) {
    stop_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      "stop_vins_mapping", shared_from_this());
  }

  // Wait service
  bool connect = stop_client_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Waiting for the service(stop_vins_mapping) timeout");
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
    auto future_result = stop_client_->invoke(request, std::chrono::seconds(5s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }
  return result && DeleteMap();
}

bool ExecutorVisionMapping::CanTransform(const std::string & parent_link, const std::string & clild_link)
{
  // Look up for the transformation between parent_link and clild_link frames
  return tf_buffer_->canTransform(parent_link, clild_link, tf2::get_now(), tf2::durationFromSec(1));
}

void ExecutorVisionMapping::ResetFlags()
{
  is_exit_ = false;
}

}  // namespace algorithm
}  // namespace cyberdog
