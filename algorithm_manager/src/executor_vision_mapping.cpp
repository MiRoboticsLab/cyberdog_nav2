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
  outdoor_client_ = create_client<LabelParam>(
    "outdoor", rmw_qos_profile_services_default);

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
  Timer timer, total_timer;
  timer.Start();
  total_timer.Start();
  (void)goal;
  INFO("Vision Mapping started");

  // Check current from map to base_link tf exist, if exit `Vision Localization`
  // in activate, so that this error case
  bool tf_exist = CanTransform("map", "base_link");
  if (tf_exist) {
    ERROR("Check current from map to base_link tf exist, should never happen");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    task_abort_callback_();
    return;
  }
  INFO("[0] Check TF Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());
  timer.Start();
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
  INFO("[1] Activate lifecycle nodes Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());
  timer.Start();
  // Realtime response user stop operation
  if (CheckExit()) {
    WARN("Vision mapping is stop, not need start mapping service.");
    return;
  }

  // miloc manager for map delete
  if (miloc_client_ == nullptr) {
    miloc_client_ = std::make_shared<nav2_util::ServiceClient<MilocMapHandler>>(
      "delete_reloc_map", shared_from_this());
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
  INFO("[2] Start start_vins_mapping service Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());
  timer.Start();
  // Realtime response user stop operation
  if (CheckExit()) {
    WARN("Laser mapping is stop, not need start report realtime pose service.");
    return;
  }

  // Enable report realtime robot pose
  auto pose_thread = std::make_shared<std::thread>(
    [&]() {
      int try_count = 0;
      while (rclcpp::ok()) {
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
  pose_thread->join();
  INFO("[3] Enable report realtime pose Elapsed time: %.5f [seconds]", timer.ElapsedSeconds());

  // 结束激活进度的上报
  UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_SUCCESS);
  INFO("Vision Mapping success.");
  INFO("[Total] Start Vision mapping Elapsed time: %.5f [seconds]", total_timer.ElapsedSeconds());
}

void ExecutorVisionMapping::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  INFO("Vision Mapping will stop");
  response->result = StopTaskSrv::Response::SUCCESS;

  // Disenable report realtime robot pose
  bool success = EnableReportRealtimePose(false);
  if (!success) {
    ERROR("[Vision Mapping] Disenable report realtime robot pose failed.");
    response->result = StopTaskSrv::Response::FAILED;
    // UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    task_abort_callback_();
    return;
  }

  // MapServer
  success = StopBuildMapping(request->map_name);
  if (!success) {
    response->result = StopTaskSrv::Response::FAILED;
    ERROR("[Vision Mapping] Vision Mapping stop failed.");
    // UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    task_abort_callback_();
    return;
  }

  auto reset_thread = std::make_shared<std::thread>(
    [&]() {
      while (true) {
        if (!success) {
          continue;
        }

        // If check map name is empty, so that delete vision background data.
        if (request->map_name.empty()) {
          WARN("Start delete background vision map datasets.");
          bool delete_success = DeleteBackgroundVisionMapDatasets();
          if (!delete_success) {
            WARN("Delete background vision map datasets failed.");
          } else {
            INFO("Delete background vision map datasets success.");
          }
        }

        // RGB-G camera lifecycle
        success = LifecycleNodeManager::GetSingleton()->Pause(
          LifeCycleNodeType::RGBCameraSensor);
        if (!success) {
          response->result = StopTaskSrv::Response::FAILED;
          ERROR("[Vision Mapping] Vision Mapping stop failed, deactivate RGB-D sensor failed");
          UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
          task_abort_callback_();
          return;
        }

        // realsense camera lifecycle
        success = LifecycleNodeManager::GetSingleton()->Pause(
          LifeCycleNodeType::RealSenseCameraSensor);
        if (!success) {
          response->result = StopTaskSrv::Response::FAILED;
          ERROR("[Vision Mapping] Vision Mapping stop failed, deactivate realsense sensor failed.");
          // UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
          task_abort_callback_();
          return;
        }

        // mivins lifecycle
        success = mapping_client_->Pause();
        if (!success) {
          response->result = StopTaskSrv::Response::FAILED;
          ERROR("[Vision Mapping] Vision Mapping stop failed.");
          UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
          task_abort_callback_();
          return;
        }

        // UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_SUCCESS);
        response->result = StopTaskSrv::Response::SUCCESS;
        INFO("[Vision Mapping] Vision Mapping stoped success");
        break;
      }
    });
  reset_thread->detach();
  INFO("[Vision Mapping] Elapsed time: %.5f [mircoseconds]", timer_.ElapsedMicroSeconds());

  // invaild feedback code for send app
  const int32_t kInvalidFeedbackCode = -1;
  UpdateFeedback(kInvalidFeedbackCode);
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

  return result;
}

bool ExecutorVisionMapping::StopBuildMapping(const std::string & map_filename)
{
  if (stop_client_ == nullptr) {
    stop_client_ = std::make_shared<nav2_util::ServiceClient<MapRequest>>(
      "stop_vins_mapping", shared_from_this());
  }

  // Wait service
  bool connect = stop_client_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Waiting for the service(stop_vins_mapping). but cannot connect the service.");
    return false;
  }

  // Set request data
  // bool finish
  // string map_name
  // ---
  // bool success
  // string message

  auto request = std::make_shared<MapRequest::Request>();
  if (map_filename.empty()) {
    WARN("User set map name is empty");
    request->finish = false;
    request->map_name = "";
  } else {
    INFO("Saved map building filename: %s", map_filename.c_str());
    request->finish = true;
    request->map_name = map_filename;
  }

  // Send request
  // return start_->invoke(request, response);
  bool result = false;
  try {
    std::lock_guard<std::mutex> lock(service_mutex_);
    is_slam_service_activate_ = false;
    auto future_result = stop_client_->invoke(request, std::chrono::seconds(10s));
    result = future_result->success;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }

  if (result && !map_filename.empty()) {
    INFO("Trying start vision mapping outdoor flag service");
    bool ok = InvokeOutdoorFlag(map_filename);
    if (!ok) {
      ERROR("Start vision mapping outdoor flag service failed");
    } else {
      INFO("Start vision mapping outdoor flag service success");
    }
    return ok;
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

bool ExecutorVisionMapping::InvokeOutdoorFlag(const std::string & mapname)
{
  bool connect = outdoor_client_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Waiting for service(%s) timeout", outdoor_client_->get_service_name());
    return false;
  }

  // Set request data
  auto request = std::make_shared<LabelParam::Request>();
  request->label.is_outdoor = true;
  request->label.map_name = mapname;

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
    stop_client_ = std::make_shared<nav2_util::ServiceClient<MapRequest>>(
      "stop_vins_mapping", shared_from_this());
  }

  // Wait service
  bool connect = stop_client_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Waiting for the service(stop_vins_mapping) timeout");
    return false;
  }

  // Set request data
  auto request = std::make_shared<MapRequest::Request>();
  request->finish = false;
  request->map_name = "";

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
  return result;
}

bool ExecutorVisionMapping::CanTransform(
  const std::string & parent_link,
  const std::string & clild_link)
{
  // Look up for the transformation between parent_link and clild_link frames
  return tf_buffer_->canTransform(parent_link, clild_link, tf2::get_now(), tf2::durationFromSec(1));
}

void ExecutorVisionMapping::ResetFlags()
{
  is_exit_ = false;
}

bool ExecutorVisionMapping::DeleteBackgroundVisionMapDatasets()
{
  // Wait service
  while (!miloc_client_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("[Laser Mapping] Waiting for the service. but cannot connect the service.");
      return false;
    }
  }

  // Set request data
  auto request = std::make_shared<MilocMapHandler::Request>();
  auto response = std::make_shared<MilocMapHandler::Response>();
  request->map_id = 1;

  // Send request
  // return start_->invoke(request, response);
  bool result = false;
  try {
    auto future_result = miloc_client_->invoke(request, std::chrono::seconds(5s));
    constexpr int kDeleteSuccess = 0;
    constexpr int kDeleteFailure = 100;

    if (future_result->code == kDeleteSuccess) {
      INFO("Delete the relocation map successfully.");
      result = true;
    } else if (future_result->code == kDeleteFailure) {
      ERROR("Delete relocation map exception.");
    }
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }
  return result;
}

}  // namespace algorithm
}  // namespace cyberdog
