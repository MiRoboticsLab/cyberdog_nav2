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

  // spin
  std::thread{[this]() {rclcpp::spin(this->get_node_base_interface());}}.detach();
}

void ExecutorVisionMapping::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  Timer timer_;
  timer_.Start();
  (void)goal;
  INFO("[Vision Mapping] Vision Mapping started");

  // If current slam mapping in background, it's not available build mapping now
  // bool available = CheckBuildMappingAvailable();
  // if (!available) {
  //   ERROR("[Vision Mapping] Vision Mapping can't start, due to miloc creating map data.");
  //   UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
  //   task_cancle_callback_();
  //   return;
  // }

  // Check all sensors turn on
  bool ready = IsDependsReady();
  if (!ready) {
    ERROR("[Vision Mapping] Vision Mapping lifecycle depend start up failed.");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    ResetLifecycleDefaultValue();
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

   // miloc manager for map delete
  if (miloc_client_ == nullptr) {
    miloc_client_ = std::make_shared<nav2_util::ServiceClient<MilocMapHandler>>(
      "delete_reloc_map", shared_from_this());
  }

  // Start build mapping
  bool success = StartBuildMapping();
  if (!success) {
    ERROR("[Vision Mapping] Start Vision Mapping failed.");
    UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_FAILURE);
    ResetLifecycleDefaultValue();
    task_abort_callback_();
    return;
  }

  // Smoother walk
  VelocitySmoother();

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
          ResetLifecycleDefaultValue();
          task_abort_callback_();
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    });
  pose_thread->detach();

  // 结束激活进度的上报
  UpdateFeedback(AlgorithmMGR::Feedback::NAVIGATION_FEEDBACK_SLAM_BUILD_MAPPING_SUCCESS);
  INFO("[Vision Mapping] Vision Mapping success.");
  INFO("[Vision Mapping] Elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());

  // invaild feedback code for send app
  const int32_t kInvalidFeedbackCode = -1;
  UpdateFeedback(kInvalidFeedbackCode);
}

void ExecutorVisionMapping::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  Timer timer_;
  timer_.Start();
  INFO("[Vision Mapping] Vision Mapping will stop");

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
  INFO("[Vision Mapping] Vision Mapping will cancel");
}

bool ExecutorVisionMapping::IsDependsReady()
{
  Timer timer_;
  timer_.Start();

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

  INFO("[Vision Mapping] RealSense camera elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());

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

  INFO("[Vision Mapping] RGB-G camera elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());

  if (!mapping_client_->IsActivate()) {
    success = mapping_client_->Configure() && mapping_client_->Startup();
    if (!success) {
      ERROR("[Vision Mapping] lifecycle manager mivinsmapping set activate state failed.");
      return false;
    }
  }

  INFO("[Vision Mapping] mivinsmapping elapsed time: %.5f [seconds]", timer_.ElapsedSeconds());
  INFO("[Vision Mapping] Start all depends lifecycle nodes success.");
  return true;
}

bool ExecutorVisionMapping::StartBuildMapping()
{
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
  (void)map_filename;
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

bool ExecutorVisionMapping::ResetLifecycleDefaultValue()
{
  bool success = LifecycleNodeManager::GetSingleton()->Pause(
    LifeCycleNodeType::RealSenseCameraSensor);
  if (!success) {
    ERROR("Release RealSense failed.");
  }

  LifecycleNodeManager::GetSingleton()->Pause(
    LifeCycleNodeType::RGBCameraSensor);
  if (!success) {
    ERROR("Release RGBCamera failed.");
  }

  mapping_client_->Pause();
  if (!success) {
    ERROR("Release mapping_client failed.");
  }
  return success;
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
