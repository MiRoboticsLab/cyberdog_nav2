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

  // mapping build type
  lidar_mapping_trigger_pub_ = create_publisher<std_msgs::msg::Bool>("lidar_mapping_alive", 10);
  robot_pose_pub_ = create_publisher<std_msgs::msg::Bool>("pose_enable", 10);

  outdoor_client_ = create_client<std_srvs::srv::SetBool>(
    "lidar_outdoor", rmw_qos_profile_services_default);

  // Control lidar mapping report realtime pose turn on and turn off
  realtime_pose_client_ = create_client<std_srvs::srv::SetBool>(
    "PoseEnable", rmw_qos_profile_services_default);

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

ExecutorLaserMapping::~ExecutorLaserMapping()
{
  INFO("ExecutorLaserMapping shutdown() call.");
  EnableReportRealtimePose(false);
}

void ExecutorLaserMapping::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  (void)goal;
  INFO("Laser Mapping started");

  Timer timer_;
  timer_.Start();

  // TODO(dyq): This is bug
  //    when tf transform not exit from A to B frame, tf still check tf exit
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
    WARN("Laser mapping is stop, not need start velocity smoother service.");
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
  success = EnableReportRealtimePose(true);
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
  if (is_realtime_pose_service_activate_) {
    INFO("Trying close report realtime robot pose service(PoseEnable)");
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
  } else {
    request->finish = true;
    request->map_name = map_filename;
  }
  INFO("Saved lidar map building filename: %s", map_filename.c_str());

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

  if (result) {
    PublishBuildMapType();
  }
  return result;
}

bool ExecutorLaserMapping::EnableReportRealtimePose(bool enable, bool use_topic)
{
  if (!use_topic) {
    // Wait service
    bool connect = realtime_pose_client_->wait_for_service(std::chrono::seconds(2s));
    if (!connect) {
      ERROR("Waiting for the service(PoseEnable). but cannot connect the service.");
      return false;
    }

    // Set request data
    auto request = std::make_shared<std_srvs::srv::SetBool_Request>();
    request->data = enable;

    // Print enable and disenable message
    if (enable) {
      INFO("Robot starting report realtime pose");
    } else {
      INFO("Robot stopping report realtime pose.");
    }

    // Send request
    INFO("EnableReportRealtimePose(): Trying to get realtime_pose_mutex");
    std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
    is_realtime_pose_service_activate_ = enable;
    INFO("EnableReportRealtimePose(): Success to get realtime_pose_mutex");

    auto future = realtime_pose_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(10s)) == std::future_status::timeout) {
      ERROR("Connect position checker service timeout");
      return false;
    }

    start_report_realtime_pose_ = true;
    return future.get()->success;
  } else {
    std_msgs::msg::Bool enable_command;
    enable_command.data = enable;
    robot_pose_pub_->publish(enable_command);
  }
  return true;
}

bool ExecutorLaserMapping::CheckAvailable()
{
  return true;
}

bool ExecutorLaserMapping::VelocitySmoother()
{
  if (velocity_smoother_ == nullptr) {
    velocity_smoother_ = std::make_shared<nav2_util::ServiceClient<MotionServiceCommand>>(
      "velocity_adaptor_gait", shared_from_this());
  }

  bool connect = velocity_smoother_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Connect velocity adaptor service timeout");
    return false;
  }

  // Set request data
  auto request = std::make_shared<MotionServiceCommand::Request>();
  std::vector<float> step_height{0.01, 0.01};
  request->motion_id = 303;
  request->value = 2;
  request->step_height = step_height;

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

bool ExecutorLaserMapping::DeleteBackgroundVisionMapDatasets()
{
  // Wait service
  bool connect = miloc_client_->wait_for_service(std::chrono::seconds(2s));
  if (!connect) {
    ERROR("Waiting for the service. but cannot connect the service.");
    return false;
  }

  // Set request data
  auto request = std::make_shared<MilocMapHandler::Request>();
  auto response = std::make_shared<MilocMapHandler::Response>();
  request->map_id = 0;

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

bool ExecutorLaserMapping::InvokeOutdoorFlag()
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

bool ExecutorLaserMapping::CanTransform(const std::string & parent_link, const std::string & clild_link)
{
  // Look up for the transformation between parent_link and clild_link frames
  return tf_buffer_->canTransform(parent_link, clild_link, tf2::get_now(), tf2::durationFromSec(1));
}

void ExecutorLaserMapping::ResetFlags()
{
  is_exit_ = false;
}

}  // namespace algorithm
}  // namespace cyberdog


