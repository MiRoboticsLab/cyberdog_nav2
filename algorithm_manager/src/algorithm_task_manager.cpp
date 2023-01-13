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
#include <cstdio>
#include <memory>
#include <vector>
#include <string>
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "algorithm_manager/executor_factory.hpp"
#include "algorithm_manager/algorithm_task_manager.hpp"

namespace cyberdog
{
namespace algorithm
{
AlgorithmTaskManager::AlgorithmTaskManager()
: machine::MachineActuator("algorithm_manager")
{
}

AlgorithmTaskManager::~AlgorithmTaskManager()
{
}

bool AlgorithmTaskManager::Init()
{
  node_ = std::make_shared<rclcpp::Node>("algorithm_manager");
  if (!BuildExecutorMap()) {
    ERROR("Init failed, cannot build executor map!");
    return false;
  }
  ros_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  ros_executor_->add_node(node_);
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  start_algo_task_server_ = rclcpp_action::create_server<AlgorithmMGR>(
    node_, "start_algo_task",
    std::bind(
      &AlgorithmTaskManager::HandleAlgorithmManagerGoal,
      this, std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &AlgorithmTaskManager::HandleAlgorithmManagerCancel,
      this, std::placeholders::_1),
    std::bind(
      &AlgorithmTaskManager::HandleAlgorithmManagerAccepted,
      this, std::placeholders::_1), rcl_action_server_get_default_options());
  stop_algo_task_server_ = node_->create_service<protocol::srv::StopAlgoTask>(
    "stop_algo_task",
    std::bind(
      &AlgorithmTaskManager::HandleStopTaskCallback, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    callback_group_);
  algo_task_status_server_ = node_->create_service<protocol::srv::AlgoTaskStatus>(
    "algo_task_status",
    std::bind(
      &AlgorithmTaskManager::HandleTaskStatusCallback, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    callback_group_);
  algo_task_status_publisher_ = node_->create_publisher<protocol::msg::AlgoTaskStatus>(
    "algo_task_status", 10);
  SetStatus(ManagerStatus::kIdle);
  std::thread{[this]() {
      while (rclcpp::ok()) {
        this->PublishStatus();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }}.detach();
  code_ptr_ = std::make_shared<system::CyberdogCode<AlgoTaskCode>>(
    cyberdog::system::ModuleCode::kNavigation);
  audio_client_ = node_->create_client<protocol::srv::AudioTextPlay>(
    "speech_text_play",
    rmw_qos_profile_services_default,
    callback_group_);
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto path = local_share_dir + std::string("/toml_config/manager/state_machine_config.toml");
  heart_beats_ptr_ = std::make_unique<cyberdog::machine::HeartBeatsActuator>("algorithm_manager");
  heart_beats_ptr_->HeartBeatRun();
  if (!this->MachineActuatorInit(path, node_)) {
    ERROR("Init failed, actuator init error.");
    return false;
  }
  this->RegisterStateCallback("SetUp", std::bind(&AlgorithmTaskManager::OnSetUp, this));
  this->RegisterStateCallback("TearDown", std::bind(&AlgorithmTaskManager::OnTearDown, this));
  this->RegisterStateCallback("SelfCheck", std::bind(&AlgorithmTaskManager::OnSelfCheck, this));
  this->RegisterStateCallback("Active", std::bind(&AlgorithmTaskManager::OnActive, this));
  this->RegisterStateCallback("DeActive", std::bind(&AlgorithmTaskManager::OnDeActive, this));
  this->RegisterStateCallback("Protected", std::bind(&AlgorithmTaskManager::OnProtected, this));
  this->RegisterStateCallback("LowPower", std::bind(&AlgorithmTaskManager::OnLowPower, this));
  this->RegisterStateCallback("OTA", std::bind(&AlgorithmTaskManager::OnOTA, this));
  this->RegisterStateCallback("Error", std::bind(&AlgorithmTaskManager::OnError, this));
  if (!this->ActuatorStart()) {
    ERROR("Init failed, actuator start error.");
    return false;
  }
  status_map_.emplace(FsmState::kUninit, "Uninit");
  status_map_.emplace(FsmState::kSetup, "Setup");
  status_map_.emplace(FsmState::kTearDown, "TearDown");
  status_map_.emplace(FsmState::kSelfCheck, "SelfCheck");
  status_map_.emplace(FsmState::kActive, "Active");
  status_map_.emplace(FsmState::kDeactive, "Deactive");
  status_map_.emplace(FsmState::kProtected, "Protected");
  status_map_.emplace(FsmState::kLowPower, "LowPower");
  status_map_.emplace(FsmState::kOTA, "OTA");
  status_map_.emplace(FsmState::kError, "Error");
  return true;
}

void AlgorithmTaskManager::Run()
{
  ros_executor_->spin();
  rclcpp::shutdown();
}

bool AlgorithmTaskManager::BuildExecutorMap()
{
  std::string task_config = ament_index_cpp::get_package_share_directory("algorithm_manager") +
    "/config/Task.toml";
  toml::value tasks;
  if (!cyberdog::common::CyberdogToml::ParseFile(task_config, tasks)) {
    FATAL("Cannot parse %s", task_config.c_str());
    return false;
  }
  if (!tasks.is_table()) {
    FATAL("Toml format error");
    return false;
  }
  toml::value values;
  cyberdog::common::CyberdogToml::Get(tasks, "task", values);
  bool result = true;
  for (size_t i = 0; i < values.size(); i++) {
    auto value = values.at(i);
    std::string task_name;
    TaskRef task_ref;
    GET_TOML_VALUE(value, "TaskName", task_name);
    GET_TOML_VALUE(value, "Id", task_ref.id);
    GET_TOML_VALUE(value, "OutDoor", task_ref.out_door);
    auto executor_ptr = CreateExecutor(task_ref.id, task_ref.out_door, task_name);
    if (executor_ptr == nullptr) {
      ERROR("BuildExecutorMap failed, cannot create executor: %s!", task_name.c_str());
      result = false;
      break;
    }
    if (!executor_ptr->Init(
        std::bind(&AlgorithmTaskManager::TaskFeedBack, this, std::placeholders::_1),
        std::bind(&AlgorithmTaskManager::TaskSuccessd, this),
        std::bind(&AlgorithmTaskManager::TaskCanceled, this),
        std::bind(&AlgorithmTaskManager::TaskAborted, this)))
    {
      ERROR("BuildExecutorMap failed, cannot init executor: %s!", task_name.c_str());
      result = false;
      break;
    }
    task_ref.executor = executor_ptr;
    std::vector<std::string> temp;
    GET_TOML_VALUE(value, "DepsNav2LifecycleNodes", temp);
    for (auto s : temp) {
      task_ref.nav2_lifecycle_clients.emplace(s, nullptr);
    }
    GET_TOML_VALUE(value, "DepsLifecycleNodes", temp);
    for (auto s : temp) {
      task_ref.lifecycle_nodes.emplace(s, LifecycleClients{nullptr, nullptr});
    }
    task_map_.emplace(task_name, task_ref);
  }
  if ((!result && (!task_map_.empty()))) {
    task_map_.clear();
  }
  return result;
}

void AlgorithmTaskManager::HandleTaskStatusCallback(
  const protocol::srv::AlgoTaskStatus::Request::SharedPtr,
  protocol::srv::AlgoTaskStatus::Response::SharedPtr response)
{
  response->status = static_cast<uint8_t>(GetStatus());
}

void AlgorithmTaskManager::HandleStopTaskCallback(
  const protocol::srv::StopAlgoTask::Request::SharedPtr request,
  protocol::srv::StopAlgoTask::Response::SharedPtr response)
{
  INFO("=====================");
  auto status = GetStatus();
  bool reset_all = false;
  if (request->task_id == 0) {
    reset_all = true;
    if (status != ManagerStatus::kExecutingLaserAbNavigation &&
      status != ManagerStatus::kExecutingVisAbNavigation &&
      status != ManagerStatus::kExecutingLaserLocalization &&
      status != ManagerStatus::kExecutingVisLocalization &&
      status != ManagerStatus::kLaserLocalizing &&
      status != ManagerStatus::kVisLocalizing)
    {
      ERROR("Cannot Reset Nav when %d", (int)status);
      response->result = protocol::srv::StopAlgoTask::Response::FAILED;
      return;
    }

    std::string task_name;
    for (auto task : task_map_) {
      if (task.second.id == request->task_id) {
        task_name = task.first;
      }
    }
    auto iter = task_map_.find(task_name);
    if (iter == task_map_.end()) {
      ERROR("Error when get ResetNav executor");
      response->result = protocol::srv::StopAlgoTask::Response::FAILED;
      return;
    } else {
      SetTaskExecutor(iter->second.executor);
    }
    INFO("Will Reset Nav");
  } else {
    auto status_reparsed = ReParseStatus(status);
    if (status_reparsed != request->task_id) {
      ERROR(
        "Cannot execute to stop %d when %d(raw: %d)", request->task_id, (int)status_reparsed,
        (int)status);
      return;
    }
  }
  SetStatus(ManagerStatus::kStoppingTask);
  if (activated_executor_ != nullptr) {
    activated_executor_->Stop(request, response);
  }
  if (!reset_all) {
    if (status == ManagerStatus::kExecutingLaserAbNavigation) {
      SetStatus(ManagerStatus::kLaserLocalizing);
      return;
    } else if (status == ManagerStatus::kExecutingVisAbNavigation) {
      SetStatus(ManagerStatus::kVisLocalizing);
      return;
    }
  }
  ResetManagerStatus();
}

rclcpp_action::GoalResponse AlgorithmTaskManager::HandleAlgorithmManagerGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const AlgorithmMGR::Goal> goal)
{
  (void)uuid;
  INFO("---------------------");
  int32_t code = 0;
  if (!IsStateValid(code)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  INFO("goal->outdoor : %d", goal->outdoor);

  if (!CheckStatusValid(goal)) {
    ERROR(
      "Cannot accept task: %d, status is invalid!", goal->nav_type);
    return rclcpp_action::GoalResponse::REJECT;
  }
  std::string task_name;
  for (auto task : task_map_) {
    if (goal->nav_type == AlgorithmMGR::Goal::NAVIGATION_TYPE_START_AB) {
      if (task.second.id == goal->nav_type) {
        task_name = task.first;
        break;
      } 
    } else {
      if (task.second.id == goal->nav_type && task.second.out_door == goal->outdoor) {
        task_name = task.first;
        break;
      }
    }
  }
  auto iter = task_map_.find(task_name);
  if (iter == task_map_.end()) {
    ERROR(
      "Cannot accept task: %d, nav type is invalid!", goal->nav_type);
    return rclcpp_action::GoalResponse::REJECT;
  } else {
    std::string outdoor = iter->second.out_door ? "true" : "false";
    INFO("Run current task: %s, outdoor : %s", task_name.c_str(), outdoor.c_str());
    SetTaskExecutor(iter->second.executor);
  }
  // SetStatus(static_cast<ManagerStatus>(goal->nav_type));
  SetStatus(goal);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AlgorithmTaskManager::HandleAlgorithmManagerCancel(
  const std::shared_ptr<GoalHandleAlgorithmMGR> goal_handle)
{
  INFO("---------------------");
  if (CheckStatusValid()) {
    INFO("No task to cancel");
    return rclcpp_action::CancelResponse::REJECT;
  }
  INFO("Received request to cancel task");
  (void)goal_handle;
  activated_executor_->Cancel();
  auto result = std::make_shared<AlgorithmMGR::Result>();
  activated_executor_.reset();
  ResetManagerStatus();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AlgorithmTaskManager::HandleAlgorithmManagerAccepted(
  const std::shared_ptr<GoalHandleAlgorithmMGR> goal_handle)
{
  SetTaskHandle(goal_handle);
  std::thread{[this, goal_handle]() {
      activated_executor_->Start(goal_handle->get_goal());
    }}.detach();
}

void AlgorithmTaskManager::TaskFeedBack(const AlgorithmMGR::Feedback::SharedPtr feedback)
{
  global_feedback_ = feedback->feedback_code;
  if (goal_handle_executing_ != nullptr &&
    GetStatus() != ManagerStatus::kStoppingTask)
  {
    goal_handle_executing_->publish_feedback(feedback);
  }
}

void AlgorithmTaskManager::TaskSuccessd()
{
  INFO("Got Executor Success");
  auto result = std::make_shared<AlgorithmMGR::Result>();
  result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_SUCCESS;
  if (goal_handle_executing_) {
    goal_handle_executing_->succeed(result);
  }
  INFO("Manager success");
  ResetTaskHandle();
  INFO("Manager TaskHandle reset bc success");
  ResetManagerSubStatus();
  auto status = GetStatus();
  if (status == ManagerStatus::kExecutingLaserLocalization ||
    status == ManagerStatus::kExecutingLaserAbNavigation)
  {
    SetStatus(ManagerStatus::kLaserLocalizing);
    return;
  }
  if (status == ManagerStatus::kExecutingVisLocalization ||
    status == ManagerStatus::kExecutingVisAbNavigation)
  {
    SetStatus(ManagerStatus::kVisLocalizing);
    return;
  }
  ResetManagerStatus();
}

void AlgorithmTaskManager::TaskCanceled()
{
  INFO("Got Executor canceled");
  auto result = std::make_shared<AlgorithmMGR::Result>();
  result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_CANCEL;
  if (goal_handle_executing_ != nullptr) {
    goal_handle_executing_->abort(result);
    INFO("Manager canceled");
    ResetTaskHandle();
    INFO("Manager TaskHandle reset bc canceled");
    ResetManagerSubStatus();
    auto status = GetStatus();
    if (status == ManagerStatus::kExecutingLaserAbNavigation) {
      SetStatus(ManagerStatus::kLaserLocalizing);
      return;
    } else if (status == ManagerStatus::kExecutingVisAbNavigation) {
      SetStatus(ManagerStatus::kVisLocalizing);
      return;
    }
    ResetManagerStatus();
  } else {
    ERROR("GoalHandle is null when server executing cancel, this should never happen");
  }
}
void AlgorithmTaskManager::TaskAborted()
{
  INFO("Got Executor abort");
  auto result = std::make_shared<AlgorithmMGR::Result>();
  result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_FAILED;
  goal_handle_executing_->abort(result);
  INFO("Manager abort");
  ResetTaskHandle();
  INFO("Manager TaskHandle reset bc aborted");
  ResetManagerSubStatus();
  auto status = GetStatus();
  if (status == ManagerStatus::kExecutingLaserAbNavigation) {
    SetStatus(ManagerStatus::kLaserLocalizing);
    return;
  } else if (status == ManagerStatus::kExecutingVisAbNavigation) {
    SetStatus(ManagerStatus::kVisLocalizing);
    return;
  } else if (status == ManagerStatus::kExecutingLaserLocalization) {
    SetStatus(ManagerStatus::kLaserLocalizationFailed);
    return;
  } else if (status == ManagerStatus::kExecutingVisLocalization) {
    SetStatus(ManagerStatus::kVisLocalizationFailed);
    return;
  }
  ResetManagerStatus();
}

std::string ToString(const ManagerStatus & status)
{
  switch (status) {
    case ManagerStatus::kUninitialized:
      return "kUninitialized:100";

    case ManagerStatus::kIdle:
      return "kIdle:101";

    case ManagerStatus::kLaunchingLifecycleNode:
      return "kLaunchingLifecycleNode:102";

    case ManagerStatus::kStoppingTask:
      return "kStoppingTask:103";

    case ManagerStatus::kExecutingLaserMapping:
      return "kExecutingLaserMapping:5";

    case ManagerStatus::kExecutingVisMapping:
      return "kExecutingVisMapping:15";

    case ManagerStatus::kExecutingLaserLocalization:
      return "kExecutingLaserLocalization:7";

    case ManagerStatus::kExecutingVisLocalization:
      return "kExecutingVisLocalization:17";

    case ManagerStatus::kExecutingLaserAbNavigation:
      return "kExecutingLaserAbNavigation:1";

    case ManagerStatus::kExecutingAutoDock:
      return "kExecutingAutoDock:9";

    case ManagerStatus::kExecutingUwbTracking:
      return "kExecutingUwbTracking:11";

    case ManagerStatus::kExecutingHumanTracking:
      return "kExecutingHumanTracking:13";

    case ManagerStatus::kExecutingFollowing:
      return "kExecutingFollowing:3";

    default:
      break;
  }
  return "Unknown";
}

}  // namespace algorithm
}  // namespace cyberdog
