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
: rclcpp::Node("AlgorithmTaskManager")
{
}

AlgorithmTaskManager::~AlgorithmTaskManager()
{
}

bool AlgorithmTaskManager::Init()
{
  if (!BuildExecutorMap()) {
    ERROR("Init failed, cannot build executor map!");
    return false;
  }
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  start_algo_task_server_ = rclcpp_action::create_server<AlgorithmMGR>(
    this, "start_algo_task",
    std::bind(
      &AlgorithmTaskManager::HandleAlgorithmManagerGoal,
      this, std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &AlgorithmTaskManager::HandleAlgorithmManagerCancel,
      this, std::placeholders::_1),
    std::bind(
      &AlgorithmTaskManager::HandleAlgorithmManagerAccepted,
      this, std::placeholders::_1), rcl_action_server_get_default_options(), callback_group_);
  stop_algo_task_server_ = this->create_service<protocol::srv::StopAlgoTask>(
    "stop_algo_task",
    std::bind(
      &AlgorithmTaskManager::HandleStopTaskCallback, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    callback_group_);
  SetStatus(ManagerStatus::kIdle);
  return true;
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
    auto executor_ptr = CreateExecutor(task_ref.id, task_ref.out_door);
    if (executor_ptr == nullptr) {
      ERROR("BuildExecutorMap failed, cannot create executor: %s!", task_name.c_str());
      result = false;
      break;
    }
    if (!executor_ptr->Init(
        std::bind(&AlgorithmTaskManager::TaskFeedBack, this, std::placeholders::_1),
        std::bind(&AlgorithmTaskManager::TaskSuccessd, this),
        std::bind(&AlgorithmTaskManager::TaskCancled, this),
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

void AlgorithmTaskManager::HandleStopTaskCallback(
  const protocol::srv::StopAlgoTask::Request::SharedPtr request,
  protocol::srv::StopAlgoTask::Response::SharedPtr response)
{
  if (static_cast<uint8_t>(manager_status_) != request->task_id) {
    ERROR("No task to stop");
    return;
  }
  SetStatus(ManagerStatus::kStoppingTask);
  if (activated_executor_ != nullptr) {
    activated_executor_->Stop(request, response);
  }
  ResetManagerStatus();
}

rclcpp_action::GoalResponse AlgorithmTaskManager::HandleAlgorithmManagerGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const AlgorithmMGR::Goal> goal)
{
  (void)uuid;
  (void)goal;
  INFO("---------------------");
  if (!CheckStatusValid()) {
    ERROR(
      "Cannot accept task: %d, status is invalid!", goal->nav_type);
    return rclcpp_action::GoalResponse::REJECT;
  }
  std::string task_name;
  for (auto task : task_map_) {
    if (task.second.id == goal->nav_type) {
      task_name = task.first;
    }
  }
  auto iter = task_map_.find(task_name);
  if (iter == task_map_.end()) {
    ERROR(
      "Cannot accept task: %d, nav type is invalid!", goal->nav_type);
    return rclcpp_action::GoalResponse::REJECT;
  } else {
    SetTaskExecutor(iter->second.executor);
  }
  SetStatus(static_cast<ManagerStatus>(goal->nav_type));
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AlgorithmTaskManager::HandleAlgorithmManagerCancel(
  const std::shared_ptr<GoalHandleAlgorithmMGR> goal_handle)
{
  INFO("---------------------");
  INFO("Received request to cancel task");
  (void)goal_handle;
  activated_executor_->Cancel();
  auto result = std::make_shared<AlgorithmMGR::Result>();
  activated_executor_.reset();
  manager_status_ = ManagerStatus::kIdle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AlgorithmTaskManager::HandleAlgorithmManagerAccepted(
  const std::shared_ptr<GoalHandleAlgorithmMGR> goal_handle)
{
  SetTaskHandle(goal_handle);
  activated_executor_->Start(goal_handle->get_goal());
}

void AlgorithmTaskManager::TaskFeedBack(const AlgorithmMGR::Feedback::SharedPtr feedback)
{
  goal_handle_executing_->publish_feedback(feedback);
}

void AlgorithmTaskManager::TaskSuccessd()
{
  INFO("Got Executor Success");
  auto result = std::make_shared<AlgorithmMGR::Result>();
  result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_SUCCESS;
  goal_handle_executing_->succeed(result);
  ResetTaskHandle();
  ResetManagerStatus();
}

void AlgorithmTaskManager::TaskCancled()
{
  INFO("Got Executor cancle");
  auto result = std::make_shared<AlgorithmMGR::Result>();
  result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_CANCEL;
  goal_handle_executing_->abort(result);
  ResetTaskHandle();
  ResetManagerStatus();
}
void AlgorithmTaskManager::TaskAborted()
{
  INFO("Got Executor abort");
  auto result = std::make_shared<AlgorithmMGR::Result>();
  result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_FAILED;
  goal_handle_executing_->abort(result);
  ResetTaskHandle();
  ResetManagerStatus();
}
}  // namespace algorithm
}  // namespace cyberdog