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
  SetStatus(ManagerStatus::kIdle);
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

void AlgorithmTaskManager::HandleStopTaskCallback(
  const protocol::srv::StopAlgoTask::Request::SharedPtr request,
  protocol::srv::StopAlgoTask::Response::SharedPtr response)
{
  INFO("=====================");
  if (request->task_id == 0) {
    auto status = GetStatus();
    if (status != ManagerStatus::kExecutingAbNavigation) {
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
    auto status = GetStatus();
    if (static_cast<uint8_t>(status) != request->task_id) {
      ERROR("Task %d cannot stop when %d", request->task_id, (int)status);
      return;
    }
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
    if (task.second.id == goal->nav_type && task.second.out_door == goal->outdoor) {
      task_name = task.first;
      break;
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
  if (goal_handle_executing_ != nullptr) {
    goal_handle_executing_->publish_feedback(feedback);
  }
}

void AlgorithmTaskManager::TaskSuccessd()
{
  INFO("Got Executor Success");
  auto result = std::make_shared<AlgorithmMGR::Result>();
  result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_SUCCESS;
  goal_handle_executing_->succeed(result);
  INFO("Manager success");
  ResetTaskHandle();
  INFO("Manager TaskHandle reset bc success");
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
  ResetManagerStatus();
}
}  // namespace algorithm
}  // namespace cyberdog
