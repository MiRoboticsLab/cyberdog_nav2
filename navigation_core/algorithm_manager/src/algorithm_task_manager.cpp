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
  // executor_start_cv_.notify_one();
}

bool AlgorithmTaskManager::Init()
{
  if(!BuildExecutorMap()) {
    ERROR("Init failed, cannot build executor map!");
    return false;
  }

  start_algo_task_server_ = rclcpp_action::create_server<AlgorithmMGR>(
    this, "CyberdogNavigation",
    std::bind(
      &AlgorithmTaskManager::HandleAlgorithmManagerGoal,
      this, std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &AlgorithmTaskManager::HandleAlgorithmManagerCancel,
      this, std::placeholders::_1),
    std::bind(
      &AlgorithmTaskManager::HandleAlgorithmManagerAccepted,
      this, std::placeholders::_1));
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  stop_algo_task_server_ = this->create_service<protocol::srv::StopAlgoTask>(
    "stop_algo_task", 
    std::bind(&AlgorithmTaskManager::HandleStopTaskCallback, this,
      std::placeholders::_1, std::placeholders::_2), 
    rmw_qos_profile_services_default, 
    callback_group_);
  SetStatus(ManagerStatus::kIdle);
  return true;
}

bool AlgorithmTaskManager::BuildExecutorMap()
{
  toml::value executor_config;
  if(!common::CyberdogToml::ParseFile(
    ament_index_cpp::get_package_share_directory("algorithm_manager") +
      "/config/Executor.toml", executor_config)) {
    ERROR("BuildExecutorMap failed, cannot parse config file!");
    return false;
  }
  std::vector<std::string> executor_vector;
  if(!common::CyberdogToml::Get(executor_config, "executors", executor_vector)) {
    ERROR("BuildExecutorMap failed, cannot get executors array!");
    return false;
  }
  toml::value type_config;
  if(!common::CyberdogToml::Get(executor_config, "type", type_config)) {
    ERROR("BuildExecutorMap failed, cannot get executors type!");
    return false;
  }
  auto result = std::all_of(executor_vector.cbegin(), executor_vector.cend(), 
    [this, &type_config](const std::string & name){
      uint8_t type_value;
      if(!common::CyberdogToml::Get(type_config, name, type_value)) {
        ERROR("BuildExecutorMap failed, cannot get executor: %s type value!", name.c_str());
        return false;
      }
      auto executor_ptr = CreateExecutor(type_value);
      if(executor_ptr == nullptr) {
        ERROR("BuildExecutorMap failed, cannot create executor: %s!", name.c_str());
        return false;
      }
      if(!executor_ptr->Init(std::bind(&AlgorithmTaskManager::TaskFeedBack, this, std::placeholders::_1),
          std::bind(&AlgorithmTaskManager::TaskSuccessd, this),
          std::bind(&AlgorithmTaskManager::TaskCancled, this),
          std::bind(&AlgorithmTaskManager::TaskAborted, this))) {
        ERROR("BuildExecutorMap failed, cannot init executor: %s!", name.c_str());
        return false;
      }
      this->executor_map_.insert(std::make_pair(type_value, executor_ptr));
      return true;
    }
  );  // end of all_of
  if((!result && (!executor_map_.empty()))) {
    executor_map_.clear();
  }
  return result;
}

void AlgorithmTaskManager::HandleStopTaskCallback(
  const protocol::srv::StopAlgoTask::Request::SharedPtr request,
  protocol::srv::StopAlgoTask::Response::SharedPtr response) 
{
  if(static_cast<uint8_t>(manager_status_) != request->task_id) {
    ERROR("No task to stop");
    return;
  }
  SetStatus(ManagerStatus::kStoppingTask);
  activated_executor_->Stop(request);
  response->result = StopTaskSrv::Response::SUCCESS;
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
  auto iter = executor_map_.find(goal->nav_type);
  if(iter == executor_map_.end()) {
    ERROR(
      "Cannot accept task: %d, nav type is invalid!", goal->nav_type);
    return rclcpp_action::GoalResponse::REJECT;
  } else {
    // activated_executor_ = iter->second;
    SetTaskExecutor(iter->second);
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
  INFO("%s on call.", __FUNCTION__);
  SetTaskHandle(goal_handle);
  executor_map_.find(goal_handle->get_goal()->nav_type)->second->Start(goal_handle->get_goal());
}

void AlgorithmTaskManager::TaskFeedBack(const AlgorithmMGR::Feedback::SharedPtr feedback)
{
    goal_handle_executing_->publish_feedback(feedback);
}

void AlgorithmTaskManager::TaskSuccessd()
{
  INFO("Got ExecutorData Success");
  auto result = std::make_shared<AlgorithmMGR::Result>();
  result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_SUCCESS;
  goal_handle_executing_->succeed(result);
  ResetTaskHandle();
  ResetManagerStatus();
}

void AlgorithmTaskManager::TaskCancled()
{
  INFO("Got ExecutorData cancle");
  auto result = std::make_shared<AlgorithmMGR::Result>();
  result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_CANCEL;
  goal_handle_executing_->abort(result);
  ResetTaskHandle();
  ResetManagerStatus();
}
void AlgorithmTaskManager::TaskAborted()
{
  INFO("Got ExecutorData abort");
  auto result = std::make_shared<AlgorithmMGR::Result>();
  result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_FAILED;
  goal_handle_executing_->abort(result);
  ResetTaskHandle();
  ResetManagerStatus();
}
}  // namespace algorithm
}  // namespace cyberdog
