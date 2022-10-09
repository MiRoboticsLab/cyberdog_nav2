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
  // TODO: checkout LifeCycle Configure;
  // ExecutorBase::RegisterUpdateImpl(  // *** 这里有问题
  //   std::bind(&AlgorithmTaskManager::UpdateExecutorData, this, std::placeholders::_1));

  if(!BuildExecutorMap()) {
    ERROR("Init failed, cannot build executor map!");
    return false;
  }
  std::thread{std::bind(&AlgorithmTaskManager::TaskFeedBack, this)}.detach();

  navigation_server_ = rclcpp_action::create_server<AlgorithmMGR>(
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
  
  return true;
}

bool AlgorithmTaskManager::BuildExecutorMap()
{
  toml::value executor_config;
  if(!common::CyberdogToml::ParseFile("./config/Executor.toml", executor_config)) {
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
  auto result = std::all_of(executor_vector.cbegin(), executor_vector.cend(), [this, &type_config](const std::string & name){
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
    if(!executor_ptr->Init(std::bind(&AlgorithmTaskManager::TaskFeedBack, std::placeholders::_1),
        std::bind(&AlgorithmTaskManager::TaskSuccessd),
        std::bind(&AlgorithmTaskManager::TaskCancled),
        std::bind(&AlgorithmTaskManager::TaskAborted))) {
      ERROR("BuildExecutorMap failed, cannot init executor: %s!", name.c_str());
      return false;
    }
    this->executor_map_.insert(std::make_pair(type_value, executor_ptr));
  });  // end of all_of
  if((!result && (!executor_map_.empty()))) {
    executor_map_.clear();
  }
  return result;
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
    SetTaskHandle(executor = iter->second);
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  /* 使用封装风格的check函数
  if(!CheckCmdValid(goal->nav_type)) {
    return rclcpp_action::GoalResponse::REJECT;
  } else {
    activated_executor_->Start();
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  */
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
  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  /* obsolete code
  goal_handle_new_ = goal_handle;
  if (goal_handle_executing_ != nullptr) {
    INFO(
      "Receive task %d to stop pre task %d", goal_handle->get_goal()->nav_type,
      static_cast<int>(manager_status_));
    goal_handle_to_stop_ = goal_handle;
    INFO(
      "To interupt: %ld, by: %ld", int64_t(goal_handle_executing_.get()),
      int64_t(goal_handle_to_stop_.get()));
  } else {
    goal_handle_executing_ = goal_handle;
    INFO(
      "To executing: %ld, while: %ld", int64_t(goal_handle_executing_.get()),
      int64_t(goal_handle_to_stop_.get()));
  }

  std::thread{std::bind(&AlgorithmTaskManager::TaskExecute, this)}.detach();
  */
  INFO("%s on call.", __FUNCTION__);
  SetTaskHandle(goal_handle);
  executor_map_.find(goal_handle->get_goal()->nav_type)->second->Start(goal_handle->get_goal());
}

void AlgorithmTaskManager::TaskExecute()
{
  auto goal = goal_handle_new_->get_goal();
  auto result = std::make_shared<AlgorithmMGR::Result>();
  result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_REJECT;

  // activated_executor_ = task_map_.at(goal->nav_type).executor_ptr;

  switch (goal->nav_type) {
    // case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_AB:
    //   {
    //     INFO("Receive Start AB Navigation Task");
    //     activated_executor_ = executor_ab_navigation_;
    //     if (!activated_executor_->Start(goal)) {
    //       goal_handle_->abort(result);
    //       return;
    //     }
    //     manager_status_ = ManagerStatus::kExecutingAbNavigation;
    //   }
    //   break;

    // case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_MAPPING:
    //   {
    //     INFO("Receive Start Mapping Task");
    //     activated_executor_ = executor_laser_mapping_;
    //     if (!activated_executor_->Start(goal)) {
    //       goal_handle_->abort(result);
    //       return;
    //     }
    //     manager_status_ = ManagerStatus::kExecutingLaserMapping;
    //   }
    //   break;

    // case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_LOCALIZATION:
    //   {
    //     INFO("Receive Start LaserLocalization Task");
    //     activated_executor_ = executor_laser_localization_;
    //     if (!activated_executor_->Start(goal)) {
    //       goal_handle_->abort(result);
    //       return;
    //     }
    //     manager_status_ = ManagerStatus::kExecutingLaserLocalization;
    //   }
    //   break;

    // case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_AUTO_DOCKING:
    //   {
    //     INFO("Receive Start AutoDock Task");
    //     activated_executor_ = executor_auto_dock_;
    //     if (!activated_executor_->Start(goal)) {
    //       goal_handle_->abort(result);
    //       return;
    //     }
    //     manager_status_ = ManagerStatus::kExecutingAutoDock;
    //   }
    //   break;

    case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_UWB_TRACKING:
      {
        INFO("Receive Start UWB Tracking Task");
        manager_status_ = ManagerStatus::kExecutingUwbTracking;
        activated_executor_ = executor_uwb_tracking_;
        if (!activated_executor_->Start(goal)) {
          goal_handle_executing_->abort(result);
          ResetAllGoalHandle();
          manager_status_ = ManagerStatus::kIdle;
          return;
        }
      }
      break;

    case AlgorithmMGR::Goal::NAVIGATION_TYPE_STOP_UWB_TRACKING:
      {
        INFO("Receive Stop UWB Tracking Task");
        manager_status_ = ManagerStatus::kExecutingUwbTracking;
        activated_executor_ = executor_uwb_tracking_;
        activated_executor_->Stop();
        // TODO(Harvey): 
        // 当前“停止任务”的任务状态如何判断
        result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_SUCCESS;
        INFO("Will succeed: %ld", int64_t(goal_handle_to_stop_.get()));
        goal_handle_to_stop_->succeed(result);
        goal_handle_to_stop_.reset();
        activated_executor_.reset();
        manager_status_ = ManagerStatus::kIdle;
      }
      break;

    default:
      break;
  }
}

void AlgorithmTaskManager::TaskFeedBack()
{
  ExecutorData executor_data;
  auto feedback_ = std::make_shared<AlgorithmMGR::Feedback>();
  while (rclcpp::ok()) {
    INFO("Will Check ExecutorData");
    if(!GetFeedBack(executor_data)) {
      continue;
    }
    INFO("Finish Check ExecutorData");
    if (!rclcpp::ok()) {
      return;
    }
    *feedback_ = executor_data.feedback;
    INFO("feedback: %d", feedback_->feedback_code);
    goal_handle_executing_->publish_feedback(feedback_);

    #if 0 obsolete code
    switch (executor_data.status) {
      case ExecutorStatus::kExecuting:
        {
          *feedback_ = executor_data.feedback;
          INFO("feedback: %d", feedback_->feedback_code);
          goal_handle_executing_->publish_feedback(feedback_);
          break;
        }

      case ExecutorStatus::kSuccess:
        {
          INFO("Got ExecutorData Success");
          result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_SUCCESS;
          goal_handle_executing_->succeed(result);
          activated_executor_.reset();
          ResetAllGoalHandle();
          manager_status_ = ManagerStatus::kIdle;
          break;
        }

      case ExecutorStatus::kAborted:
        {
          INFO("Got ExecutorData Aborted");
          result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_FAILED;
          goal_handle_executing_->abort(result);
          activated_executor_.reset();
          ResetAllGoalHandle();
          manager_status_ = ManagerStatus::kIdle;
          break;
        }

      case ExecutorStatus::kCanceled:
        {
          INFO("Got ExecutorData Canceled");
          result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_CANCEL;
          // NOTE
          /**
           * @brief 
           * 在收到执行器kCanceled状态时，调用abort()接口；
           * canceled()接口在这里不能用，只适用于server收到cancel请求、goal被置
           * 为canceling时的调用；
           * 在manager的场景下，没有判断is_canceling()的需求，所以abort()可以涵
           * 盖goal被Stop和Cancel的两种情况；
           * 
           */
          goal_handle_executing_->abort(result);
          activated_executor_.reset();
          ResetAllGoalHandle();
          manager_status_ = ManagerStatus::kIdle;
          break;
        }

      default:
        break;
    }
    #endif
  }
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
