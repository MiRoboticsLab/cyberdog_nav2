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
#include "algorithm_manager/algorithm_task_manager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace algorithm
{
AlgorithmTaskManager::AlgorithmTaskManager()
: rclcpp::Node("AlgorithmTaskManager")
{
  executor_laser_mapping_ =
    std::make_shared<ExecutorLaserMapping>(std::string("LaserMapping"));
  executor_laser_localization_ =
    std::make_shared<ExecutorLaserLocalization>(std::string("LaserLocalization"));
  executor_ab_navigation_ =
    std::make_shared<ExecutorAbNavigation>(std::string("AbNavigation"));
  executor_auto_dock_ =
    std::make_shared<ExecutorAutoDock>(std::string("AutoDock"));
  executor_uwb_tracking_ =
    std::make_shared<ExecutorUwbTracking>(std::string("UwbTracking"));
  executor_vision_tracking_ =
    std::make_shared<ExecutorVisionTracking>(std::string("VisionTracking"));

  // task_map_.emplace(5, TaskRef{0, executor_uwb_tracking_});
  // task_map_.emplace(6, TaskRef{5, executor_uwb_tracking_});
  // task_map_.emplace(11, TaskRef{0, executor_uwb_tracking_});
  // task_map_.emplace(12, TaskRef{11, executor_uwb_tracking_});

  task_map_.emplace("UwbTracking", TaskRef{
    std::make_shared<ExecutorUwbTracking>(std::string("UwbTracking")), 
    std::unordered_map<std::string, std::shared_ptr<Nav2LifecyleMgrClient>>{
      {"lifecycle_manager_navigation", nullptr},
      {"lifecycle_manager_mcr_uwb", nullptr}}, 
    std::unordered_map<std::string, LifecycleClients>{
      {"", {nullptr, nullptr}},
      {"", {nullptr, nullptr}}},
    AlgorithmMGR::Goal::NAVIGATION_TYPE_START_UWB_TRACKING,
    false});
  feedback_ = std::make_shared<AlgorithmMGR::Feedback>();
  // ExecutorBase::RegisterUpdateImpl(
  //   std::bind(&AlgorithmTaskManager::UpdateExecutorData, this, std::placeholders::_1));
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
  std::thread{std::bind(&AlgorithmTaskManager::GetExecutorStatus, this)}.detach();
}

AlgorithmTaskManager::~AlgorithmTaskManager()
{
  executor_start_cv_.notify_one();
}


rclcpp_action::GoalResponse AlgorithmTaskManager::HandleAlgorithmManagerGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const AlgorithmMGR::Goal> goal)
{
  (void)uuid;
  (void)goal;
  INFO("---------------------");
  std::unique_lock<std::mutex> lk(executor_start_mutex_);
  if(manager_status_ != ManagerStatus::kIdle) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  executor_start_cv_.notify_one();
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
  /* 是否有必要启用线程，直接调用算法类指针执行？ */
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

void AlgorithmTaskManager::GetExecutorStatus()
{
  while (rclcpp::ok()) {
    static auto result = std::make_shared<AlgorithmMGR::Result>();
    INFO("Will Check ExecutorData");
    // ExecutorData executor_data = activated_executor_->GetExecutorData();
    ExecutorData executor_data = GetExecutorData();
    INFO("Finish Check ExecutorData");
    if (!rclcpp::ok()) {
      return;
    }
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
  }
}
}  // namespace algorithm
}  // namespace cyberdog
