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
  executor_start_cv_.notify_all();
}


rclcpp_action::GoalResponse AlgorithmTaskManager::HandleAlgorithmManagerGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const AlgorithmMGR::Goal> goal)
{
  (void)uuid;
  (void)goal;
  if (manager_status_ != ManagerStatus::kIdle) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AlgorithmTaskManager::HandleAlgorithmManagerCancel(
  const std::shared_ptr<GoalHandleAlgorithmMGR> goal_handle)
{
  INFO("Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AlgorithmTaskManager::HandleAlgorithmManagerAccepted(
  const std::shared_ptr<GoalHandleAlgorithmMGR> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  goal_handle_ = goal_handle;
  std::thread{std::bind(&AlgorithmTaskManager::TaskExecute, this)}.detach();
}

void AlgorithmTaskManager::TaskExecute()
{
  auto goal = goal_handle_->get_goal();
  auto result = std::make_shared<AlgorithmMGR::Result>();
  result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_REJECT;
  switch (goal->nav_type) {
    case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_AB:
      {
        INFO("Receive Start AB Navigation Task");
        executor_ab_navigation_->Start(goal);
        activated_executor_ = executor_ab_navigation_;
        manager_status_ = ManagerStatus::kExecutingAbNavigation;
      }
      break;

    case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_MAPPING:
      {
        INFO("Receive Start Mapping Task");
        executor_laser_mapping_->Start(goal);
        activated_executor_ = executor_laser_mapping_;
        manager_status_ = ManagerStatus::kExecutingLaserMapping;
      }
      break;

    case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_LOCALIZATION:
      {
        INFO("Receive Start LaserLocalization Task");
        executor_laser_localization_->Start(goal);
        activated_executor_ = executor_laser_localization_;
        manager_status_ = ManagerStatus::kExecutingLaserLocalization;
      }
      break;

    case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_AUTO_DOCKING:
      {
        INFO("Receive Start AutoDock Task");
        executor_auto_dock_->Start(goal);
        activated_executor_ = executor_auto_dock_;
        manager_status_ = ManagerStatus::kExecutingAutoDock;
      }
      break;

    case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_UWB_TRACKING:
      {
        INFO("Receive Start UWB Tracking Task");
        if (!executor_uwb_tracking_->Start(goal)) {
          goal_handle_->abort(result);
          return;
        }
        activated_executor_ = executor_uwb_tracking_;
        manager_status_ = ManagerStatus::kExecutingUwbTracking;
      }
      break;

    default:
      break;
  }
  std::unique_lock<std::mutex> lk(executor_start_mutex_);
  executor_start_cv_.notify_all();
}

void AlgorithmTaskManager::GetExecutorStatus()
{
  while (rclcpp::ok()) {
    if (activated_executor_ == nullptr) {
      std::unique_lock<std::mutex> lk(executor_start_mutex_);
      executor_start_cv_.wait(lk);
    }
    if (activated_executor_ == nullptr) {
      continue;
    }
    static auto result = std::make_shared<AlgorithmMGR::Result>();
    ExecutorData executor_data = activated_executor_->GetExecutorData();
    switch (executor_data.status) {
      case ExecutorStatus::kExecuting:
        {
          static auto feedback = std::make_shared<AlgorithmMGR::Feedback>(executor_data.feedback);
          goal_handle_->publish_feedback(feedback);
          break;
        }

      case ExecutorStatus::kSuccess:
        {
          result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_SUCCESS;
          goal_handle_->succeed(result);
          activated_executor_.reset();
          manager_status_ = ManagerStatus::kIdle;
          break;
        }

      case ExecutorStatus::kAborted:
        {
          result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_FAILED;
          goal_handle_->abort(result);
          activated_executor_.reset();
          manager_status_ = ManagerStatus::kIdle;
          break;
        }

      case ExecutorStatus::kCanceled:
        {
          result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_CANCEL;
          goal_handle_->abort(result);
          activated_executor_.reset();
          manager_status_ = ManagerStatus::kIdle;
          break;
        }

      default:
        break;
    }
    if (goal_handle_->is_canceling()) {
      activated_executor_->Cancel();
      result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_CANCEL;
      goal_handle_->canceled(result);
      activated_executor_.reset();
      manager_status_ = ManagerStatus::kIdle;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}
}  // namespace algorithm
}  // namespace cyberdog

int main(int argc, char ** argv)
{
  LOGGER_MAIN_INSTANCE("AlgorithmTaskManager");
  cyberdog::debug::register_signal();
  rclcpp::init(argc, argv);
  auto atm = std::make_shared<cyberdog::algorithm::AlgorithmTaskManager>();
  rclcpp::spin(atm);
}
