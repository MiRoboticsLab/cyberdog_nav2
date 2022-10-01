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

#include "algorithm_manager/algorithm_task_manager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace algorithm
{
AlgorithmTaskManager::AlgorithmTaskManager()
: rclcpp::Node("AlgorithmTaskManager")
{
  // executor_ = std::make_shared<ExecutorBase>("executor");
  executor_laser_mapping_ = std::make_shared<ExecutorLaserMapping>(std::string("LaserMapping"));
  executor_laser_localization_ =
    std::make_shared<ExecutorLaserLocalization>(std::string("LaserLocalization"));
  executor_ab_navigation_ = std::make_shared<ExecutorAbNavigation>(std::string("AbNavigation"));
  executor_auto_dock_ = std::make_shared<ExecutorAutoDock>(std::string("AutoDock"));
  executor_uwb_tracking_ = std::make_shared<ExecutorUwbTracking>(std::string("UwbTracking"));
  executor_vision_tracking_ =
    std::make_shared<ExecutorVisionTracking>(std::string("VisionTracking"));
  navigation_server_ = rclcpp_action::create_server<AlgorithmMGR>(
    this, "CyberdogNavigation",
    std::bind(
      &AlgorithmTaskManager::HandleNavigationGoal,
      this, std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &AlgorithmTaskManager::HandleNavigationCancel,
      this, std::placeholders::_1),
    std::bind(
      &AlgorithmTaskManager::HandleNavigationAccepted,
      this, std::placeholders::_1));
  std::thread{std::bind(&AlgorithmTaskManager::GetExecutorStatus, this)}.detach();
}

AlgorithmTaskManager::~AlgorithmTaskManager()
{
  executor_status_cv_.notify_all();
}


rclcpp_action::GoalResponse AlgorithmTaskManager::HandleNavigationGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const AlgorithmMGR::Goal> goal)
{
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AlgorithmTaskManager::HandleNavigationCancel(
  const std::shared_ptr<GoalHandleAlgorithmMGR> goal_handle)
{
  INFO("Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AlgorithmTaskManager::HandleNavigationAccepted(
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

  switch (goal->nav_type) {
    case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_AB:
      {
        INFO("Start AB Navigation Task");
        // executor_ab_navigation_->Start();
        // activated_executor_ = executor_ab_navigation_;
      }
      break;

    case AlgorithmMGR::Goal::NAVIGATION_TYPE_STOP_AB:
      {
        INFO("Stop AB Navigation Task");
      }
      break;

    case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_MAPPING:
      {
        INFO("Start Mapping Task");
        // executor_ = std::make_shared<ExecutorAbNavigation>();
        // executor_laser_mapping_->Start(goal);
        // activated_executor_ = executor_laser_mapping_;
      }
      break;

    case AlgorithmMGR::Goal::NAVIGATION_TYPE_STOP_MAPPING:
      {
        INFO("Stop Mapping Task");
        // executor_laser_mapping_->Stop();
        // activated_executor_ = executor_laser_mapping_;
      }
      break;

    case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_LOCALIZATION:
      {
        INFO("Start Localization Task");
        // executor_laser_localization_->Start();
        // activated_executor_ = executor_laser_localization_;
      }
      break;

    case AlgorithmMGR::Goal::NAVIGATION_TYPE_STOP_LOCALIZATION:
      {
        INFO("Stop Localization Task");
        // executor_laser_localization_->Stop();
        // activated_executor_ = executor_laser_localization_;
      }
      break;

    case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_AUTO_DOCKING:
      {
        INFO("Start AutoDock Task");
      }
      break;

    case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_UWB_TRACKING:
      {
        INFO("Start UWB Tracking Task");
        executor_uwb_tracking_->Start(goal);
        activated_executor_ = executor_uwb_tracking_;
      }
      break;

    default:
      break;
  }
  executor_status_cv_.notify_all();
}

void AlgorithmTaskManager::GetExecutorStatus()
{
  while (rclcpp::ok()) {
    if (activated_executor_ == nullptr) {
      std::unique_lock<std::mutex> lk(executor_status_mutex_);
      executor_status_cv_.wait(lk);
    }
    if (activated_executor_ == nullptr) {
      continue;
    }
    static auto result = std::make_shared<AlgorithmMGR::Result>();
    ExecutorBase::ExecutorData executor_data = activated_executor_->GetStatus();
    switch (executor_data.status) {
      case ExecutorInterface::ExecutorStatus::kExecuting:
        {
          static auto feedback = std::make_shared<AlgorithmMGR::Feedback>(executor_data.feedback);
          goal_handle_->publish_feedback(feedback);
          break;
        }

      case ExecutorInterface::ExecutorStatus::kSuccess:
        {
          result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_SUCCESS;
          goal_handle_->succeed(result);
          activated_executor_.reset();
          break;
        }

      case ExecutorInterface::ExecutorStatus::kAborted:
        {
          result->result = AlgorithmMGR::Result::NAVIGATION_RESULT_TYPE_FAILED;
          goal_handle_->abort(result);
          activated_executor_.reset();
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
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}
}
}

int main(int argc, char ** argv)
{
  LOGGER_MAIN_INSTANCE("MotionManager");
  cyberdog::debug::register_signal();
  rclcpp::init(argc, argv);
  auto atm = std::make_shared<cyberdog::algorithm::AlgorithmTaskManager>();
  rclcpp::spin(atm);
}
