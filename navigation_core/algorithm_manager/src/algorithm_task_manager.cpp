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
  navigation_server_ = rclcpp_action::create_server<Navigation>(
    this, "CyberdogNavigation",
    std::bind(&AlgorithmTaskManager::HandleNavigationGoal,
      this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&AlgorithmTaskManager::HandleNavigationCancel,
      this, std::placeholders::_1),
    std::bind(&AlgorithmTaskManager::HandleNavigationAccepted,
      this, std::placeholders::_1));
  std::thread{std::bind(&AlgorithmTaskManager::PublishFeedback, this)}.detach();

}


rclcpp_action::GoalResponse AlgorithmTaskManager::HandleNavigationGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Navigation::Goal> goal)
{
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AlgorithmTaskManager::HandleNavigationCancel(
  const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
  INFO("Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AlgorithmTaskManager::HandleNavigationAccepted(
  const std::shared_ptr<GoalHandleNavigation> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  goal_handle_ = goal_handle;
  std::thread{std::bind(&AlgorithmTaskManager::TaskExecute, this)}.detach();
}

void AlgorithmTaskManager::TaskExecute()
{
  const auto goal = goal_handle_->get_goal();
  auto result = std::make_shared<Navigation::Result>();

  switch (goal->nav_type) {
    case Navigation::Goal::NAVIGATION_TYPE_START_AB:
      {
        INFO("Start AB Navigation Task");
        // executor_ = std::make_shared<ExecutorAbNavigation>();
        executor_->Start();
      }
      break;

    case Navigation::Goal::NAVIGATION_TYPE_STOP_AB:
      {
        INFO("Stop AB Navigation Task");
      }
      break;

    case Navigation::Goal::NAVIGATION_TYPE_START_MAPPING:
      {
        INFO("Start Mapping Task");
        // executor_ = std::make_shared<ExecutorAbNavigation>();
        executor_->Start();
      }
      break;

    case Navigation::Goal::NAVIGATION_TYPE_STOP_MAPPING:
      {
        INFO("Stop Mapping Task");
      }
      break;

    case Navigation::Goal::NAVIGATION_TYPE_START_LOCALIZATION:
      {
        INFO("Start Localization Task");
      }
      break;

    case Navigation::Goal::NAVIGATION_TYPE_STOP_LOCALIZATION:
      {
        INFO("Stop Localization Task");
      }
      break;

    case Navigation::Goal::NAVIGATION_TYPE_START_AUTO_DOCKING:
      {
        INFO("Start AutoDock Task");
      }
      break;

    case Navigation::Goal::NAVIGATION_TYPE_START_UWB_TRACKING:
      {
        INFO("Start UWB Tracking Task");
      }
      break;

    default:
      break;
  }
}

void AlgorithmTaskManager::PublishFeedback()
{
  while (rclcpp::ok())
  {
    if (executor_->GetStatus() == ExecutorBase::ExecutorStatus::kIdle) {
      std::this_thread::yield();
    }
    static auto feedback = std::make_shared<Navigation::Feedback>();
    executor_->GetFeedback(feedback);
    goal_handle_->publish_feedback(feedback);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

}
}

int main(int argc, char** argv) 
{
  rclcpp::init(argc, argv);
  auto atm = std::make_shared<cyberdog::algorithm::AlgorithmTaskManager>();
  rclcpp::spin(atm);
}