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

#ifndef ALGORITHM_MANAGER__ALGORITHM_TASK_MANAGER_HPP_
#define ALGORITHM_MANAGER__ALGORITHM_TASK_MANAGER_HPP_

#include <memory>
#include <vector>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"

// #include "std_srvs/srv/set_bool.hpp"
// #include "std_msgs/msg/int32.hpp"
// #include "cyberdog_common/cyberdog_log.hpp"
#include "algorithm_manager/executor_base.hpp"
#include "cyberdog_debug/backtrace.hpp"
namespace cyberdog
{
namespace algorithm
{
std::vector<std::string> ExecutorVector_V{"LaserMapping", "LaserLocalization", "AbNavigation", "AutoDock", "UwbTracking", "VisionTracking"};
std::vector<uint8_t> ExecutorVector_V{1, 3, 5, 7, 9, 11};

using TaskId = uint8_t;
class AlgorithmTaskManager : public rclcpp::Node
{
public:
  AlgorithmTaskManager();
  ~AlgorithmTaskManager();

  bool Init();
private:
  /* task result api, callback functions */
  void TaskSuccessd() {}
  void TaskCancled() {}
  void TaskAborted() {}
  void TaskFeedBack();

  /* task check cmd is valid or not */
  bool CheckStatusValid() {
    std::lock_guard<std::mutex> lk(status_mutex_);
    return manager_status_ == ManagerStatus::kIdle;
  }

  void SetStatus(ManagerStatus status) {
    std::lock_guard<std::mutex> lk(status_mutex_);
    manager_status_ = status;
  }

  void ResetManagerStatus() {
    SetStatus(ManagerStatus::kIdle);
  }

  void SetFeedBack(const ExecutorData & executor_data)
  {
    executor_data_queue_.EnQueueOne(executor_data);
  }

  bool GetFeedBack(ExecutorData & executor_data)
  {
    executor_data_queue_.DeQueue(executor_data);
    return executor_data_;
  }
  void BuildExecutorMap();

  void SetTaskHandle(std::shared_ptr<ExecutorBase> executor = nullptr, std::shared_ptr<GoalHandleAlgorithmMGR> goal_handle = nullptr) {
    activated_executor_ = (executor == nullptr ? activated_executor_ : executor);
    goal_handle_executing_ = (goal_handle == nullptr ? goal_handle_executing_ : goal_handle);
  }

  void ResetTaskHandle()
  {
    goal_handle_executing_.reset();
    activated_executor_.reset();
  }
private:
  enum class ManagerStatus : uint8_t
  {
    kUninitialized,
    kIdle,
    kLaunchingLifecycleNode = 100,
    kExecutingLaserMapping = AlgorithmMGR::Goal::NAVIGATION_TYPE_START_MAPPING,
    kExecutingLaserLocalization = AlgorithmMGR::Goal::NAVIGATION_TYPE_START_LOCALIZATION,
    kExecutingAbNavigation = AlgorithmMGR::Goal::NAVIGATION_TYPE_START_AB,
    kExecutingAutoDock = AlgorithmMGR::Goal::NAVIGATION_TYPE_START_AUTO_DOCKING,
    kExecutingUwbTracking = AlgorithmMGR::Goal::NAVIGATION_TYPE_START_UWB_TRACKING,
    kShuttingDownUwbTracking = AlgorithmMGR::Goal::NAVIGATION_TYPE_STOP_UWB_TRACKING,
  };

  rclcpp_action::GoalResponse HandleAlgorithmManagerGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const AlgorithmMGR::Goal> goal);
  rclcpp_action::CancelResponse HandleAlgorithmManagerCancel(
    const std::shared_ptr<GoalHandleAlgorithmMGR> goal_handle);
  void HandleAlgorithmManagerAccepted(
    const std::shared_ptr<GoalHandleAlgorithmMGR> goal_handle);
  // void TaskExecute();

private:
  rclcpp_action::Server<AlgorithmMGR>::SharedPtr navigation_server_{nullptr};
  std::shared_ptr<GoalHandleAlgorithmMGR> goal_handle_executing_{nullptr};
  std::shared_ptr<ExecutorBase> activated_executor_{nullptr};
  ManagerStatus manager_status_{ManagerStatus::kUninitialized};
  std::mutex status_mutex_;
  common::MsgQueue<ExecutorData> executor_data_queue_;
  std::unordered_map executor_map_;
};  // class algorithm_manager
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__ALGORITHM_TASK_MANAGER_HPP_
