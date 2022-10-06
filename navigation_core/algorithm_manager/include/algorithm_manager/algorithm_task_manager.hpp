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
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"

// #include "std_srvs/srv/set_bool.hpp"
// #include "std_msgs/msg/int32.hpp"
// #include "cyberdog_common/cyberdog_log.hpp"
#include "algorithm_manager/executor_base.hpp"
#include "algorithm_manager/executor_ab_navigation.hpp"
#include "algorithm_manager/executor_auto_dock.hpp"
#include "algorithm_manager/executor_laser_localization.hpp"
#include "algorithm_manager/executor_laser_mapping.hpp"
#include "algorithm_manager/executor_uwb_tracking.hpp"
#include "algorithm_manager/executor_vision_tracking.hpp"
#include "cyberdog_debug/backtrace.hpp"
namespace cyberdog
{
namespace algorithm
{

using TaskId = uint8_t;
class AlgorithmTaskManager : public rclcpp::Node
{
public:
  AlgorithmTaskManager();
  ~AlgorithmTaskManager();

private:
  struct TaskRef
  {
    TaskId pre_task;
    std::shared_ptr<ExecutorBase> executor_ptr;
  };
  enum class ManagerStatus : uint8_t
  {
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
  void TaskExecute();
  void GetExecutorStatus();
  void UpdateExecutorData(const ExecutorData & executor_data)
  {
    executor_data_queue_.EnQueueOne(executor_data);
  }
  ExecutorData & GetExecutorData()
  {
    executor_data_queue_.DeQueue(executor_data_);
    return executor_data_;
  }
  void ResetAllGoalHandle()
  {
    goal_handle_executing_.reset();
    goal_handle_to_stop_.reset();
    goal_handle_new_.reset();
  }

  rclcpp_action::Server<AlgorithmMGR>::SharedPtr navigation_server_;
  std::shared_ptr<GoalHandleAlgorithmMGR> goal_handle_executing_;
  std::shared_ptr<GoalHandleAlgorithmMGR> goal_handle_to_stop_;
  std::shared_ptr<GoalHandleAlgorithmMGR> goal_handle_new_;
  std::shared_ptr<ExecutorBase> activated_executor_;
  std::shared_ptr<ExecutorAbNavigation> executor_ab_navigation_;
  std::shared_ptr<ExecutorAutoDock> executor_auto_dock_;
  std::shared_ptr<ExecutorLaserMapping> executor_laser_mapping_;
  std::shared_ptr<ExecutorLaserLocalization> executor_laser_localization_;
  std::shared_ptr<ExecutorUwbTracking> executor_uwb_tracking_;
  std::shared_ptr<ExecutorVisionTracking> executor_vision_tracking_;
  std::condition_variable executor_start_cv_, executor_status_cv_;
  std::mutex executor_start_mutex_, executor_status_mutex_;
  ManagerStatus manager_status_{ManagerStatus::kIdle};
  AlgorithmMGR::Feedback::SharedPtr feedback_;
  std::unordered_map<TaskId, TaskRef> task_map_;
  common::MsgQueue<ExecutorData> executor_data_queue_;
  ExecutorData executor_data_;
};  // class algorithm_manager
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__ALGORITHM_TASK_MANAGER_HPP_
