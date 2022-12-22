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
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "algorithm_manager/executor_base.hpp"
#include "cyberdog_debug/backtrace.hpp"
#include "protocol/srv/stop_algo_task.hpp"
#include "protocol/srv/algo_task_status.hpp"
#include "motion_action/motion_macros.hpp"
namespace cyberdog
{
namespace algorithm
{

using TaskId = uint8_t;

enum class ManagerStatus : uint8_t
{
  kUninitialized = 100,
  kIdle = 101,
  kLaunchingLifecycleNode = 102,
  kStoppingTask = 103,
  kExecutingLaserMapping = AlgorithmMGR::Goal::NAVIGATION_TYPE_START_MAPPING,
  kExecutingLaserLocalization = AlgorithmMGR::Goal::NAVIGATION_TYPE_START_LOCALIZATION,
  kExecutingAbNavigation = AlgorithmMGR::Goal::NAVIGATION_TYPE_START_AB,
  kExecutingAutoDock = AlgorithmMGR::Goal::NAVIGATION_TYPE_START_AUTO_DOCKING,
  kExecutingUwbTracking = AlgorithmMGR::Goal::NAVIGATION_TYPE_START_UWB_TRACKING,
  kShuttingDownUwbTracking = AlgorithmMGR::Goal::NAVIGATION_TYPE_STOP_UWB_TRACKING,
};

std::string ToString(const ManagerStatus & status);

struct TaskRef
{
  std::shared_ptr<ExecutorBase> executor;
  std::unordered_map<std::string, std::shared_ptr<Nav2LifecyleMgrClient>> nav2_lifecycle_clients;
  std::unordered_map<std::string, LifecycleClients> lifecycle_nodes;
  uint8_t id;
  bool out_door;
};

class AlgorithmTaskManager
{
public:
  AlgorithmTaskManager();
  ~AlgorithmTaskManager();
  bool Init();
  void Run();

private:
  void TaskSuccessd();
  void TaskCanceled();
  void TaskAborted();
  void TaskFeedBack(const AlgorithmMGR::Feedback::SharedPtr feedback);

  bool CheckStatusValid()
  {
    auto status = GetStatus();
    INFO("Current status: %s", ToString(status).c_str());
    return status == ManagerStatus::kIdle;
  }

  void SetStatus(const ManagerStatus & status)
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    manager_status_ = status;
  }

  ManagerStatus & GetStatus()
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    return manager_status_;
  }

  void ResetManagerStatus()
  {
    SetStatus(ManagerStatus::kIdle);
  }

  void SetFeedBack(const ExecutorData & executor_data)
  {
    executor_data_queue_.EnQueueOne(executor_data);
  }

  bool GetFeedBack(ExecutorData & executor_data)
  {
    return executor_data_queue_.DeQueue(executor_data);
  }

  bool BuildExecutorMap();

  void SetTaskHandle(std::shared_ptr<GoalHandleAlgorithmMGR> goal_handle = nullptr)
  {
    goal_handle_executing_ = (goal_handle == nullptr ? goal_handle_executing_ : goal_handle);
  }

  void SetTaskExecutor(std::shared_ptr<ExecutorBase> executor = nullptr)
  {
    activated_executor_ = (executor == nullptr ? activated_executor_ : executor);
  }

  void ResetTaskHandle()
  {
    goal_handle_executing_.reset();
    activated_executor_.reset();
  }

private:
  rclcpp_action::GoalResponse HandleAlgorithmManagerGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const AlgorithmMGR::Goal> goal);
  rclcpp_action::CancelResponse HandleAlgorithmManagerCancel(
    const std::shared_ptr<GoalHandleAlgorithmMGR> goal_handle);
  void HandleAlgorithmManagerAccepted(
    const std::shared_ptr<GoalHandleAlgorithmMGR> goal_handle);
  // void TaskExecute();
  void HandleStopTaskCallback(
    const protocol::srv::StopAlgoTask::Request::SharedPtr request,
    protocol::srv::StopAlgoTask::Response::SharedPtr response);
  void HandleTaskStatusCallback(
    const protocol::srv::AlgoTaskStatus::Request::SharedPtr request,
    protocol::srv::AlgoTaskStatus::Response::SharedPtr response);

private:
  rclcpp::Node::SharedPtr node_{nullptr};
  rclcpp_action::Server<AlgorithmMGR>::SharedPtr start_algo_task_server_{nullptr};
  rclcpp::Service<protocol::srv::StopAlgoTask>::SharedPtr stop_algo_task_server_{nullptr};
  rclcpp::Service<protocol::srv::AlgoTaskStatus>::SharedPtr algo_task_status_server_{nullptr};
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
  std::shared_ptr<GoalHandleAlgorithmMGR> goal_handle_executing_{nullptr};
  std::shared_ptr<ExecutorBase> activated_executor_{nullptr};
  rclcpp::executors::MultiThreadedExecutor::SharedPtr ros_executor_{nullptr};
  ManagerStatus manager_status_{ManagerStatus::kUninitialized};
  std::mutex status_mutex_;
  common::MsgQueue<ExecutorData> executor_data_queue_;
  std::unordered_map<uint8_t, std::shared_ptr<ExecutorBase>> executor_map_;
  std::unordered_map<std::string, TaskRef> task_map_;
};  // class algorithm_manager
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__ALGORITHM_TASK_MANAGER_HPP_
