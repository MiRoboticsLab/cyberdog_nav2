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
#include "protocol/msg/algo_task_status.hpp"
#include "motion_action/motion_macros.hpp"
#include "cyberdog_machine/cyberdog_fs_machine.hpp"
#include "cyberdog_machine/cyberdog_heartbeats.hpp"
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
  kExecutingLaserMapping = 5,
  kExecutingVisMapping = 15,
  kExecutingLaserLocalization = 7,
  kExecutingVisLocalization = 17,
  kLaserLocalizationFailed = 6,
  kVisLocalizationFailed = 16,
  kLaserLocalizing = 8,
  kVisLocalizing = 18,
  kExecutingLaserAbNavigation = 1,
  kExecutingVisAbNavigation = 21,
  kExecutingAutoDock = 9,
  kExecutingUwbTracking = 11,
  kExecutingHumanTracking = 13,
  kExecutingFollowing = 3,
};

enum class FsmState : uint8_t
{
  kUninit,
  kSetup,
  kTearDown,
  kSelfCheck,
  kActive,
  kDeactive,
  kProtected,
  kLowPower,
  kOTA,
  kError
};

enum class AlgoTaskCode : uint8_t
{
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

class AlgorithmTaskManager : public machine::MachineActuator
{
public:
  AlgorithmTaskManager();
  ~AlgorithmTaskManager();
  bool Init();
  void Run();

private:
  void SetState(const FsmState & state)
  {
    std::unique_lock<std::mutex> lk(state_mutex_);
    state_ = state;
  }
  FsmState &
  GetState()
  {
    std::unique_lock<std::mutex> lk(state_mutex_);
    return state_;
  }
  bool IsStateValid(int32_t & code)
  {
    auto state = GetState();
    if (state == FsmState::kActive || state == FsmState::kProtected) {
      code = code_ptr_->GetKeyCode(system::KeyCode::kOK);
      return true;
    } else if (state == FsmState::kLowPower) {
      OnlineAudioPlay("低功耗模式，任务启动失败");
    } else {
      OnlineAudioPlay("状态机无效，任务启动失败");
    }
    ERROR("FSM invalid with current state: %s", status_map_.at(state_).c_str());
    code = code_ptr_->GetKeyCode(system::KeyCode::kStateInvalid);
    return false;
  }

  int32_t OnSetUp()
  {
    INFO("Get fsm: Setup");
    SetState(FsmState::kSetup);
    return code_ptr_->GetKeyCode(system::KeyCode::kOK);
  }

  int32_t OnTearDown()
  {
    INFO("Get fsm: TearDown");
    SetState(FsmState::kTearDown);
    return code_ptr_->GetKeyCode(system::KeyCode::kOK);
  }

  int32_t OnSelfCheck()
  {
    INFO("Get fsm: SelfCheck");
    SetState(FsmState::kSelfCheck);
    return code_ptr_->GetKeyCode(system::KeyCode::kOK);
  }

  int32_t OnActive()
  {
    INFO("Get fsm: Active");
    SetState(FsmState::kActive);
    return code_ptr_->GetKeyCode(system::KeyCode::kOK);
  }

  int32_t OnDeActive()
  {
    INFO("Get fsm: Deactive");
    SetState(FsmState::kDeactive);
    return code_ptr_->GetKeyCode(system::KeyCode::kOK);
  }

  int32_t OnProtected()
  {
    INFO("Get fsm: Protected");
    SetState(FsmState::kProtected);
    return code_ptr_->GetKeyCode(system::KeyCode::kOK);
  }

  int32_t OnLowPower()
  {
    INFO("Get fsm: LowPower");
    SetState(FsmState::kLowPower);
    return code_ptr_->GetKeyCode(system::KeyCode::kOK);
  }

  int32_t OnOTA()
  {
    INFO("Get fsm: OTA");
    SetState(FsmState::kOTA);
    return code_ptr_->GetKeyCode(system::KeyCode::kOK);
  }

  int32_t OnError()
  {
    INFO("Get fsm: OTA");
    SetState(FsmState::kError);
    return code_ptr_->GetKeyCode(system::KeyCode::kOK);
  }
  void OnlineAudioPlay(const std::string & text)
  {
    static bool playing = false;
    if (playing) {
      return;
    }
    auto request = std::make_shared<protocol::srv::AudioTextPlay::Request>();
    request->is_online = true;
    request->module_name = "AlgorithmManager";
    request->text = text;
    playing = true;
    auto callback = [](rclcpp::Client<protocol::srv::AudioTextPlay>::SharedFuture future) {
        playing = false;
        INFO("Audio play result: %s", future.get()->status == 0 ? "success" : "failed");
      };
    auto future = audio_client_->async_send_request(request, callback);
    if (future.wait_for(std::chrono::milliseconds(500)) == std::future_status::timeout) {
      playing = false;
      ERROR("Cannot get response from AudioPlay");
    }
  }
  void TaskSuccessd();
  void TaskCanceled();
  void TaskAborted();
  void TaskFeedBack(const AlgorithmMGR::Feedback::SharedPtr feedback);

  bool CheckStatusValid()
  {
    auto status = GetStatus();
    INFO("Current status : %d", (int)status);
    return status == ManagerStatus::kIdle ||
           status == ManagerStatus::kExecutingLaserMapping ||
           status == ManagerStatus::kExecutingVisMapping ||
           status == ManagerStatus::kLaserLocalizationFailed ||
           status == ManagerStatus::kVisLocalizationFailed;
  }

  bool CheckStatusValid(std::shared_ptr<const AlgorithmMGR::Goal> goal)
  {
    auto status = GetStatus();
    INFO("Current status : %d", (int)status);
    // INFO("Current status: %s", ToString(status).c_str());
    if (goal->nav_type == AlgorithmMGR::Goal::NAVIGATION_TYPE_START_AB) {
      if (goal->outdoor) {
        return status == ManagerStatus::kVisLocalizing;
      } else {
        return status == ManagerStatus::kLaserLocalizing;
      }
    }
    return status == ManagerStatus::kIdle ||
           status == ManagerStatus::kLaserLocalizationFailed ||
           status == ManagerStatus::kVisLocalizationFailed;
  }

  void SetStatus(const ManagerStatus & status)
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    manager_status_ = status;
  }

  void SetStatus(std::shared_ptr<const AlgorithmMGR::Goal> goal)
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    if (goal->nav_type == AlgorithmMGR::Goal::NAVIGATION_TYPE_START_MAPPING) {
      if (goal->outdoor) {
        manager_status_ = ManagerStatus::kExecutingVisMapping;
      } else {
        manager_status_ = ManagerStatus::kExecutingLaserMapping;
      }
      return;
    }
    if (goal->nav_type == AlgorithmMGR::Goal::NAVIGATION_TYPE_START_LOCALIZATION) {
      if (goal->outdoor) {
        manager_status_ = ManagerStatus::kExecutingVisLocalization;
      } else {
        manager_status_ = ManagerStatus::kExecutingLaserLocalization;
      }
      return;
    }
    if (goal->nav_type == AlgorithmMGR::Goal::NAVIGATION_TYPE_START_AB) {
      if (goal->outdoor) {
        manager_status_ = ManagerStatus::kExecutingVisAbNavigation;
      } else {
        manager_status_ = ManagerStatus::kExecutingLaserAbNavigation;
      }
    }
    if (goal->nav_type == AlgorithmMGR::Goal::NAVIGATION_TYPE_START_HUMAN_TRACKING) {
      if (goal->object_tracking) {
        manager_status_ = ManagerStatus::kExecutingFollowing;
      } else {
        manager_status_ = ManagerStatus::kExecutingHumanTracking;
      }
      return;
    }
    manager_status_ = static_cast<ManagerStatus>(goal->nav_type);
  }

  ManagerStatus & GetStatus()
  {
    std::lock_guard<std::mutex> lk(status_mutex_);
    return manager_status_;
  }

  uint8_t ReParseStatus(const ManagerStatus & status)
  {
    if (status == ManagerStatus::kExecutingLaserMapping ||
      status == ManagerStatus::kExecutingVisMapping)
    {
      return AlgorithmMGR::Goal::NAVIGATION_TYPE_START_MAPPING;
    }
    if (status == ManagerStatus::kExecutingLaserLocalization ||
      status == ManagerStatus::kExecutingVisLocalization)
    {
      return AlgorithmMGR::Goal::NAVIGATION_TYPE_START_LOCALIZATION;
    }
    if (status == ManagerStatus::kExecutingHumanTracking ||
      status == ManagerStatus::kExecutingFollowing)
    {
      return AlgorithmMGR::Goal::NAVIGATION_TYPE_START_HUMAN_TRACKING;
    }
    if (status == ManagerStatus::kExecutingLaserAbNavigation ||
      status == ManagerStatus::kExecutingVisAbNavigation)
    {
      return AlgorithmMGR::Goal::NAVIGATION_TYPE_START_AB;
    }
    return static_cast<uint8_t>(status);
  }

  void ResetManagerStatus()
  {
    SetStatus(ManagerStatus::kIdle);
  }

  void ResetManagerSubStatus()
  {
    global_feedback_ = 0;
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

  void PublishStatus()
  {
    global_task_status_.task_status = static_cast<uint8_t>(manager_status_);
    global_task_status_.task_sub_status = global_feedback_;
    algo_task_status_publisher_->publish(global_task_status_);
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
  rclcpp::Publisher<protocol::msg::AlgoTaskStatus>::SharedPtr algo_task_status_publisher_{nullptr};
  std::shared_ptr<GoalHandleAlgorithmMGR> goal_handle_executing_{nullptr};
  std::shared_ptr<ExecutorBase> activated_executor_{nullptr};
  rclcpp::executors::MultiThreadedExecutor::SharedPtr ros_executor_{nullptr};
  ManagerStatus manager_status_{ManagerStatus::kUninitialized};
  std::mutex status_mutex_;
  common::MsgQueue<ExecutorData> executor_data_queue_;
  std::unordered_map<uint8_t, std::shared_ptr<ExecutorBase>> executor_map_;
  std::unordered_map<std::string, TaskRef> task_map_;
  protocol::msg::AlgoTaskStatus global_task_status_;
  int32_t global_feedback_{0};
  std::shared_ptr<system::CyberdogCode<AlgoTaskCode>> code_ptr_{nullptr};
  rclcpp::Client<protocol::srv::AudioTextPlay>::SharedPtr audio_client_{nullptr};
  std::unique_ptr<cyberdog::machine::HeartBeatsActuator> heart_beats_ptr_{nullptr};
  FsmState state_{FsmState::kUninit};
  std::unordered_map<FsmState, std::string> status_map_;
  std::mutex state_mutex_;
};  // class algorithm_manager
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__ALGORITHM_TASK_MANAGER_HPP_
