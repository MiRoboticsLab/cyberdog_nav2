// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#ifndef ALGORITHM_MANAGER__EXECUTOR_BASE_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_BASE_HPP_

#include <memory>
#include <unordered_map>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "protocol/action/navigation.hpp"
#include "protocol/action/seat_adjust.hpp"
#include "mcr_msgs/action/automatic_recharge.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
// #include "nav2_msgs/action/follow_waypoints.hpp"
// #include "nav2_msgs/action/navigate_through_poses.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
#include "mcr_msgs/action/target_tracking.hpp"
// #include "nav2_util/geometry_utils.hpp"
// #include "protocol/msg/follow_points.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
// #include "algorithm_manager/algorithm_task_manager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_msg_queue.hpp"
#include "protocol/srv/stop_algo_task.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "motion_action/motion_macros.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_util/lifecycle_service_client.hpp"
#include "behavior_manager/behavior_manager.hpp"
#include "protocol/msg/bms_status.hpp"

using namespace std::chrono_literals;   // NOLINT

namespace cyberdog
{
namespace algorithm
{
enum class LifecycleClientID : uint8_t
{
  kNav,
  kLaserMapping,
  kLaserLoc,
  kVisMapping,
  kVisLoc,
  kVisVo,
  kMcrUwb,
};  // enum LifecycleClientID

enum class ExecutorStatus : uint8_t
{
  kIdle = 0,
  kExecuting = 1,
  kSuccess = 2,
  kAborted = 3,
  kCanceled = 4
};  // enum ExecutorStatus

struct LifecycleClients
{
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client_get_state_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_change_state_;
};

enum class Nav2LifecycleMode : uint8_t
{
  kStartUp = 0,
  kPause = 1,
  kResume = 2,
};

using NavigationGoalHandle =
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
// using WaypointFollowerGoalHandle =
//   rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
// using NavThroughPosesGoalHandle =
//   rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>;
using TargetTrackingGoalHandle =
  rclcpp_action::ClientGoalHandle<mcr_msgs::action::TargetTracking>;
using GoalStatus = action_msgs::msg::GoalStatus;
using McrTargetTracking = mcr_msgs::action::TargetTracking;
using AlgorithmMGR = protocol::action::Navigation;
using GoalHandleAlgorithmMGR = rclcpp_action::ServerGoalHandle<AlgorithmMGR>;
using Nav2LifecyleMgrClient = nav2_lifecycle_manager::LifecycleManagerClient;
using LifecyleNav2LifecyleMgrClientMap =
  std::unordered_map<LifecycleClientID, std::shared_ptr<Nav2LifecyleMgrClient>>;
using Nav2LifecyleMgrClientMap =
  std::unordered_map<std::string, std::unordered_map<std::string,
    std::shared_ptr<Nav2LifecyleMgrClient>>>;
using LifecyleNodesMap =
  std::unordered_map<std::string, std::unordered_map<std::string, LifecycleClients>>;
using StopTaskSrv = protocol::srv::StopAlgoTask;
struct ExecutorData
{
  ExecutorStatus status;
  AlgorithmMGR::Feedback feedback;
};  // struct ExecutorData

class ExecutorInterface
{
public:
  ExecutorInterface() {}
  virtual void Start(const AlgorithmMGR::Goal::ConstSharedPtr goal) = 0;
  virtual void Stop(
    const StopTaskSrv::Request::SharedPtr request,
    StopTaskSrv::Response::SharedPtr response) = 0;
  virtual void Cancel() = 0;
};

class ExecutorBase : public ExecutorInterface, public rclcpp::Node
{
public:
  struct LifecycleNodeIndexs
  {
    std::vector<std::string> nav2_lifecycle_indexs;
    std::vector<std::string> lifecycle_indexs;
  };

  struct LifecycleNodeRef
  {
    std::string name;
    std::shared_ptr<nav2_util::LifecycleServiceClient> lifecycle_client;
  };

  explicit ExecutorBase(std::string node_name)
  : rclcpp::Node(node_name)
  {
    std::thread{std::bind(&ExecutorBase::UpdatePreparationStatus, this)}.detach();
    feedback_ = std::make_shared<AlgorithmMGR::Feedback>();
    last_feedback_ = std::make_shared<AlgorithmMGR::Feedback>();
  }
  ~ExecutorBase()
  {
    preparation_finished_ = true;
    preparation_finish_cv_.notify_one();
  }

  bool Init(
    std::function<void(const AlgorithmMGR::Feedback::SharedPtr)> feedback_callback,
    std::function<void(void)> success_callback,
    std::function<void(void)> cancle_callback,
    std::function<void(void)> abort_callback)
  {
    task_feedback_callback_ = feedback_callback;
    task_success_callback_ = success_callback;
    task_cancle_callback_ = cancle_callback;
    task_abort_callback_ = abort_callback;

    std::string task_config = ament_index_cpp::get_package_share_directory("algorithm_manager") +
      "/config/Task.toml";
    toml::value tasks;
    if (!cyberdog::common::CyberdogToml::ParseFile(task_config, tasks)) {
      FATAL("Cannot parse %s", task_config.c_str());
      return false;
    }
    if (!tasks.is_table()) {
      FATAL("Toml format error");
      return false;
    }
    toml::value values;
    cyberdog::common::CyberdogToml::Get(tasks, "task", values);
    for (size_t i = 0; i < values.size(); i++) {
      auto value = values.at(i);
      std::string task_name;
      LifecycleNodeIndexs lifecycle_ref;
      GET_TOML_VALUE(value, "TaskName", task_name);
      GET_TOML_VALUE(value, "DepsNav2LifecycleNodes", lifecycle_ref.nav2_lifecycle_indexs);
      GET_TOML_VALUE(value, "DepsLifecycleNodes", lifecycle_ref.lifecycle_indexs);
      task_map_.emplace(task_name, lifecycle_ref);
    }
    return true;
  }

protected:
  bool OperateDepsNav2LifecycleNodes(const std::string & task_name, Nav2LifecycleMode mode)
  {
    auto indexs = task_map_.at(task_name).nav2_lifecycle_indexs;
    for (auto & index : indexs) {
      if (nav2_lifecycle_clients.find(index) == nav2_lifecycle_clients.end()) {
        nav2_lifecycle_clients.emplace(
          index,
          std::make_shared<Nav2LifecyleMgrClient>(index));
      }
      auto status = nav2_lifecycle_clients.find(index)->second->is_active(
        std::chrono::nanoseconds(get_lifecycle_timeout_ * 1000 * 1000 * 1000));
      if (status == nav2_lifecycle_manager::SystemStatus::TIMEOUT) {
        ERROR("Failed to get %s state", index.c_str());
        return false;
      }
      switch (mode) {
        case Nav2LifecycleMode::kPause:
          {
            if (status == nav2_lifecycle_manager::SystemStatus::INACTIVE) {
              continue;
            }
            if (!nav2_lifecycle_clients.find(index)->second->pause()) {
              ERROR("Failed to Pause %s", index.c_str());
              return false;
            }
            INFO("Success to Pause %s", index.c_str());
          }
          break;

        case Nav2LifecycleMode::kStartUp:
          {
            if (status == nav2_lifecycle_manager::SystemStatus::ACTIVE) {
              continue;
            }
            if (!nav2_lifecycle_clients.find(index)->second->startup()) {
              ERROR("Failed to Startup %s", index.c_str());
              return false;
            }
            INFO("Success to Startup %s", index.c_str());
          }
          break;

        case Nav2LifecycleMode::kResume:
          {
            if (status == nav2_lifecycle_manager::SystemStatus::ACTIVE) {
              continue;
            }
            if (!nav2_lifecycle_clients.find(index)->second->resume()) {
              ERROR("Failed to Resume %s", index.c_str());
              return false;
            }
            INFO("Success to Resume %s", index.c_str());
          }
          break;

        default:
          break;
      }
    }
    return true;
  }
  std::vector<LifecycleNodeRef>
  GetDepsLifecycleNodes(const std::string & task_name)
  {
    // TODO(Harvey): 非Nav2Lifecycle的node管理方式
    std::vector<LifecycleNodeRef> clients;
    auto indexs = task_map_.at(task_name).lifecycle_indexs;
    for (auto index : indexs) {
      if (lifecycle_clients.find(index) == lifecycle_clients.end()) {
        lifecycle_clients.emplace(
          index,
          std::make_shared<nav2_util::LifecycleServiceClient>(index));
      }
      clients.push_back({index, lifecycle_clients.find(index)->second});
    }
    return clients;
  }

  bool ActivateDepsLifecycleNodes(const std::string & task_name, int timeout = 20000)
  {
    lifecycle_activated_.clear();
    for (auto client : GetDepsLifecycleNodes(task_name)) {
      INFO("Trying to launch [%s]", client.name.c_str());
      int lifecycle_query = 10;
      if (!client.lifecycle_client->service_exist(std::chrono::seconds(lifecycle_query))) {
        ERROR("Lifecycle [%s] not exist in %ds", client.name.c_str(), lifecycle_query);
        return false;
      }
      bool is_timeout = false;
      auto state = client.lifecycle_client->get_state(is_timeout, timeout);
      if (is_timeout) {
        ERROR("Cannot get state of [%s]", client.name.c_str());
        return false;
      }
      if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        INFO("Lifecycle [%s] already be active", client.name.c_str());
        lifecycle_activated_.push_back(client);
        continue;
      } else {
        // INFO("%s 1st: %d", client.name.c_str(), client.lifecycle_client->get_state());
        if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
          if (!client.lifecycle_client->change_state(
              lifecycle_msgs::msg::Transition::
              TRANSITION_CONFIGURE, timeout))
          {
            WARN("Get error when configuring [%s], try to active", client.name.c_str());
          }
        }
        // INFO("%s 2nd: %d", client.name.c_str(), client.lifecycle_client->get_state());
        if (!client.lifecycle_client->change_state(
            lifecycle_msgs::msg::Transition::
            TRANSITION_ACTIVATE, timeout))
        {
          ERROR("Get error when activing [%s] with state: %d", client.name.c_str(), state);
          return false;
        }
        lifecycle_activated_.push_back(client);
        INFO("Success to active [%s]", client.name.c_str());
      }
    }
    return true;
  }

  bool DeactivateDepsLifecycleNodes(const std::string & task_name, int timeout = 20000)
  {
    lifecycle_activated_ = GetDepsLifecycleNodes(task_name);
    return DeactivateDepsLifecycleNodes(timeout);
  }

  bool DeactivateDepsLifecycleNodes(int timeout = 20000, bool cleanup = false)
  {
    // for (auto client : lifecycle_activated_) {
    for (auto lifecycle_activated_rtr = lifecycle_activated_.rbegin();
      lifecycle_activated_rtr != lifecycle_activated_.rend(); lifecycle_activated_rtr++)
    {
      auto client = *lifecycle_activated_rtr;
      int lifecycle_query = 10;
      if (!client.lifecycle_client->service_exist(std::chrono::seconds(lifecycle_query))) {
        WARN(
          "Lifecycle [%s] not exist in %ds, will not deactive it",
          client.name.c_str(), lifecycle_query);
        continue;
      }
      bool is_timeout = false;
      auto state = client.lifecycle_client->get_state(is_timeout, timeout);
      if (is_timeout) {
        ERROR("Cannot get state of [%s]", client.name.c_str());
        continue;
      }
      if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        INFO("Lifecycle [%s] is unconfigured, no need to deactivate", client.name.c_str());
        continue;
      } else if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        INFO("Lifecycle [%s] already be inactive", client.name.c_str());
        continue;
      } else {
        if (!client.lifecycle_client->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, timeout))
        {
          ERROR("Get error when deactive %s", client.name.c_str());
        } else {
          INFO("Success to deactive [%s]", client.name.c_str());
        }
        if (cleanup) {
          if (client.name == std::string("vision_manager")) {
            continue;
          }
          if (!client.lifecycle_client->change_state(
              lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, timeout))
          {
            ERROR("Get error when cleanup %s", client.name.c_str());
          } else {
            INFO("Success to cleanup [%s]", client.name.c_str());
          }
        }
      }
    }
    return true;
  }


  void UpdateExecutorFeedback(const AlgorithmMGR::Feedback::SharedPtr feedback)
  {
    // INFO("Will Enqueue");
    task_feedback_callback_(feedback);
    // INFO("Over Enqueue");
  }

  void UpdatePreparationStatus()
  {
    while (rclcpp::ok()) {
      if (preparation_finished_) {
        std::unique_lock<std::mutex> lk(preparation_finish_mutex_);
        preparation_finish_cv_.wait(lk);
      }
      if (!rclcpp::ok()) {
        INFO("UpdatePreparationStatus exit");
        return;
      }
      // INFO("Peparation Report: %d", feedback_->feedback_code);
      task_feedback_callback_(feedback_);
      static uint8_t count = 0;
      if (feedback_->feedback_code != AlgorithmMGR::Feedback::TASK_PREPARATION_EXECUTING) {
        ++count;
      }
      if (count > preparation_finished_report_time_) {
        preparation_count_cv_.notify_one();
        count = 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
  /**
   * @brief
   * 更新激活依赖节点以及向底层任务执行器send_goal前的准备状态
   *
   */
  void ReportPreparationStatus()
  {
    std::unique_lock<std::mutex> lk(preparation_finish_mutex_);
    preparation_finished_ = false;
    feedback_->feedback_code = AlgorithmMGR::Feedback::TASK_PREPARATION_EXECUTING;
    preparation_finish_cv_.notify_one();
  }

  void UpdateFeedback(int32_t feedback_code)
  {
    // if (feedback_code == last_feedback_->feedback_code) {
    //   WARN(
    //     "Last Feedback: %d, new feedback: %d, will not send", last_feedback_->feedback_code,
    //     feedback_code);
    //   return;
    // }
    std::lock_guard<std::mutex> lk(feedback_mutex_);
    last_feedback_->feedback_code = feedback_code;
    feedback_->feedback_code = feedback_code;
    task_feedback_callback_(feedback_);
  }

  void SetFeedbackCode(uint32_t feedback)
  {
    feedback_->feedback_code = feedback;
  }

  /**
   * @brief
   * 结束向底层任务执行器send_goal前的状态上报
   *
   * @param feedback
   */
  void ReportPreparationFinished(uint32_t feedback)
  {
    {
      std::unique_lock<std::mutex> lk(preparation_finish_mutex_);
      feedback_->feedback_code = feedback;
      INFO("Update Last Report: %d", feedback_->feedback_code);
    }
    std::unique_lock<std::mutex> lk(preparation_count_mutex_);
    preparation_count_cv_.wait(lk);
    StopReportPreparationThread();
  }
  void StopReportPreparationThread()
  {
    if (preparation_finished_) {
      return;
    }
    std::unique_lock<std::mutex> lk(preparation_finish_mutex_);
    preparation_finished_ = true;
  }
  std::shared_ptr<BehaviorManager>
  GetBehaviorManager()
  {
    if (behavior_manager_ == nullptr) {
      behavior_manager_ = std::make_shared<BehaviorManager>("behavior_manager");
    }
    return behavior_manager_;
  }
  static std::unordered_map<std::string,
    std::shared_ptr<Nav2LifecyleMgrClient>> nav2_lifecycle_clients;
  static std::unordered_map<std::string,
    std::shared_ptr<nav2_util::LifecycleServiceClient>> lifecycle_clients;
  static std::unordered_map<std::string, LifecycleNodeIndexs> task_map_;
  static constexpr uint8_t preparation_finished_report_time_ = 0;  // count
  std::chrono::milliseconds server_timeout_{2000};
  rclcpp::Node::SharedPtr action_client_node_;
  AlgorithmMGR::Feedback::SharedPtr feedback_;
  AlgorithmMGR::Feedback::SharedPtr last_feedback_;
  std::function<void(const AlgorithmMGR::Feedback::SharedPtr)> task_feedback_callback_;
  std::function<void(void)> task_success_callback_;
  std::function<void(void)> task_cancle_callback_;
  std::function<void(void)> task_abort_callback_;
  uint32_t get_lifecycle_timeout_{10};

private:
  std::mutex preparation_count_mutex_;
  std::mutex preparation_finish_mutex_;
  std::mutex feedback_mutex_;
  std::condition_variable preparation_count_cv_;
  std::condition_variable preparation_finish_cv_;
  std::vector<LifecycleNodeRef> lifecycle_activated_{};
  static std::shared_ptr<BehaviorManager> behavior_manager_;
  bool preparation_finished_{true};


  /* add by North.D.K. 10.09*/
  // std::function<void()>
};   // class ExecutorBase
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_BASE_HPP_
