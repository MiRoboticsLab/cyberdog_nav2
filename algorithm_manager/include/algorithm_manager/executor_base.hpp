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

#ifndef ALGORITHM_MANAGER__EXECUTOR_BASE_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_BASE_HPP_

#include <memory>
#include <unordered_map>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "protocol/action/navigation.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
// #include "nav2_msgs/action/follow_waypoints.hpp"
// #include "nav2_msgs/action/navigate_through_poses.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
#include "mcr_msgs/action/target_tracking.hpp"
// #include "nav2_util/geometry_utils.hpp"
// #include "protocol/msg/follow_points.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "algorithm_manager/realsense_lifecycle_manager.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
// #include "algorithm_manager/algorithm_task_manager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_msg_queue.hpp"
#include "protocol/srv/stop_algo_task.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "motion_action/motion_macros.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

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
using RealSenseClient = RealSenseLifecycleServiceClient;
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
  struct LifecycleNodeRef
  {
    std::unordered_map<std::string, std::shared_ptr<Nav2LifecyleMgrClient>> nav2_lifecycle_clients;
    std::unordered_map<std::string, LifecycleClients> lifecycle_nodes;
  };

  explicit ExecutorBase(std::string node_name)
  : rclcpp::Node(node_name)
  {
    std::thread{std::bind(&ExecutorBase::UpdatePreparationStatus, this)}.detach();
    feedback_ = std::make_shared<AlgorithmMGR::Feedback>();
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
      LifecycleNodeRef lifecycle_ref;
      GET_TOML_VALUE(value, "TaskName", task_name);
      std::vector<std::string> temp;
      GET_TOML_VALUE(value, "DepsNav2LifecycleNodes", temp);
      for (auto s : temp) {
        lifecycle_ref.nav2_lifecycle_clients.emplace(s, nullptr);
      }
      GET_TOML_VALUE(value, "DepsLifecycleNodes", temp);
      for (auto s : temp) {
        lifecycle_ref.lifecycle_nodes.emplace(s, LifecycleClients{nullptr, nullptr});
      }
      task_map_.emplace(task_name, lifecycle_ref);
    }
    return true;
  }

protected:
  bool OperateDepsNav2LifecycleNodes(const std::string & task_name, Nav2LifecycleMode mode)
  {
    auto clients = task_map_.at(task_name).nav2_lifecycle_clients;
    for (auto & client : clients) {
      if (client.second == nullptr) {
        client.second = std::make_shared<Nav2LifecyleMgrClient>(client.first);
      }
      auto status = client.second->is_active(
        std::chrono::nanoseconds(get_lifecycle_timeout_ * 1000 * 1000 * 1000));
      if (status == nav2_lifecycle_manager::SystemStatus::TIMEOUT) {
        ERROR("Failed to get Nav2LifecycleNode %s state", client.first.c_str());
        return false;
      }
      switch (mode) {
        case Nav2LifecycleMode::kPause:
          {
            if (status == nav2_lifecycle_manager::SystemStatus::INACTIVE) {
              continue;
            }
            if (!client.second->pause()) {
              ERROR("Failed to Pause Nav2LifecycleNode %s", client.first.c_str());
              return false;
            }
          }
          break;

        case Nav2LifecycleMode::kStartUp:
          {
            if (status == nav2_lifecycle_manager::SystemStatus::ACTIVE) {
              continue;
            }
            if (!client.second->startup()) {
              ERROR("Failed to Startup av2LifecycleNode %s", client.first.c_str());
              return false;
            }
          }
          break;

        case Nav2LifecycleMode::kResume:
          {
            if (status == nav2_lifecycle_manager::SystemStatus::ACTIVE) {
              continue;
            }
            if (client.second->resume()) {
              ERROR("Failed to Resume Nav2LifecycleNode %s", client.first.c_str());
              return false;
            }
          }
          break;

        default:
          break;
      }
    }
    return true;
  }

  bool OperateDepsLifecycleNodes(const std::string & task_name)
  {
    // TODO(Harvey): 非Nav2Lifecycle的node管理方式
    auto clients = task_map_.at(task_name).lifecycle_nodes;
    for (auto client : clients) {
      if (client.second.client_change_state_ == nullptr ||
        client.second.client_get_state_ == nullptr)
      {
        client.second.client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
          client.first);
        client.second.client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
          client.first);
      }
    }
    // TODO(Harvey): configure、activate节点，deactive、unconfigure节点
    return true;
  }

  static std::shared_ptr<RealSenseClient> GetRealsenseLifecycleMgrClient()
  {
    if (lifecycle_client_realsense_ == nullptr) {
      lifecycle_client_realsense_ = std::make_shared<RealSenseClient>("realsense_client");
    }
    return lifecycle_client_realsense_;
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
      INFO("Peparation Report: %d", feedback_->feedback_code);
      task_feedback_callback_(feedback_);
      static uint8_t count = 0;
      if (feedback_->feedback_code != AlgorithmMGR::Feedback::TASK_PREPARATION_EXECUTING) {
        ++count;
      }
      if (count > preparation_finished_report_time_) {
        preparation_count_cv_.notify_one();
        count = 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
    std::unique_lock<std::mutex> lk(preparation_finish_mutex_);
    preparation_finished_ = true;
  }
  static std::unordered_map<std::string, LifecycleNodeRef> task_map_;
  static std::shared_ptr<RealSenseClient> lifecycle_client_realsense_;
  static constexpr uint8_t preparation_finished_report_time_ = 5;  // count
  std::chrono::milliseconds server_timeout_{2000};
  rclcpp::Node::SharedPtr action_client_node_;
  AlgorithmMGR::Feedback::SharedPtr feedback_;
  std::function<void(const AlgorithmMGR::Feedback::SharedPtr)> task_feedback_callback_;
  std::function<void(void)> task_success_callback_;
  std::function<void(void)> task_cancle_callback_;
  std::function<void(void)> task_abort_callback_;
  uint32_t get_lifecycle_timeout_{10};

private:
  std::mutex preparation_count_mutex_;
  std::mutex preparation_finish_mutex_;
  std::condition_variable preparation_count_cv_;
  std::condition_variable preparation_finish_cv_;
  bool preparation_finished_{true};

  /* add by North.D.K. 10.09*/
  // std::function<void()>
};   // class ExecutorBase
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_BASE_HPP_