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
#include "rclcpp/rclcpp.hpp"
#include "protocol/action/navigation.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "mcr_msgs/action/target_tracking.hpp"
// #include "nav2_util/geometry_utils.hpp"
// #include "protocol/msg/follow_points.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "algorithm_manager/realsense_lifecycle_manager.hpp"
// #include "algorithm_manager/algorithm_task_manager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_msg_queue.hpp"

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

using NavigationGoalHandle =
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
using WaypointFollowerGoalHandle =
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
using NavThroughPosesGoalHandle =
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>;
using TargetTrackingGoalHandle =
  rclcpp_action::ClientGoalHandle<mcr_msgs::action::TargetTracking>;
using GoalStatus = action_msgs::msg::GoalStatus;
using McrTargetTracking = mcr_msgs::action::TargetTracking;
using AlgorithmMGR = protocol::action::Navigation;
using GoalHandleAlgorithmMGR = rclcpp_action::ServerGoalHandle<AlgorithmMGR>;
using Nav2LifecyleMgrClient = nav2_lifecycle_manager::LifecycleManagerClient;
using LifecyleNav2LifecyleMgrClientMap =
  std::unordered_map<LifecycleClientID, std::shared_ptr<Nav2LifecyleMgrClient>>;
using RealSenseClient = RealSenseLifecycleServiceClient;

struct ExecutorData
{
  ExecutorStatus status;
  AlgorithmMGR::Feedback feedback;
};  // struct ExecutorData
class ExecutorInterface
{
public:
  ExecutorInterface() {}
  virtual bool Start(const AlgorithmMGR::Goal::ConstSharedPtr goal) = 0;
  virtual void Cancel() = 0;
};

class ExecutorBase : public ExecutorInterface, public rclcpp::Node
{
public:
  explicit ExecutorBase(std::string node_name)
  : rclcpp::Node(node_name)
  {
    lifecycle_client_ids_.emplace(
      LifecycleClientID::kNav,
      "lifecycle_manager_navigation");
    lifecycle_client_ids_.emplace(
      LifecycleClientID::kLaserMapping,
      "lifecycle_manager_laser_mapping");
    lifecycle_client_ids_.emplace(
      LifecycleClientID::kLaserLoc,
      "lifecycle_manager_laser_loc");
    lifecycle_client_ids_.emplace(
      LifecycleClientID::kVisMapping,
      "lifecycle_manager_vis_mapping");
    lifecycle_client_ids_.emplace(
      LifecycleClientID::kVisLoc,
      "lifecycle_manager_vis_loc");
    lifecycle_client_ids_.emplace(
      LifecycleClientID::kVisVo,
      "lifecycle_manager_vis_vo");
    lifecycle_client_ids_.emplace(
      LifecycleClientID::kMcrUwb,
      "lifecycle_manager_mcr_uwb");
    std::thread{std::bind(&ExecutorBase::UpdatePreparationStatus, this)}.detach();
  }
  virtual void Stop() {}
  ~ExecutorBase()
  {
    preparation_finished_ = true;
    preparation_finish_cv_.notify_one();
  }
  /**
   * @brief 
   * 向任务管理器提供查询当前任务执行器的状态数据，包括执行阶段和反馈数据，
   * 其中反馈数据包含了激活依赖节点的状态和底层执行器的反馈数据
   *
   * @return ExecutorData&
   */
  ExecutorData & GetExecutorData()
  {
    // INFO("queue size before: %d", executor_data_queue_.Size());
    executor_data_queue_.DeQueue(executor_data_);
    // INFO("queue size after: %d", executor_data_queue_.Size());
    return executor_data_;
  }
  static void RegisterUpdateImpl(std::function<void(const ExecutorData &)> f)
  {
    if (f != nullptr) {
      update_executor_f_ = f;
    }
  }

protected:
  /**
   * @brief
   * 获取依赖节点的LifecycleMgr的客户端对象
   *
   * @param id
   * @return std::shared_ptr<Nav2LifecyleMgrClient>
   */
  static std::shared_ptr<Nav2LifecyleMgrClient> GetNav2LifecycleMgrClient(
    const LifecycleClientID & id)
  {
    if (lifecycle_client_map_.find(id) == lifecycle_client_map_.end()) {
      lifecycle_client_map_[id] =
        std::make_shared<Nav2LifecyleMgrClient>(lifecycle_client_ids_.at(id));
    }
    return lifecycle_client_map_.at(id);
  }
  /**
   * @brief
   * 获取Realsense的LifecycleMgr的客户端对象
   *
   * @return std::shared_ptr<RealSenseClient>
   */
  static std::shared_ptr<RealSenseClient> GetRealsenseLifecycleMgrClient()
  {
    if (lifecycle_client_realsense_ == nullptr) {
      lifecycle_client_realsense_ = std::make_shared<RealSenseClient>("realsense_client");
    }
    return lifecycle_client_realsense_;
  }
  /**
   * @brief
   * 更新当前任务执行器的状态数据，所有继承的子类中在需要上报状态时调用该接口
   *
   * @param executor_data
   */
  void UpdateExecutorData(const ExecutorData & executor_data)
  {
    // INFO("Will Enqueue");
    // executor_data_queue_.EnQueueOne(executor_data);
    update_executor_f_(executor_data);
    // INFO("Over Enqueue");
  }
  /**
   * @brief
   * 激活依赖的Lifecycle节点
   *
   * @param node
   * @return true
   * @return false
   */
  bool LaunchNav2LifeCycleNode(
    std::shared_ptr<Nav2LifecyleMgrClient> node)
  {
    auto status = node->is_active(
      std::chrono::nanoseconds(get_lifecycle_timeout_ * 1000 * 1000 * 1000));
    if (status == nav2_lifecycle_manager::SystemStatus::TIMEOUT) {
      ERROR("Failed to get lifecycle state");
      return false;
    } else if (status == nav2_lifecycle_manager::SystemStatus::ACTIVE) {
      return true;
    }
    if (!node->startup()) {
      ERROR("Failed to launch lifecycle");
      return false;
    }
    return true;
  }
  /**
   * @brief
   * 更新任务开始后，在向底层执行器send_goal之前的状态上报线程
   *
   */
  void UpdatePreparationStatus()
  {
    ExecutorData executor_uwb_tracking_data;
    executor_uwb_tracking_data.status = ExecutorStatus::kExecuting;
    while (rclcpp::ok()) {
      if (preparation_finished_) {
        std::unique_lock<std::mutex> lk(preparation_finish_mutex_);
        preparation_finish_cv_.wait(lk);
      }
      if (!rclcpp::ok()) {
        INFO("UpdatePreparationStatus exit");
        return;
      }
      INFO("Peparation Report: %d", feedback_);
      executor_uwb_tracking_data.feedback.feedback_code = feedback_;
      UpdateExecutorData(executor_uwb_tracking_data);
      static uint8_t count = 0;
      if (feedback_ != AlgorithmMGR::Feedback::TASK_PREPARATION_EXECUTING) {
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
    feedback_ = AlgorithmMGR::Feedback::TASK_PREPARATION_EXECUTING;
    preparation_finish_cv_.notify_one();
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
      feedback_ = feedback;
      INFO("Update Last Report: %d", feedback_);
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
  static LifecyleNav2LifecyleMgrClientMap lifecycle_client_map_;
  static std::unordered_map<LifecycleClientID, std::string> lifecycle_client_ids_;
  static std::shared_ptr<RealSenseClient> lifecycle_client_realsense_;
  static constexpr uint8_t preparation_finished_report_time_ = 5;  // count
  std::chrono::milliseconds server_timeout_{2000};
  rclcpp::Node::SharedPtr action_client_node_;
  uint32_t feedback_;
  uint32_t get_lifecycle_timeout_{10};

private:
  std::mutex executor_data_mutex_;
  std::mutex preparation_count_mutex_;
  std::mutex preparation_finish_mutex_;
  std::condition_variable executor_data_cv_;
  std::condition_variable preparation_count_cv_;
  std::condition_variable preparation_finish_cv_;
  ExecutorData executor_data_;
  common::MsgQueue<ExecutorData> executor_data_queue_;
  static std::function<void(const ExecutorData &)> update_executor_f_;
  bool preparation_finished_{true};
};   // class ExecutorBase
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_BASE_HPP_
