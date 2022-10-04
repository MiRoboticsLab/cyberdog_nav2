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
  }
  ExecutorData & GetExecutorData()
  {
    std::unique_lock<std::mutex> lk(executor_data_mutex_);
    executor_data_cv_.wait(lk);
    // INFO("Report: %d", executor_data_.feedback.feedback_code);
    return executor_data_;
  }
  static std::shared_ptr<Nav2LifecyleMgrClient> GetNav2LifecycleMgrClient(
    const LifecycleClientID & id)
  {
    if (lifecycle_client_map_.find(id) == lifecycle_client_map_.end()) {
      lifecycle_client_map_[id] =
        std::make_shared<Nav2LifecyleMgrClient>(lifecycle_client_ids_.at(id));
    }
    return lifecycle_client_map_.at(id);
  }
  static std::shared_ptr<RealSenseClient> GetRealsenseLifecycleMgrClient()
  {
    if (lifecycle_client_realsense_ == nullptr) {
      lifecycle_client_realsense_ = std::make_shared<RealSenseClient>("realsense_client");
    }
    return lifecycle_client_realsense_;
  }

protected:
  void UpdateExecutorData(const ExecutorData & executor_data)
  {
    executor_data_ = executor_data;
    std::unique_lock<std::mutex> lk(executor_data_mutex_);
    executor_data_cv_.notify_all();
    // INFO("Update: %d", executor_data_.feedback.feedback_code);
  }
  bool LaunchNav2LifeCycleNode(
    std::shared_ptr<Nav2LifecyleMgrClient> node)
  {
    if (node->is_active() == nav2_lifecycle_manager::SystemStatus::ACTIVE) {
      return true;
    }
    if (!node->startup()) {
      return false;
    }
    return true;
  }
  void ReportPreparationStatus()
  {
    preparation_finished_ = false;
    feedback_ = AlgorithmMGR::Feedback::TASK_PREPARATION_EXECUTING;
    std::thread t = std::thread([this](){
      ExecutorData executor_uwb_tracking_data;
      executor_uwb_tracking_data.status = ExecutorStatus::kExecuting;
      do {
        // INFO("preparation: %d", feedback_);
        executor_uwb_tracking_data.feedback.feedback_code = feedback_;
        UpdateExecutorData(executor_uwb_tracking_data);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        static uint8_t count = 0;
        if (feedback_ != AlgorithmMGR::Feedback::TASK_PREPARATION_EXECUTING) {
            ++count;
        }
        if (count > preparation_finished_report_time_) {
          this->preparation_cv_.notify_one();
          count = 0;
        }
      } while (!this->preparation_finished_);
    }); 
    t.detach();   
  }
  void ReportPreparationFinished(uint32_t feedback)
  { 
    feedback_ = feedback;
    std::unique_lock<std::mutex> lk(preparation_mutex_);
    preparation_cv_.wait(lk);
    StopReportPreparationThread();
  }
  void StopReportPreparationThread() { preparation_finished_ = true; }
  static LifecyleNav2LifecyleMgrClientMap lifecycle_client_map_;
  static std::unordered_map<LifecycleClientID, std::string> lifecycle_client_ids_;
  static std::shared_ptr<RealSenseClient> lifecycle_client_realsense_;
  static constexpr uint8_t preparation_finished_report_time_ = 5;
  std::chrono::milliseconds server_timeout_{2000};
  rclcpp::Node::SharedPtr action_client_node_;
  uint32_t feedback_;

private:
  std::mutex executor_data_mutex_, preparation_mutex_;
  std::condition_variable executor_data_cv_, preparation_cv_;
  ExecutorData executor_data_;
  bool preparation_finished_{false};
};   // class ExecutorBase
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_BASE_HPP_
