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
using RealSenseClient = RealSenseLifecycleServiceClient;

class ExecutorInterface
{
public:
  enum class ExecutorStatus : uint8_t
  {
    kIdle = 0,
    kExecuting = 1,
    kSuccess = 2,
    kAborted = 3,
    kCanceled = 4
  };

public:
  ExecutorInterface() {}
  virtual void Start(const AlgorithmMGR::Goal::ConstSharedPtr goal) = 0;
  virtual void Stop() = 0;
  virtual void Cancel() = 0;
};

class ExecutorBase : public ExecutorInterface, public rclcpp::Node
{
public:
  struct ExecutorData
  {
    ExecutorStatus status;
    AlgorithmMGR::Feedback feedback;
  };

  explicit ExecutorBase(std::string node_name)
  : rclcpp::Node(node_name),
    client_nav_("lifecycle_manager_navigation"),
    client_loc_("lifecycle_manager_localization")
  {
    executor_data_future_ = executor_data_promise_.get_future();
  }
  virtual ExecutorData & GetStatus() final
  {
    std::unique_lock<std::mutex> lk(executor_data_mutex_);
    executor_data_cv_.wait(lk);
    return executor_data_;
  }

protected:
  virtual void UpdateExecutorData(const ExecutorData & executor_data) final
  {
    executor_data_ = executor_data;
    std::unique_lock<std::mutex> lk(executor_data_mutex_);
    executor_data_cv_.notify_all();
  }
  virtual bool LaunchNav2LifeCycleNode(
    nav2_lifecycle_manager::LifecycleManagerClient & node) final
  {
    if (node.is_active() == nav2_lifecycle_manager::SystemStatus::ACTIVE) {
      return true;
    }
    if (!node.startup()) {
      return false;
    }
    return true;
  }
  nav2_lifecycle_manager::LifecycleManagerClient client_nav_;
  nav2_lifecycle_manager::LifecycleManagerClient client_loc_;
  std::shared_ptr<RealSenseClient> client_realsense_{nullptr};
  std::chrono::milliseconds server_timeout_{2000};
  rclcpp::Node::SharedPtr action_client_node_;

private:
  std::mutex executor_data_mutex_;
  std::condition_variable executor_data_cv_;
  ExecutorData executor_data_;
  std::promise<ExecutorData> executor_data_promise_;
  std::future<ExecutorData> executor_data_future_;
};   // class ExecutorBase
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_BASE_HPP_
