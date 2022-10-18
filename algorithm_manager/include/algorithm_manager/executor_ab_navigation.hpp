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

#ifndef ALGORITHM_MANAGER__EXECUTOR_AB_NAVIGATION_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_AB_NAVIGATION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "algorithm_manager/executor_base.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "algorithm_manager/lifecycle_controller.hpp"
namespace cyberdog
{
namespace algorithm
{

class ExecutorAbNavigation : public ExecutorBase
{
public:
  explicit ExecutorAbNavigation(std::string node_name);
  ~ExecutorAbNavigation();

  /**
   * @brief Start Navigation AB
   *
   * @param goal APP or rviz set target pose goal
   */
  void Start(const AlgorithmMGR::Goal::ConstSharedPtr goal) override;

  /**
   * @brief Handle APP set stop Navigation AB
   *
   * @param request The
   * @param response The
   */
  void Stop(
    const StopTaskSrv::Request::SharedPtr request,
    StopTaskSrv::Response::SharedPtr response) override;

  /**
   * @brief Cancel Navigation AB, it't can debug ros command
   */
  void Cancel() override;

private:
  /**
   * @brief Handle `nav_action_client_` action client response callback function
   *
   * @param goal_handle
   */
  void HandleGoalResponseCallback(NavigationGoalHandle::SharedPtr goal_handle);

  /**
   * @brief Handle `nav_action_client_` action client feedback callback function
   *
   * @param feedback
   */
  void HandleFeedbackCallback(
    NavigationGoalHandle::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);

  /**
   * @brief Handle `nav_action_client_` action client result callback function
   *
   * @param goal_handle
   */
  void HandleResultCallback(const NavigationGoalHandle::WrappedResult result);

  /**
   * @brief Check `lifecycle_manager_navigation` and `lifecycle_manager_localization`
   * `real sense` sensor lidar status
   *
   * @return true Return success
   * @return false Return failure
   */
  bool IsDependsReady();

  /**
   * @brief Check action connect server
   *
   * @return true Return success
   * @return false Return failure
   */
  bool IsConnectServer();

  /**
   * @brief Check current given pose is available
   *
   * @param pose Target pose goal
   * @return true  Return success
   * @return false  Return failure
   */
  bool IsLegal(const AlgorithmMGR::Goal::ConstSharedPtr goal);

  /**
   * @brief Send APP or rviz set target pose goal
   *
   * @param pose Target pose goal
   * @return true Return success
   * @return false Return failure
   */
  bool SendGoal(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Normalized app given pose
   *
   * @param pose
   */
  void NormalizedGoal(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Sources reset and cleanup
   *
   * @return true Success
   * @return false Failure
   */
  bool ReinitializeAndCleanup();

  /**
   * @brief Reinitialize all lifecycle nodes
   * 
   * @return true Success
   * @return false Failure
   */
  bool LifecycleNodesReinitialize();

  /**
   * @brief Print target goal pose
   *
   * @param pose APP or rviz set target pose goal
   */
  void Debug2String(const geometry_msgs::msg::PoseStamped & pose);

  // feedback data
  ExecutorData executor_nav_ab_data_;

  // Navigation lifecycles
  std::unordered_map<std::string, std::shared_ptr<LifecycleController>> navigation_lifecycle_;

  // navigation target goal
  nav2_msgs::action::NavigateToPose::Goal target_goal_;

  // nav client as request
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_ {nullptr};

  // navigation goal handle
  NavigationGoalHandle::SharedPtr nav_goal_handle_ {nullptr};

  // Lifecycle controller
  std::unique_ptr<nav2_lifecycle_manager::LifecycleManagerClient> nav_client_ {nullptr};

  // Control localization_node lifecycle
  // std::shared_ptr<LifecycleController> localization_lifecycle_ {nullptr};

  // Control `map server` lifecycle node
  std::shared_ptr<LifecycleController> map_server_lifecycle_ {nullptr};

  // all depend is ready
  bool lifecycle_depend_ready_ {false};
};  // class ExecutorAbNavigation
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_AB_NAVIGATION_HPP_
