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

#ifndef ALGORITHM_MANAGER__EXECUTOR_LASER_LOCALIZATION_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_LASER_LOCALIZATION_HPP_

#include <string>
#include <memory>

#include "algorithm_manager/executor_base.hpp"
#include "algorithm_manager/lifecycle_node_manager.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "algorithm_manager/timer.hpp"

namespace cyberdog
{
namespace algorithm
{

class ExecutorLaserLocalization : public ExecutorBase
{
public:
  using LifeCycleNodeType = LifecycleNodeManager::LifeCycleNode;

  enum class LocationStatus
  {
    Unknown,
    SUCCESS,
    FAILURE
  };

  /**
   * @brief Construct a new Executor Laser Localization object
   *
   * @param node_name Executor node name
   */
  explicit ExecutorLaserLocalization(std::string node_name);

  /**
  * @brief Start lidar localization
  *
  * @param goal APP or rviz set target goal
  */
  void Start(const AlgorithmMGR::Goal::ConstSharedPtr goal) override;

  /**
   * @brief Handle APP set stop lidar localization
   *
   * @param request The GRPC service request command
   * @param response The GRPC service response result
   */
  void Stop(
    const StopTaskSrv::Request::SharedPtr request,
    StopTaskSrv::Response::SharedPtr response) override;

  /**
   * @brief Cancel lidar localization, it't can debug ros command
   */
  void Cancel() override;

private:
  /**
   * Handle location request status
   */
  void HandleLocationServiceCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  /**
   * @brief Handle lidar relocalization result
   *
   * @param msg The lidar relocalization result
   */
  void HandleRelocalizationCallback(const std_msgs::msg::Int32::SharedPtr msg);

  /**
   * @brief Handle some request stop location module
   *
   * @param msg Request command
   */
  void HandleStopTriggerCommandMessages(const std_msgs::msg::Bool::SharedPtr msg);

  /**
  * @brief Check `camera/camera` real sense sensor status
  *
  * @return true Return success
  * @return false Return failure
  */
  bool IsDependsReady();

  /**
   * @brief Wait relocalization result is success
   *
   * @param timeout Wait time
   * @return true Return success
   * @return false Return failure
   */
  bool WaitRelocalization(std::chrono::seconds timeout);

  /**
   * @brief Enable Lidar Relocalization turn on
   *
   * @return true Return success
   * @return false Return failure
   */
  bool EnableRelocalization();

  /**
   * @brief Enable Lidar Relocalization turn off
   *
   * @return true Return success
   * @return false Return failure
   */
  bool DisenableRelocalization();

  /**
   * @brief Turn on ot turn off report realtime robot pose
   *
   * @param enable True enable report, false disenable report
   * @return true Return success
   * @return false Return failure
   */
  bool EnableReportRealtimePose(bool enable);

  void StopLocalization();

  /**
   * @brief Activate all nav2 lifecycle nodes
   *
   * @return true Return success
   * @return false Return failure
   */
  bool ActivateAllNavigationLifecycleNodes();

  /**
   * @brief Check current `pose server` in activate state
   * 
   * @return true Return success
   * @return false Return failure
   */
  bool CheckPoseServerActivate();

  /**
   * @brief Set all lifecycle default state
   *
   * @return true Return success
   * @return false Return failure
   */
  bool ResetLifecycleDefaultValue();

  // feedback data
  ExecutorData executor_laser_mapping_data_;

  // Control localization_node lifecycle
  std::shared_ptr<LifecycleController> localization_lifecycle_ {nullptr};

  // std::unique_ptr<nav2_lifecycle_manager::LifecycleManagerClient> localization_client_ {nullptr};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr location_status_service_ {nullptr};

  // service client
  std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> start_client_ {nullptr};
  std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> stop_client_ {nullptr};
  std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> realtime_pose_client_ {nullptr};
  std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> pose_server_client_ {nullptr};


  // Subscription lidar localization topic result
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr relocalization_sub_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_trigger_sub_{nullptr};

  // Record relocalization result
  bool relocalization_success_ {false};
  bool relocalization_failure_ {false};

  //  LocationStatus location_status_;
  LocationStatus location_status_;

  // in service
  bool is_activate_ {false};
  bool location_stop_function_starting_ {false};
};  // class ExecutorLaserLocalization
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_LASER_LOCALIZATION_HPP_
