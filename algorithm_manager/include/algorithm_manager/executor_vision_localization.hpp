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


#ifndef ALGORITHM_MANAGER__EXECUTOR_VISION_LOCALIZATION_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_VISION_LOCALIZATION_HPP_

#include <string>
#include <memory>

#include "algorithm_manager/executor_base.hpp"
#include "algorithm_manager/lifecycle_node_manager.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cyberdog_visions_interfaces/srv/miloc_map_handler.hpp"

namespace cyberdog
{
namespace algorithm
{

class ExecutorVisionLocalization : public ExecutorBase
{
public:
  using LifeCycleNodeType = LifecycleNodeManager::LifeCycleNode;
  using MapAvailableResult = cyberdog_visions_interfaces::srv::MilocMapHandler;

  /**
   * @brief Construct a new Executor Laser Localization object
   *
   * @param node_name Executor node name
   */
  explicit ExecutorVisionLocalization(std::string node_name);

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

  /**
   * @brief Check curent map building available
   *
   * @return true Return success
   * @return false Return failure
   */
  bool CheckMapAvailable();

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

  // service client
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr start_client_ {nullptr};
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stop_client_ {nullptr};
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr realtime_pose_client_ {nullptr};

  // Subscription lidar localization topic result
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr relocalization_sub_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_trigger_sub_{nullptr};

  // Get vision build map available result
  std::shared_ptr<nav2_util::ServiceClient<MapAvailableResult>> map_result_client_ {nullptr};

  // Record relocalization result
  bool relocalization_success_ {false};
  bool relocalization_failure_ {false};
};  // class ExecutorLaserLocalization
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_VISION_LOCALIZATION_HPP_
