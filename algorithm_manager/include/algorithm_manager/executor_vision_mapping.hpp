//
// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#ifndef ALGORITHM_MANAGER__EXECUTOR_VISION_MAPPING_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_VISION_MAPPING_HPP_

#include <string>
#include <memory>

#include "std_msgs/msg/bool.hpp"
#include "algorithm_manager/executor_base.hpp"
#include "algorithm_manager/lifecycle_node_manager.hpp"
#include "visualization/srv/stop.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "cyberdog_visions_interfaces/srv/miloc_map_handler.hpp"
#include "algorithm_manager/timer.hpp"

namespace cyberdog
{
namespace algorithm
{

class ExecutorVisionMapping : public ExecutorBase
{
public:
  using LifeCycleNodeType = LifecycleNodeManager::LifeCycleNode;
  using MotionServiceCommand = protocol::srv::MotionResultCmd;
  using MapAvailableResult = cyberdog_visions_interfaces::srv::MilocMapHandler;

  explicit ExecutorVisionMapping(std::string node_name);
  void Start(AlgorithmMGR::Goal::ConstSharedPtr goal) override;
  void Stop(
    const StopTaskSrv::Request::SharedPtr request,
    StopTaskSrv::Response::SharedPtr response) override;
  void Cancel() override;

private:
  /**
   * @brief Check `camera/camera` real sense sensor status
   * and RGB-D sensor
   *
   * @return true Return success
   * @return false Return failure
   */
  bool IsDependsReady();

  /**
   * @brief Vision start build mapping
   *
   * @return true Return success
   * @return false Return failure
   */
  bool StartBuildMapping();

  /**
   * @brief Vision stop build mapping
   *
   * @param map_filename Set Vision save map filename
   * @return true Return success
   * @return false Return failure
   */
  bool StopBuildMapping(const std::string & map_filename);

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
  bool CheckBuildMappingAvailable();

  /**
   * @brief When robot mapping it's should walk smoother
   *
   * @return true Return success
   * @return false Return failure
   */
  bool VelocitySmoother();

  /**
   * @brief Vision is mapping
   */
  void PublishBuildMapType();

  /**
  * @brief Set all lifecycle default state
  *
  * @return true Return success
  * @return false Return failure
  */
  bool ResetLifecycleDefaultValue();

  // feedback data
  ExecutorData executor_laser_mapping_data_;

  // Control mivinsmapping lifecycle
  std::shared_ptr<LifecycleController> mapping_client_ {nullptr};

  // service client
  // rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr start_client_ {nullptr};
  // rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stop_client_ {nullptr};
  // rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr realtime_pose_client_ {nullptr};

  std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> start_client_ {nullptr};
  std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> stop_client_ {nullptr};
  std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> realtime_pose_client_ {nullptr};

  // velocity smoother 'velocity_adaptor_gait'
  std::shared_ptr<nav2_util::ServiceClient<MotionServiceCommand>> velocity_smoother_ {nullptr};

  // Get vision build map available result
  std::shared_ptr<nav2_util::ServiceClient<MapAvailableResult>> mapping_available_client_ {nullptr};

  // vision mapping alive
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_mapping_trigger_pub_{nullptr};
};  // class ExecutorVisionMapping
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_VISION_MAPPING_HPP_
