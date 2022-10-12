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

#ifndef ALGORITHM_MANAGER__EXECUTOR_VISION_MAPPING_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_VISION_MAPPING_HPP_

#include <string>
#include <memory>

#include "algorithm_manager/executor_base.hpp"
#include "algorithm_manager/lifecycle_node_manager.hpp"
#include "visualization/srv/stop.hpp"

namespace cyberdog
{
namespace algorithm
{

class ExecutorVisionMapping : public ExecutorBase
{
public:
  explicit ExecutorLaserMapping(std::string node_name);
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

  // feedback data
  ExecutorData executor_laser_mapping_data_;

  // // Control lidar mapping lifecycle(Nav2 lifecycle)
  // nav2_lifecycle_manager::LifecycleManagerClient client_mapping_{nullptr};

  // service client
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr start_client_ {nullptr};
  rclcpp::Client<visualization::srv::Stop>::SharedPtr stop_client_ {nullptr};
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr realtime_pose_client_ {nullptr};

  // Control realsense camera lifecycle
  std::shared_ptr<LifecycleNodeManager> realsense_lifecycle_ {nullptr};

  // realtime robot pose
  bool start_report_realtime_pose_ {false};
};  // class ExecutorVisionMapping
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_VISION_MAPPING_HPP_
