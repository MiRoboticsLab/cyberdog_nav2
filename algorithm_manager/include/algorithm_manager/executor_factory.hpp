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
#ifndef ALGORITHM_MANAGER__EXECUTOR_FACTORY_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_FACTORY_HPP_

#include <memory>
#include <string>
#include "cyberdog_common/cyberdog_log.hpp"
#include "algorithm_manager/executor_ab_navigation.hpp"
#include "algorithm_manager/executor_poses_through_navigation.hpp"
#include "algorithm_manager/executor_auto_dock.hpp"
#include "algorithm_manager/executor_vision_localization.hpp"
#include "algorithm_manager/executor_laser_localization.hpp"
#include "algorithm_manager/executor_laser_mapping.hpp"
#include "algorithm_manager/executor_vision_mapping.hpp"
#include "algorithm_manager/executor_uwb_tracking.hpp"
#include "algorithm_manager/executor_vision_tracking.hpp"
#include "algorithm_manager/executor_reset_nav.hpp"
#include "algorithm_manager/executor_base.hpp"

namespace cyberdog
{
namespace algorithm
{
/**
 * @brief Create a Executor object
 *        1. name should be valid
 *        2. return variable should be check nullptr
 * @param algorithm_type executor type by design
 * @return std::shared_ptr<cyberdog::algorithm::ExecutorBase>
 */
std::shared_ptr<cyberdog::algorithm::ExecutorBase> CreateExecutor(
  uint8_t algorithm_type,
  bool out_door,
  const std::string & task_name)
{
  std::shared_ptr<cyberdog::algorithm::ExecutorBase> result = nullptr;
  switch (algorithm_type) {
    case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_MAPPING:
      if (out_door) {
        result = std::make_shared<ExecutorVisionMapping>(task_name);
      } else {
        result = std::make_shared<ExecutorLaserMapping>(task_name);
      }
      break;

    case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_UWB_TRACKING:
      result = std::make_shared<ExecutorUwbTracking>(task_name);
      break;

    case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_HUMAN_TRACKING:
      result = std::make_shared<ExecutorVisionTracking>(task_name);
      break;

    case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_AUTO_DOCKING:
      result = std::make_shared<ExecutorAutoDock>(task_name);
      break;

    case 0:
      result = std::make_shared<ExecutorResetNav>(task_name);
      break;

    // NavAB
    case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_AB:
      result = std::make_shared<ExecutorAbNavigation>(task_name);
      break;

    // PosesThrough
    case AlgorithmMGR::Goal::NAVIGATION_TYPE_STOP_AB:
      result = std::make_shared<ExecutorPosesThroughNavigation>(task_name);
      break;

    // Laser Localization
    case AlgorithmMGR::Goal::NAVIGATION_TYPE_START_LOCALIZATION:
      if (out_door) {
        result = std::make_shared<ExecutorVisionLocalization>(task_name);
      } else {
        result = std::make_shared<ExecutorLaserLocalization>(task_name);
      }
      break;

    default:
      ERROR("Create executor failed, name is invalid!");
      break;
  }
  return result;
}
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_FACTORY_HPP_
