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

#ifndef ALGORITHM_MANAGER__EXECUTOR_POSES_THROUGH_NAVIGATION_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_POSES_THROUGH_NAVIGATION_HPP_

#include <string>
#include <memory>
#include <vector>

#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "algorithm_manager/executor_base.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cyberdog_visions_interfaces/srv/miloc_map_handler.hpp"

namespace cyberdog
{
namespace algorithm
{

class ExecutorPosesThroughNavigation : public ExecutorBase
{
public:
  explicit ExecutorPosesThroughNavigation(std::string node_name);
  ~ExecutorPosesThroughNavigation();

  /**
   * @brief Start Poses Though AB
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
   * @brief Cancel Navigation Poses Though, it't can debug ros command
   */
  void Cancel() override;

private:
};

}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_POSES_THROUGH_NAVIGATION_HPP_
