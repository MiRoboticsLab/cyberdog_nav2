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
#ifndef ALGORITHM_MANAGER__EXECUTOR_RESET_NAV_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_RESET_NAV_HPP_

#include <memory>
#include <string>
#include "cyberdog_common/cyberdog_log.hpp"
#include "algorithm_manager/executor_ab_navigation.hpp"
#include "algorithm_manager/executor_auto_dock.hpp"
#include "algorithm_manager/executor_laser_localization.hpp"
#include "algorithm_manager/executor_laser_mapping.hpp"
#include "algorithm_manager/executor_uwb_tracking.hpp"
#include "algorithm_manager/executor_vision_tracking.hpp"
#include "algorithm_manager/executor_base.hpp"

namespace cyberdog
{
namespace algorithm
{

class ExecutorResetNav : public ExecutorBase
{
public:
  explicit ExecutorResetNav(std::string node_name)
  : ExecutorBase(node_name)
  {
    // Publish trigger signal message for navigation module, Let's stop all depend other
    // modules and some sensors
    stop_robot_nav_client_ = create_client<std_srvs::srv::SetBool>("stop_running_robot_navigation");
    stop_lidar_slam_client_ =
      create_client<std_srvs::srv::SetBool>("reset_stop_lidar_localization");
    stop_vison_slam_client_ =
      create_client<std_srvs::srv::SetBool>("reset_stop_vision_localization");

    // spin
    std::thread{[this]() {
        rclcpp::spin(this->get_node_base_interface());
      }
    }.detach();
  }

  void Start(AlgorithmMGR::Goal::ConstSharedPtr goal) override
  {
    (void)goal;
    ERROR("Error: Start ExecutorResetNav should never be called");
  }

  void Stop(
    const StopTaskSrv::Request::SharedPtr request,
    StopTaskSrv::Response::SharedPtr response) override
  {
    (void)request;
    INFO("Nav Reset");
    response->result = StopTaskSrv::Response::SUCCESS;

    // 1 Stop robot navigation and deactive all lifecycle nodes
    bool is_stop_navgation = StopRobotRunningNavgation();
    if (!is_stop_navgation) {
      ERROR("Stop current robot navigation failed.");
      response->result = StopTaskSrv::Response::FAILED;
    }
    INFO("Stop current robot navigation success.");

    // 2 Stop current SLAM Localizaiton(lidar or vision) and deactive all lifecycle nodes
    bool is_stop_slam = StopSLAMLocalization();
    if (!is_stop_slam) {
      ERROR("Stop current slam localization failed.");
      response->result = StopTaskSrv::Response::FAILED;
    }
    INFO("Stop current slam localization success.");

    // 3 Return manager status and call callback function
    task_cancle_callback_();
    INFO("[Nav Reset]: Nav Reset success.");
  }

  void Cancel() override
  {
    ERROR("Error: Cancel ExecutorResetNav should never be called");
  }

private:
  bool StopRobotRunningNavgation()
  {
    bool is_connect = stop_robot_nav_client_->wait_for_service(std::chrono::seconds(2));
    if (!is_connect) {
      ERROR("Connect stop robot navigation server timeout.");
      return false;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
    request->data = true;

    bool success = SendServerRequest(stop_robot_nav_client_, request, response);
    return success;
  }

  bool StopSLAMLocalization()
  {
    // lidar
    bool is_connect_lidar = stop_lidar_slam_client_->wait_for_service(std::chrono::seconds(2));
    bool is_connect_vision = stop_vison_slam_client_->wait_for_service(std::chrono::seconds(2));

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
    request->data = true;
    bool success = false;

    if (is_connect_lidar) {
      success = SendServerRequest(stop_lidar_slam_client_, request, response);
    }

    if (is_connect_vision) {
      success = SendServerRequest(stop_vison_slam_client_, request, response);
    }

    if (!success) {
      return false;
    }
    INFO("Stop SLAM Localization success.");
    return true;
  }

  bool ConnectStopNavigationServer()
  {
    return true;
  }

  bool ConnectLidarLocalizationServer()
  {
    return true;
  }

  bool ConnectVisionLocalizationServer()
  {
    return true;
  }

  bool SendServerRequest(
    const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client,
    const std_srvs::srv::SetBool::Request::SharedPtr & request,
    std_srvs::srv::SetBool::Response::SharedPtr & response)
  {
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
      ERROR("Cannot get response from service(%s) in 2s.", client->get_service_name());
      return false;
    }

    if (future.get()->success) {
      INFO("Success to call stop service : %s.", client->get_service_name());
    } else {
      ERROR("Get error when call stop service : %s.", client->get_service_name());
    }
    return true;
  }

  // vision mapping alive
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stop_robot_nav_client_ {nullptr};
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stop_lidar_slam_client_ {nullptr};
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stop_vison_slam_client_ {nullptr};
};  // class ExecutorLaserMapping
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_RESET_NAV_HPP_
