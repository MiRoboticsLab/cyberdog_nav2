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

#include <memory>
#include <vector>
#include <string>
#include "behavior_manager/behavior_manager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "std_srvs/srv/set_bool.hpp"

int main(int argc, char ** argv)
{
  LOGGER_MAIN_INSTANCE("BehaviorManager");
  cyberdog::debug::register_signal();
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("behavior_manager_test");
  auto atm = std::make_shared<cyberdog::algorithm::BehaviorManager>("behavior_manager");
  auto reset_bm_srv = node->create_service<std_srvs::srv::SetBool>(
      "launch_bm",
      [atm](const std_srvs::srv::SetBool_Request::SharedPtr req,
        std_srvs::srv::SetBool_Response::SharedPtr res) {
          if(req->data) {
            atm->Launch(true, true);
          } else {
            atm->Launch(false, false);
            atm->Reset();
          }
          res->success = true;
        });
  rclcpp::spin(node);
}
