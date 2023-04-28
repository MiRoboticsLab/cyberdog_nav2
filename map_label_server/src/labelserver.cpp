// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#include <cstdio>
#include <memory>

#include "map_label_server/labelserver_node.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_debug/backtrace.hpp"

int main(int argc, char ** argv)
{
  LOGGER_MAIN_INSTANCE("LabelServer");
  cyberdog::debug::register_signal();
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CYBERDOG_NAV::LabelServer>();
  rclcpp::executors::MultiThreadedExecutor exec_;
  exec_.add_node(node->get_node_base_interface());
  exec_.spin();
  rclcpp::shutdown();
  return 0;
}
