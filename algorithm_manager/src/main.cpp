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

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_debug/backtrace.hpp"
#include "algorithm_manager/algorithm_task_manager.hpp"

int main(int argc, char ** argv)
{
  LOGGER_MAIN_INSTANCE("AlgorithmTaskManager");
  cyberdog::debug::register_signal();
  rclcpp::init(argc, argv);
  auto atm_ptr = std::make_shared<cyberdog::algorithm::AlgorithmTaskManager>();
  if (!atm_ptr->Init()) {
    ERROR("Init failed, will exit with error!");
    return -1;
  }
  atm_ptr->Run();
  return 0;
}
