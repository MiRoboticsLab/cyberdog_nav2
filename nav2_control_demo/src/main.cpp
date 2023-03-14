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

#include "nav2_control_demo/nav2_mapping.hpp"
#include "nav2_control_demo/nav2_localization.hpp"
#include "nav2_control_demo/nav2_navigatte_pose.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto mapping = std::make_shared<cyberdog::nav2_control_demo::MappingNode>();
  auto localization = std::make_shared<cyberdog::nav2_control_demo::LocalizationNode>();
  auto navigatte_pose = std::make_shared<cyberdog::nav2_control_demo::NavigattePose>();

  executor.add_node(mapping);
  executor.add_node(localization);
  executor.add_node(navigatte_pose);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}