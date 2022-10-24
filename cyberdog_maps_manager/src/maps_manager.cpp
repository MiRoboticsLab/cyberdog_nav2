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

#include <string>

#include "cyberdog_maps_manager/maps_manager.hpp"

namespace cyberdog
{
namespace maps_manager
{

MapsManager::MapsManager(const std::string & name)
: rclcpp::Node(name)
{
}

MapsManager::~MapsManager()
{
}

}  // namespace maps_manager
}  // namespace cyberdog
