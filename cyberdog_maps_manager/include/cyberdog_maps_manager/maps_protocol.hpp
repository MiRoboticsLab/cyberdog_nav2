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

#ifndef CYBERDOG_MAPS_MANAGER__MAPS_PROTOCOL_HPP_
#define CYBERDOG_MAPS_MANAGER__MAPS_PROTOCOL_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/service_client.hpp"
#include "protocol/srv/map.hpp"

namespace cyberdog
{
namespace maps_manager
{

class MapsManager;

class MapsProtocol
{
public:
  MapsProtocol();
  ~MapsProtocol();

  MapsProtocol(const MapsProtocol &) = delete;
  MapsProtocol & operator=(const MapsProtocol &) = delete;

  using Request = protocol::srv::Map::Request;
  using Response = protocol::srv::Map::Response;

  bool Save(const Request & request);
  bool Delete(const Request & request);
  bool Update(const Request & request);
  bool Query(const Request & request);

private:
  bool CallService(const Request & request, Response & response);
};

}  // namespace maps_manager
}  // namespace cyberdog

#endif  // CYBERDOG_MAPS_MANAGER__MAPS_PROTOCOL_HPP_
