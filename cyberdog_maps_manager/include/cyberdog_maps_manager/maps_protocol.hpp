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

class MapsProtocol
{
public:
  MapsProtocol();
  ~MapsProtocol();

  MapsProtocol(const MapsProtocol &) = delete;
  MapsProtocol & operator=(const MapsProtocol &) = delete;

  using Request = protocol::srv::Map::Request;
  using Response = protocol::srv::Map::Response;

  bool Save(Request::SharedPtr & request);
  bool Delete(Request::SharedPtr & request);
  bool Update(Request::SharedPtr & request);
  bool Query(Request::SharedPtr & request);

  bool Save(Request::SharedPtr & request, Response::SharedPtr & response);
  bool Delete(Request::SharedPtr & request, Response::SharedPtr & response);
  bool Update(Request::SharedPtr & request, Response::SharedPtr & response);
  bool Query(Request::SharedPtr & request, Response::SharedPtr & response);

private:
  bool CallService(Request::SharedPtr & request, Response::SharedPtr & response);

  rclcpp::Node::SharedPtr node_ {nullptr};
  std::shared_ptr<nav2_util::ServiceClient<protocol::srv::Map>> map_client_ {nullptr};
};

}  // namespace maps_manager
}  // namespace cyberdog

#endif  // CYBERDOG_MAPS_MANAGER__MAPS_PROTOCOL_HPP_
