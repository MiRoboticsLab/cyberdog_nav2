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

#include <chrono>
#include <memory>

#include "cyberdog_maps_manager/maps_protocol.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

using namespace std::chrono_literals;

namespace cyberdog
{
namespace maps_manager
{

MapsProtocol::MapsProtocol()
{
  node_ = std::make_shared<rclcpp::Node>("maps_manager_service_client");
  map_client_ = std::make_shared<nav2_util::ServiceClient<protocol::srv::Map>>(
    "maps_manager", node_);

  // spin
  std::thread{[this]() {rclcpp::spin(node_->get_node_base_interface());}}.detach();
}

MapsProtocol::~MapsProtocol()
{
}

bool MapsProtocol::Save(Request::SharedPtr & request)
{
  auto response = std::make_shared<Response>();
  return CallService(request, response);
}

bool MapsProtocol::Delete(Request::SharedPtr & request)
{
  auto response = std::make_shared<Response>();
  return CallService(request, response);
}

bool MapsProtocol::Update(Request::SharedPtr & request)
{
  auto response = std::make_shared<Response>();
  return CallService(request, response);
}

bool MapsProtocol::Query(Request::SharedPtr & request)
{
  auto response = std::make_shared<Response>();
  return CallService(request, response);
}

bool MapsProtocol::Save(Request::SharedPtr & request, Response::SharedPtr & response)
{
  INFO("Call MapsProtocol::Save() function.");
  return CallService(request, response);
}

bool MapsProtocol::Delete(Request::SharedPtr & request, Response::SharedPtr & response)
{
  INFO("Call MapsProtocol::Delete() function.");
  return CallService(request, response);
}

bool MapsProtocol::Update(Request::SharedPtr & request, Response::SharedPtr & response)
{
  INFO("Call MapsProtocol::Update() function.");
  return CallService(request, response);
}

bool MapsProtocol::Query(Request::SharedPtr & request, Response::SharedPtr & response)
{
  INFO("Call MapsProtocol::Query() function.");
  return CallService(request, response);
}

bool MapsProtocol::Load(Request::SharedPtr & request, Response::SharedPtr & response)
{
  INFO("Call MapsProtocol::Load() function.");
  return CallService(request, response);
}

bool MapsProtocol::MapsProtocol::CallService(
  Request::SharedPtr & request,
  Response::SharedPtr & response)
{
  // Wait service
  while (!map_client_->wait_for_service(std::chrono::seconds(5s))) {
    if (!rclcpp::ok()) {
      ERROR("[Laser Mapping] Waiting for the service. but cannot connect the service.");
      return false;
    }
  }

  // Send request
  return map_client_->invoke(request, response);
}

}  // namespace maps_manager
}  // namespace cyberdog
