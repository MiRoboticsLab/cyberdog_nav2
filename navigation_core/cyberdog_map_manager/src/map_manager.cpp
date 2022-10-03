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

#include <string>
#include <memory>
#include <vector>

#include "cyberdog_map_manager/map_manager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_json.hpp"

namespace cyberdog
{
namespace map_manager
{

MapManager::MapManager()
: nav2_util::LifecycleNode("cyberdog_map_manager", "", true)
{
  map_client_ = this->create_client<protocol::srv::Map>("maps_manager");
  map_server_ = this->create_service<protocol::srv::Map>(
    "map_service",
    std::bind(
      &MapManager::HandleMapServiceCallback, this,
      std::placeholders::_1, std::placeholders::_2));
}

MapManager::~MapManager()
{
}

void MapManager::HandleMapServiceCallback(
  const protocol::srv::Map::Request::SharedPtr request,
  protocol::srv::Map::Response::SharedPtr response)
{
}

nav2_util::CallbackReturn MapManager::on_configure(const rclcpp_lifecycle::State & state)
{
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MapManager::on_activate(const rclcpp_lifecycle::State & state)
{
  INFO("Activating");
  // create bond connection
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MapManager::on_deactivate(const rclcpp_lifecycle::State & state)
{
  INFO("Deactivating");
  // destroy bond connection
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MapManager::on_cleanup(const rclcpp_lifecycle::State & state)
{
  INFO("Cleaning up");

  // Release any allocated resources
  map_client_.reset();
  map_server_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MapManager::on_shutdown(const rclcpp_lifecycle::State & state)
{
  INFO("Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool MapManager::SaveMap(const std::string & name, const MapType & type)
{
  // Set request command
  auto request = std::make_shared<protocol::srv::Map::Request>();
  request->command = protocol::srv::Map::Request::MAP_COMMAND_TYPE_INSERT;

  // Check lidar map or vision map
  if (type == MapType::Lidar) {
    request->map_build_type = protocol::srv::Map::Request::MAP_BUILD_TYPE_LIDAR;
  } else if (type == MapType::Vision) {
    request->map_build_type = protocol::srv::Map::Request::MAP_BUILD_TYPE_VISION;
  }

  // request json
  rapidjson::Document json_request(rapidjson::kObjectType);
  common::CyberdogJson::Add(json_request, "namecode", 1001);
  common::CyberdogJson::Add(json_request, "timestamp", 12345678);
  common::CyberdogJson::Add(json_request, "map_name", name);

  // command
  std::string string_command;
  bool convert = common::CyberdogJson::Document2String(json_request, string_command);
  if (!convert) {
    ERROR("Convert json to string failed.");
    return false;
  }
  request->request = string_command;

  // Send command request
  return CallService(request, std::chrono::seconds(5));
}

bool MapManager::DeleteMap(const std::string & name, const MapType & type)
{
  return true;
}

bool MapManager::UpdateMap(const std::string & old_name, const std::string & new_name)
{
  return true;
}

bool MapManager::GetMap(
  const std::string & name,
  const MapType & type, nav_msgs::msg::OccupancyGrid & map)
{
  return true;
}

bool MapManager::GetMapList(const MapType & type, std::vector<std::string> & maps_table)
{
  return true;
}

bool MapManager::CallService(
  const protocol::srv::Map::Request::SharedPtr request,
  const std::chrono::seconds timeout)
{
  return true;
}

}  // namespace namespace map_manager
}  // namespace cyberdog
