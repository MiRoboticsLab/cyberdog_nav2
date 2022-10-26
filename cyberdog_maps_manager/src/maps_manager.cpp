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
#include <limits>
#include <memory>
#include <vector>

#include "cyberdog_maps_manager/maps_manager.hpp"


using rapidjson::Document;
using rapidjson::kObjectType;

namespace cyberdog
{
namespace maps_manager
{

MapsManager::MapsManager()
{
  protocol_ = std::make_shared<MapsProtocol>();
}

MapsManager::~MapsManager()
{
}

bool MapsManager::Run(const Request & request, Response & response)
{
  bool success = false;
  MapData map_data;

  switch (request.cmd_type) {
    case CommandType::SAVE:
      success = Save(request.map_type, request.command);
      break;

    case CommandType::DELETE:
      success = Delete(request.map_type, request.command);
      break;

    case CommandType::UPDATE:
      success = Update(request.map_type, request.command);
      break;

    case CommandType::QUERY:
      success = Query(request.map_type, request.command);
      break;

    case CommandType::LOAD:
      success = Load(request.map_type, request.command, map_data);
      break;

    default:
      break;
  }
}

bool MapsManager::Save(const MapType & map_type, const CommandRequest & request)
{
  // Create command request
  auto param = std::make_shared<protocol::srv::Map::Request>();
  param->command = protocol::srv::Map::Request::MAP_COMMAND_TYPE_INSERT;

  // map_type
  if (map_type == MapType::Lidar) {
    param->map_build_type = protocol::srv::Map::Request::MAP_BUILD_TYPE_LIDAR;
  } else if (map_type == MapType::Vision) {
    param->map_build_type = protocol::srv::Map::Request::MAP_BUILD_TYPE_VISION;
  }

  // Convert request to json
  param->request = ToString(request);

  // Run save command
  auto response = std::make_shared<protocol::srv::Map::Response>();
  bool success = protocol_->Save(param, response);
  if (!success) {
    ERROR("Run save command failed.");
    return false;
  }

  // Check save command run success
  success = CheckSaveSuccess(response);
  if (!success) {
    ERROR("Run save command failed.");
    return false;
  }

  INFO("Run save command success.");
  return true;
}

bool MapsManager::Delete(const MapType & map_type, const CommandRequest & request)
{
  // Create command request
  auto param = std::make_shared<protocol::srv::Map::Request>();
  param->command = protocol::srv::Map::Request::MAP_COMMAND_TYPE_DELETE;

  // map_type
  if (map_type == MapType::Lidar) {
    param->map_build_type = protocol::srv::Map::Request::MAP_BUILD_TYPE_LIDAR;
  } else if (map_type == MapType::Vision) {
    param->map_build_type = protocol::srv::Map::Request::MAP_BUILD_TYPE_VISION;
  }

  // Convert request to json
  param->request = ToString(request);

  // Run delete command
  auto response = std::make_shared<protocol::srv::Map::Response>();
  bool success = protocol_->Delete(param, response);
  if (!success) {
    ERROR("Run delete command failed.");
    return false;
  }

  // Check delete command run success
  success = CheckDeleteSuccess(response);
  if (!success) {
    ERROR("Run delete command failed.");
    return false;
  }

  INFO("Run delete command success.");
  return true;
}

bool MapsManager::Update(const MapType & map_type, const CommandRequest & request)
{
  // Create command request
  auto param = std::make_shared<protocol::srv::Map::Request>();
  param->command = protocol::srv::Map::Request::MAP_COMMAND_TYPE_INSERT;

  // map_type
  if (map_type == MapType::Lidar) {
    param->map_build_type = protocol::srv::Map::Request::MAP_BUILD_TYPE_LIDAR;
  } else if (map_type == MapType::Vision) {
    param->map_build_type = protocol::srv::Map::Request::MAP_BUILD_TYPE_VISION;
  }

  // Convert request to json
  param->request = ToString(request);

  // Run update command
  auto response = std::make_shared<protocol::srv::Map::Response>();
  bool success = protocol_->Update(param, response);
  if (!success) {
    ERROR("Run update command failed.");
    return false;
  }

  // Check update command run success
  success = CheckUpdateSuccess(response);
  if (!success) {
    ERROR("Run update command failed.");
    return false;
  }

  INFO("Run update command success.");
  return true;
}

bool MapsManager::Query(const MapType & map_type, const CommandRequest & request)
{
  // Create command request
  auto param = std::make_shared<protocol::srv::Map::Request>();
  param->command = protocol::srv::Map::Request::MAP_COMMAND_TYPE_INSERT;

  // map_type
  if (map_type == MapType::Lidar) {
    param->map_build_type = protocol::srv::Map::Request::MAP_BUILD_TYPE_LIDAR;
  } else if (map_type == MapType::Vision) {
    param->map_build_type = protocol::srv::Map::Request::MAP_BUILD_TYPE_VISION;
  }

  // Convert request to json
  param->request = ToString(request);

  // Run query command
  auto response = std::make_shared<protocol::srv::Map::Response>();
  bool success = protocol_->Query(param, response);
  if (!success) {
    ERROR("Run query command failed.");
    return false;
  }

  success = CheckQuerySuccess(response);
  if (!success) {
    ERROR("Run query command failed.");
    return false;
  }

  INFO("Run query command success.");
  return true;
}

bool MapsManager::Query(const MapType & map_type,
  const CommandRequest & request, std::vector<MapInfo> & maps)
{
  // Create command request
  auto param = std::make_shared<protocol::srv::Map::Request>();
  param->command = protocol::srv::Map::Request::MAP_COMMAND_TYPE_INSERT;

  // map_type
  if (map_type == MapType::Lidar) {
    param->map_build_type = protocol::srv::Map::Request::MAP_BUILD_TYPE_LIDAR;
  } else if (map_type == MapType::Vision) {
    param->map_build_type = protocol::srv::Map::Request::MAP_BUILD_TYPE_VISION;
  }

  // Convert request to json
  param->request = ToString(request);

  // Run query command
  auto response = std::make_shared<protocol::srv::Map::Response>();
  bool success = protocol_->Query(param, response);
  if (!success) {
    ERROR("Run query command failed.");
    return false;
  }

  success = CheckQuerySuccess(response, maps);
  if (!success) {
    ERROR("Run query command failed.");
    return false;
  }


  INFO("Run query command success.");
  return true;
}

bool MapsManager::Load(const MapType & map_type, const CommandRequest & request, MapData & map)
{
  // Create command request
  auto param = std::make_shared<protocol::srv::Map::Request>();
  param->command = protocol::srv::Map::Request::MAP_COMMAND_TYPE_LOAD;

  // map_type
  if (map_type == MapType::Lidar) {
    param->map_build_type = protocol::srv::Map::Request::MAP_BUILD_TYPE_LIDAR;
  } else if (map_type == MapType::Vision) {
    param->map_build_type = protocol::srv::Map::Request::MAP_BUILD_TYPE_VISION;
  }

  // Convert request to json
  param->request = ToString(request);

  // Run query command
  auto response = std::make_shared<protocol::srv::Map::Response>();
  bool success = protocol_->Load(param, response);
  if (!success) {
    ERROR("Run load command failed.");
    return false;
  }

  success = CheckLoadSuccess(response, map);
  if (!success) {
    ERROR("Run load command failed.");
    return false;
  }


  INFO("Run load command success.");
  return true;
}

bool MapsManager::CheckSaveSuccess(const protocol::srv::Map::Response::SharedPtr & response)
{
  //   response {
  //     namecode : 1002
  //     uint64 timestamp
  //     id: int
  //     success : true
  // }

  Document json_response(kObjectType);
  if (!common::CyberdogJson::String2Document(response->response, json_response)) {
    ERROR("Response params convert to document format error.");
    return false;
  }

  int id = std::numeric_limits<int>::min();
  bool success = false;
  common::CyberdogJson::Get(json_response, "id", id);
  common::CyberdogJson::Get(json_response, "success", success);
  return success;
}

bool MapsManager::CheckDeleteSuccess(const protocol::srv::Map::Response::SharedPtr & response)
{
  // response {
  //   namecode : 1004
  //   uint64 timestamp
  //   success[]: true
  // }

  Document json_response(kObjectType);
  if (!common::CyberdogJson::String2Document(response->response, json_response)) {
    ERROR("Response params convert to document format error.");
    return false;
  }

  int id = std::numeric_limits<int>::min();
  bool success = true;

  const rapidjson::Value & result = json_response["success"];
  if (result.IsArray()) {
    for (auto it = result.Begin(); it != result.End(); it++) {
      if (!it->GetBool()) {
        success = false;
        break;
      }
    }
  }

  return success;
}

bool MapsManager::CheckUpdateSuccess(const protocol::srv::Map::Response::SharedPtr & response)
{
  // namecode : 1006
  // uint64 timestamp
  // success[] : true

  Document json_response(kObjectType);
  if (!common::CyberdogJson::String2Document(response->response, json_response)) {
    ERROR("Response params convert to document format error.");
    return false;
  }

  bool success = true;
  const rapidjson::Value & result = json_response["success"];
  if (result.IsArray()) {
    for (auto it = result.Begin(); it != result.End(); it++) {
      if (!it->GetBool()) {
        success = false;
        break;
      }
    }
  }

  return success;
}

bool MapsManager::CheckQuerySuccess(const protocol::srv::Map::Response::SharedPtr & response)
{
  return true;
}

bool MapsManager::CheckQuerySuccess(
  const protocol::srv::Map::Response::SharedPtr & response,
  std::vector<MapInfo> & maps)
{
  Document json_response(kObjectType);
  if (!common::CyberdogJson::String2Document(response->response, json_response)) {
    ERROR("Response params convert to document format error.");
    return false;
  }

  bool success = true;
  const rapidjson::Value & result = json_response["maps"];
  if (result.IsArray()) {
    for (auto it = result.Begin(); it != result.End(); it++) {
      auto map_object = it->GetObject();
      // maps.push_back({
      //   map_object["name"],
      //   map_object["id"],
      //   map_object["success"],
      //   map_object["status"]
      // });
    }
  }
  return true;
}

bool MapsManager::CheckLoadSuccess(
  const protocol::srv::Map::Response::SharedPtr & response,
  MapData & map_data)
{
  Document json_response(kObjectType);
  if (!common::CyberdogJson::String2Document(response->response, json_response)) {
    ERROR("Response params convert to document format error.");
    return false;
  }

  bool success = true;
  const rapidjson::Value & map_object = json_response["maps"];

  // int id;
  // std::string name;

  // // meta data
  // uint64_t map_load_time;
  // float resolution;
  // uint32_t width;
  // uint32_t height;

  // // position
  // float position_x;
  // float position_y;
  // float position_z;
  // float position_w;

  // // quaternion
  // float quaternion_x;
  // float quaternion_y;
  // float quaternion_z;
  // float quaternion_w;

  // // data
  // std::vector<u_int8_t> data;

  // map_data.id = map_object["id"];
  // map_data.name = map_object["name"];
  // map_data.map_load_time = map_object["map_load_time"];
  // map_data.width = map_object["width"];
  // map_data.name = map_object["height"];

  // map_data.position_x = map_object["position_x"];
  // map_data.position_y = map_object["position_y"];
  // map_data.position_z = map_object["position_z"];
  // map_data.position_w = map_object["position_w"];

  // map_data.quaternion_x = map_object["quaternion_x"];
  // map_data.quaternion_y = map_object["quaternion_y"];
  // map_data.quaternion_z = map_object["quaternion_z"];
  // map_data.quaternion_w = map_object["quaternion_w"];

  return true;
}

std::string MapsManager::CommandRequestToString(const CommandRequest & request)
{
  Document json_request(rapidjson::kObjectType);


  common::CyberdogJson::Add(json_request, "namecode", request.name_code);
  common::CyberdogJson::Add(json_request, "timestamp", request.timestamp);

  rapidjson::Value maps(rapidjson::kArrayType);
  // for (int i = 0; i < request.maps_name.size(); i++) {
  //   maps.PushBack(request.maps_name[i], json_request.GetAllocator());
  // }

  rapidjson::Value ids(rapidjson::kArrayType);
  // for (int i = 0; i < request.ids.size(); i++) {
  //   ids.PushBack(request.ids[i], json_request.GetAllocator());
  // }

  std::string result;
  if (!common::CyberdogJson::Document2String(json_request, result)) {
    ERROR("CommandRequest params convert to json format error.");
    return result;
  }
  return result;
}

std::string MapsManager::ToString(const CommandRequest & request)
{
  Document json_request(rapidjson::kObjectType);
  common::CyberdogJson::Add(json_request, "namecode", request.name_code);
  common::CyberdogJson::Add(json_request, "timestamp", request.timestamp);

  // map
  if (request.maps_name.size() == 1) {
    common::CyberdogJson::Add(json_request, "map_name", request.maps_name[0]);
  } else if (request.ids.size() >= 2) {
    rapidjson::Value maps(rapidjson::kArrayType);
    for (int i = 0; i < request.maps_name.size(); i++) {
      rapidjson::Value map(rapidjson::kStringType);
      map.SetString(request.maps_name[i].c_str(), request.maps_name[i].length(), json_request.GetAllocator());
      maps.PushBack(map, json_request.GetAllocator());
    }
    json_request.AddMember("map", maps, json_request.GetAllocator());
  }

  // ids
  if (request.ids.size() == 1) {
    common::CyberdogJson::Add(json_request, "id", request.ids[0]);
  } else if (request.ids.size() >= 2) {
    rapidjson::Value ids(rapidjson::kArrayType);
    for (int i = 0; i < request.ids.size(); i++) {
      rapidjson::Value id(rapidjson::kNumberType);
      id.SetInt64(request.ids[i]);
      ids.PushBack(id, json_request.GetAllocator());
    }
    json_request.AddMember("id", ids, json_request.GetAllocator());
  }

  // Convert to string
  std::string result;
  if (!common::CyberdogJson::Document2String(json_request, result)) {
    ERROR("CommandRequest params convert to json format error.");
    return result;
  }
  return result;
}

}  // namespace maps_manager
}  // namespace cyberdog
