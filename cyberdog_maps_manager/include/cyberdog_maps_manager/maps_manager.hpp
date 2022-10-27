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

#ifndef CYBERDOG_MAPS_MANAGER__MAPS_MANAGER_HPP_
#define CYBERDOG_MAPS_MANAGER__MAPS_MANAGER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "cyberdog_common/cyberdog_json.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_maps_manager/maps_protocol.hpp"

namespace cyberdog
{
namespace maps_manager
{

class MapsManager
{
public:
  MapsManager();
  ~MapsManager();

  enum class CommandType
  {
    SAVE,
    DELETE,
    UPDATE,
    QUERY,
    LOAD,
    UNKNOWN
  };

  enum class MapType
  {
    Lidar,
    Vision,
    Unknown
  };

  struct CommandRequest
  {
    uint64_t name_code;
    uint64_t timestamp;
    std::vector<uint64_t> ids;
    std::vector<std::string> maps_name;
  };

  struct MapInfo
  {
    std::string name;
    int id;
    bool success;
    int status;
  };

  struct MapData
  {
    int id;
    std::string name;

    // meta data
    uint64_t map_load_time;
    float resolution;
    uint32_t width;
    uint32_t height;

    // position
    float position_x;
    float position_y;
    float position_z;
    float position_w;

    // quaternion
    float quaternion_x;
    float quaternion_y;
    float quaternion_z;
    float quaternion_w;

    // data
    std::vector<u_int8_t> data;
  };

  struct Request
  {
    CommandRequest command;
    CommandType cmd_type;
    MapType map_type;
  };

  struct Response
  {
  };

  bool Run(const Request & request, Response & response);
  bool Save(const MapType & map_type, const CommandRequest & request);
  bool Delete(const MapType & map_type, const CommandRequest & request);
  bool Update(const MapType & map_type, const CommandRequest & request);
  bool Query(const MapType & map_type, const CommandRequest & request);
  bool Query(const MapType & map_type, const CommandRequest & request, std::vector<MapInfo> & maps);
  bool Load(const MapType & map_type, const CommandRequest & request, MapData & map);

// private:
  bool CheckSaveSuccess(const protocol::srv::Map::Response::SharedPtr & response);
  bool CheckDeleteSuccess(const protocol::srv::Map::Response::SharedPtr & response);
  bool CheckUpdateSuccess(const protocol::srv::Map::Response::SharedPtr & response);
  bool CheckQuerySuccess(const protocol::srv::Map::Response::SharedPtr & response);
  bool CheckQuerySuccess(
    const protocol::srv::Map::Response::SharedPtr & response,
    std::vector<MapInfo> & maps);
  bool CheckLoadSuccess(
    const protocol::srv::Map::Response::SharedPtr & response,
    MapData & map_data);

  std::string CommandRequestToString(const CommandRequest & request);
  std::string ToString(const CommandRequest & request);


  std::shared_ptr<MapsProtocol> protocol_ {nullptr};
};

}  // namespace maps_manager
}  // namespace cyberdog

#endif  // CYBERDOG_MAPS_MANAGER__MAPS_MANAGER_HPP_
