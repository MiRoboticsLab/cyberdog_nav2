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

#ifndef MAP_LABEL_SERVER__LABELSERVER_NODE_HPP_
#define MAP_LABEL_SERVER__LABELSERVER_NODE_HPP_

#include <memory>
#include <string>

#include "protocol/srv/get_map_label.hpp"
#include "protocol/srv/set_map_label.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "nav2_map_server/map_server.hpp"
#include "nav2_map_server/map_io.hpp"
#include "map_label_server/label_store.hpp"

namespace CYBERDOG_NAV
{
typedef struct LABEL
{
  float x;
  float y;
} LabelT;

class LabelServer : public rclcpp::Node
{
public:
  LabelServer();
  ~LabelServer();

  void handle_set_label(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<protocol::srv::SetMapLabel::Request> request,
    std::shared_ptr<protocol::srv::SetMapLabel::Response> response);
  void handle_get_label(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<protocol::srv::GetMapLabel::Request> request,
    std::shared_ptr<protocol::srv::GetMapLabel::Response> response);
  void read_map_label(std::string filename, LABEL & label);

  void writ_map_label(std::string filename, const LABEL & label);

  bool makeMapFolder(std::string filename);

  bool isFolderExist(std::string path);

  bool isFileExixt(std::string path);

  bool removeFile(std::string path);

  void writeLabel(std::string path, LABEL label);

  void PrintMapData();

  bool LoadMapMetaInfo(const std::string & map_name, nav_msgs::msg::OccupancyGrid & map);

  /**
   * @brief Delete map that given the name
   *
   * @param map_name Map's name
   * @return true Remove map success
   * @return false Remove map failed
   */
  bool RemoveMap(const std::string & map_name);

  /**
   * @brief Handle user save mapping map's name
   * @param request_header
   * @param request If true get current map's name
   * @param response Return user set map's name
   */
  void HandleRequestUserSaveMapName(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /**
   * @brief Set the robot map name object
   * @param name Map's name
   */
  void set_robot_map_name(const std::string & name);

  /**
   * @brief Get robot current map's name
   * @return std::string Return map's name
   */
  std::string robot_map_name() const;

private:
  std::mutex mut;
  rclcpp::Service<protocol::srv::SetMapLabel>::SharedPtr set_label_server_;
  rclcpp::Service<protocol::srv::GetMapLabel>::SharedPtr get_label_server_;

  // User save robot's map name
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr map_server_;
  std::string robot_map_name_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::shared_ptr<cyberdog::navigation::LabelStore> map_label_store_ptr_ {nullptr};
};
}  // namespace CYBERDOG_NAV
#endif  // MAP_LABEL_SERVER__LABELSERVER_NODE_HPP_
