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

#ifndef CYBERDOG_MAP_MANAGER__LABEL_TAG_HPP_
#define CYBERDOG_MAP_MANAGER__LABEL_TAG_HPP_

#include <string>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "protocol/srv/map.hpp"

namespace cyberdog
{
namespace map_manager
{

class MapManager : public nav2_util::LifecycleNode
{
public:
  explicit MapManager();
  ~MapManager();

  MapManager(const MapManager &) = delete;
  MapManager & operator=(const MapManager &) = delete;

  /**
   * @brief Handle other command request for map service
   *
   * @param request Json format
   * @param response Json format
   */
  void HandleMapServiceCallback(
    const protocol::srv::Map::Request::SharedPtr request,
    protocol::srv::Map::Response::SharedPtr response);

  /**
   * @brief Configures controller parameters and member variables
   *
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize controller
   * plugin
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activates member variables
   *
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivates member variables
   *
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Calls clean up states and resets member variables.
   *
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in Shutdown state
   *
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:

  // Map build type
  enum MapType
  {
    Lidar,
    Vision
  };

  /**
   * @brief Save map's filename
   * 
   * @param name Map's filename
   * @param type Lidar or Vision SLAM build map
   * @return true If true return success
   * @return false If false return failed
   */
  bool SaveMap(const std::string & name, const MapType & type);

  /**
   * @brief Delete map's filename
   * 
   * @param name Map's filename
   * @param type Lidar or Vision SLAM build map
   * @return true If true return success
   * @return false If false return failed
   */
  bool DeleteMap(const std::string & name, const MapType & type);

  /**
   * @brief Rename map's filename from old_name to new_name
   * 
   * @param old_name map's filename old_name
   * @param new_name map's filename new_name
   */
  void UpdateMap(const std::string & old_name, const std::string & new_name);

  /**
   * @brief Get the Map object
   * 
   * @param name Map's filename
   * @param type Lidar or Vision SLAM build map
   * @param map Get lidar or vision build OccupancyGrid map
   * @return true If true return success
   * @return false If false return failed
   */
  bool GetMap(const std::string & name, const MapType & type, nav_msgs::msg::OccupancyGrid & map);

  /**
   * @brief Get the Map List of all map's filename
   * 
   * @param type Lidar or Vision SLAM build map
   * @param maps_table all map's filename
   * @return true If true return success
   * @return false If false return failed
   */
  bool GetMapList(const MapType & type, std::vector<std::string> & maps_table);

  // As client request dataset and add delete update and query
  rclcpp::Client<protocol::srv::Map>::SharedPtr map_client_ {nullptr};

  // As server handle other command request
  rclcpp::Service<protocol::srv::Map>::SharedPtr map_server_ {nullptr};
};


}  // namespace namespace map_manager
}  // namespace cyberdog

#endif  // CYBERDOG_MAP_MANAGER__LABEL_TAG_HPP_
