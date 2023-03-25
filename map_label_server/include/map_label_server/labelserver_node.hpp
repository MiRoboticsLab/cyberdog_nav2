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
#include <set>
#include <atomic>

#include "protocol/srv/get_map_label.hpp"
#include "protocol/srv/set_map_label.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_map_server/map_server.hpp"
#include "nav2_map_server/map_io.hpp"
#include "map_label_server/label_store.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "cyberdog_visions_interfaces/srv/miloc_map_handler.hpp"
#include "nav2_util/lifecycle_service_client.hpp"

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
  using MapAvailableResult = cyberdog_visions_interfaces::srv::MilocMapHandler;

  LabelServer();
  ~LabelServer();

private:
  void HandleSetLabelServiceCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<protocol::srv::SetMapLabel::Request> request,
    std::shared_ptr<protocol::srv::SetMapLabel::Response> response);

  void HandleGetLabelServiceCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<protocol::srv::GetMapLabel::Request> request,
    std::shared_ptr<protocol::srv::GetMapLabel::Response> response);

  bool LoadMapMetaInfo(const std::string & map_name, nav_msgs::msg::OccupancyGrid & map);

  /**
   * @brief Delete map that given the name
   *
   * @param map_name Map's name
   * @return true Remove map success
   * @return false Remove map failed
   */
  bool RemoveMap(const std::string & map_name_directory);


  /**
   * @brief Set the robot map name object
   * @param name Map's name
   */
  void set_robot_map_name(const std::string & name);

  void HandleOutdoor(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<protocol::srv::SetMapLabel::Request> request,
    std::shared_ptr<protocol::srv::SetMapLabel::Response> response);

  /**
   * @brief Set the Outdoor Flag object
   *
   * @param outdoor Is outdoor: vision or lidar
   */
  void SetOutdoorFlag(const std::string & filename, bool outdoor);
  bool ReqeustVisionBuildingMapAvailable(bool & map_suatus, const std::string & map_name = "map");
  bool CheckDuplicateTags(const std::vector<protocol::msg::Label> & labels);
  bool GetOutdoorValue(const std::string & filename, bool & outdoor);
  int CheckVisonMapStatus();

  bool CheckLabelTagHavedExist(std::string & label_tag);

  std::mutex mut;
  rclcpp::Service<protocol::srv::SetMapLabel>::SharedPtr set_label_server_ {nullptr};
  rclcpp::Service<protocol::srv::GetMapLabel>::SharedPtr get_label_server_ {nullptr};

  // Get vision build map available result
  std::shared_ptr<nav2_util::ServiceClient<MapAvailableResult>> map_result_client_ {nullptr};

  // User save robot's map name
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr map_server_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr callback_group_ {nullptr};
  std::shared_ptr<cyberdog::navigation::LabelStore> label_store_ {nullptr};

  // map
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_ {nullptr};

  // outdoor
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_mapping_sub_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_mapping_sub_{nullptr};
  bool use_lidar_create_map_ {false};
  bool use_vision_create_map_ {false};
  std::set<std::string> label_set_;
};
}  // namespace CYBERDOG_NAV
#endif  // MAP_LABEL_SERVER__LABELSERVER_NODE_HPP_
