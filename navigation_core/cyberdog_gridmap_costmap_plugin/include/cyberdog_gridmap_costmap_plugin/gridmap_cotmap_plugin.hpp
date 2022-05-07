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

#ifndef GRADIENT_LAYER_HPP_
#define GRADIENT_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/TypeDefs.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_costmap_2d/grid_map_costmap_2d.hpp"

namespace cyberdog_gridmap_costmap_plugin
{

class GridMapCostLayer : public nav2_costmap_2d::Layer
{
public:
  GridMapCostLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();

  virtual bool isClearable() {return false;}

private:
  /// @brief Used for the observation message filters
  std::vector<std::shared_ptr<message_filters::SubscriberBase>> observation_subscribers_;
  /// @brief Used to make sure that transforms are available for each sensor
  std::vector<std::shared_ptr<tf2_ros::MessageFilterBase>> observation_notifiers_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  std::string global_frame_;

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr mapSubscriber_;
void
  ScanCallback(
  grid_map_msgs::msg::GridMap::ConstSharedPtr message);
  grid_map::GridMap map_;
  // Indicates that the entire gradient should be recalculated next time.
  bool need_recalculation_;
  double min_pass_threshold_;
  double max_pass_threshold_;

  // Size of gradient in cells
  int GRADIENT_SIZE = 20;
  // Step of increasing cost per one cell in gradient
  int GRADIENT_FACTOR = 10;
};

}  // namespace cyberdog_gridmap_costmap_plugin

#endif  // GRADIENT_LAYER_HPP_
