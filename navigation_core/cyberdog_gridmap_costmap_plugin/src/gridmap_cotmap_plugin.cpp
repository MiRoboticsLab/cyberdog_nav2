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
#include "cyberdog_gridmap_costmap_plugin/gridmap_cotmap_plugin.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "tf2_ros/message_filter.h"
#pragma GCC diagnostic pop
#include "message_filters/subscriber.h"
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace cyberdog_gridmap_costmap_plugin
{

  GridMapCostLayer::GridMapCostLayer()
      : last_min_x_(-std::numeric_limits<float>::max()),
        last_min_y_(-std::numeric_limits<float>::max()),
        last_max_x_(std::numeric_limits<float>::max()),
        last_max_y_(std::numeric_limits<float>::max())
  {
  }

  // This method is called at the end of plugin initialization.
  // It contains ROS parameter(s) declaration and initialization
  // of need_recalculation_ variable.
  void
  GridMapCostLayer::onInitialize()
  {
    auto node = node_.lock();
    std::string topics_string;
    double transform_tolerance;
    global_frame_ = layered_costmap_->getGlobalFrameID();
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("min_pass_threshold", rclcpp::ParameterValue(-0.15));
    declareParameter("max_pass_threshold", rclcpp::ParameterValue(0.15));
    declareParameter("topic", rclcpp::ParameterValue(std::string("")));
    // declareParameter("transform_tolerance", rclcpp::ParameterValue(0.0));
    node->get_parameter(name_ + "." + "enabled", enabled_);
    node->get_parameter(name_ + "." + "min_pass_threshold", min_pass_threshold_);
    node->get_parameter(name_ + "." + "max_pass_threshold", max_pass_threshold_);
    node->get_parameter(name_ + "." + "topic", topics_string);
    node->get_parameter("transform_tolerance", transform_tolerance);

    RCLCPP_INFO(rclcpp::get_logger(
                    "nav2_costmap_2d"),
                "transform_tolerance: %f,  topics_string: %s  global_frame_: %s",
                transform_tolerance, topics_string.c_str(), global_frame_.c_str());

    mapSubscriber_ = rclcpp_node_->create_subscription<grid_map_msgs::msg::GridMap>(
        topics_string, rclcpp::SystemDefaultsQoS(),
        std::bind(&GridMapCostLayer::ScanCallback, this, std::placeholders::_1));

    need_recalculation_ = false;
    current_ = true;
  }
  void
  GridMapCostLayer::ScanCallback(
      grid_map_msgs::msg::GridMap::ConstSharedPtr message)
  {
    grid_map::GridMapRosConverter::fromMessage(*message, map_);
    grid_map::Length mapLength_;
    grid_map::Position mapPosition_;
    double resolution_;
    grid_map::Size bufferSize_;
    grid_map::Index bufferStartIndex_;

    mapLength_ = map_.getLength();
    mapPosition_ = map_.getPosition();
    resolution_ = map_.getResolution();
    bufferSize_ = map_.getSize();
    bufferStartIndex_ = map_.getStartIndex();

    RCLCPP_DEBUG(rclcpp::get_logger(
                    "nav2_costmap_2d"),
                "ScanCallback : position: (%f, %f);  resolution %f", mapPosition_.x(), mapPosition_.y(), resolution_);

    RCLCPP_DEBUG(rclcpp::get_logger(
                    "nav2_costmap_2d"),
                "             : mapLength_:(%f, %f);  bufferSize_ (%d, %d)", mapLength_.x(), mapLength_.y(), bufferSize_(0), bufferSize_(1));

  }

  // The method is called to ask the plugin: which area of costmap it needs to update.
  // Inside this method window bounds are re-calculated if need_recalculation_ is true
  // and updated independently on its value.
  void
  GridMapCostLayer::updateBounds(
      double, double, double, double *min_x,
      double *min_y, double *max_x, double *max_y)
  {
    if (true /*need_recalculation_*/)
    {
      last_min_x_ = *min_x;
      last_min_y_ = *min_y;
      last_max_x_ = *max_x;
      last_max_y_ = *max_y;
      // For some reason when I make these -<double>::max() it does not
      // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
      // -<float>::max() instead.
      *min_x = -std::numeric_limits<float>::max();
      *min_y = -std::numeric_limits<float>::max();
      *max_x = std::numeric_limits<float>::max();
      *max_y = std::numeric_limits<float>::max();
      need_recalculation_ = false;
    }
    else
    {
      double tmp_min_x = last_min_x_;
      double tmp_min_y = last_min_y_;
      double tmp_max_x = last_max_x_;
      double tmp_max_y = last_max_y_;
      last_min_x_ = *min_x;
      last_min_y_ = *min_y;
      last_max_x_ = *max_x;
      last_max_y_ = *max_y;
      *min_x = std::min(tmp_min_x, *min_x);
      *min_y = std::min(tmp_min_y, *min_y);
      *max_x = std::max(tmp_max_x, *max_x);
      *max_y = std::max(tmp_max_y, *max_y);
    }
  }

  // The method is called when footprint was changed.
  // Here it just resets need_recalculation_ variable.
  void
  GridMapCostLayer::onFootprintChanged()
  {
    need_recalculation_ = true;

    RCLCPP_INFO(rclcpp::get_logger(
                    "nav2_costmap_2d"),
                "GridMapCostLayer::onFootprintChanged(): num footprint points: %lu",
                layered_costmap_->getFootprint().size());
  }

  // The method is called when costmap recalculation is required.
  // It updates the costmap within its window bounds.
  // Inside this method the costmap gradient is generated and is writing directly
  // to the resulting costmap master_grid without any merging with previous layers.
  void
  GridMapCostLayer::updateCosts(
      nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j,
      int max_i,
      int max_j)
  {
    if (!enabled_)
    {
      return;
    }
    // grid_map::Costmap2DConverter<grid_map::GridMap> converter;
    // converter.initializeFromGridMap(map_, master_grid);
    // master_array - is a direct pointer to the resulting master_grid.
    // master_grid - is a resulting costmap combined from all layers.
    // By using this pointer all layers will be overwritten!
    // To work with costmap layer and merge it with other costmap layers,
    // please use costmap_ pointer instead (this is pointer to current
    // costmap layer grid) and then call one of updates methods:
    // - updateWithAddition()
    // - updateWithMax()
    // - updateWithOverwrite()
    // - updateWithTrueOverwrite()
    // In this case using master_array pointer is equal to modifying local costmap_
    // pointer and then calling updateWithTrueOverwrite():
    unsigned char *master_array = master_grid.getCharMap();
    unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

    // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
    // These variables are used to update the costmap only within this window
    // avoiding the updates of whole area.
    //
    // Fixing window coordinates with map size if necessary.
    std::string layer("elevation");
    if (!map_.exists(layer))
      return;
    int columns = map_.getSize()(1);
    int rows = map_.getSize()(0);

    RCLCPP_DEBUG(rclcpp::get_logger(
                     "nav2_costmap_2d"),
                 "origin : (%f, %f))",
                 master_grid.getOriginX(), master_grid.getOriginY());

    RCLCPP_DEBUG(rclcpp::get_logger(
                     "nav2_costmap_2d"),
                 "cellsize : (%d, %d))",
                 master_grid.getSizeInCellsX(), master_grid.getSizeInCellsY());

    RCLCPP_DEBUG(rclcpp::get_logger(
                     "nav2_costmap_2d"),
                 "Metersize : (%f, %f))",
                 master_grid.getSizeInMetersX(), master_grid.getSizeInMetersY());

    RCLCPP_DEBUG(rclcpp::get_logger(
                     "nav2_costmap_2d"),
                 "resolution : (%f))",
                 master_grid.getResolution());

    min_i = std::max(0, min_i);
    min_j = std::max(0, min_j);
    max_i = std::min(static_cast<int>(size_x), max_i);
    max_j = std::min(static_cast<int>(size_y), max_j);
    max_i = std::min(static_cast<int>(rows), max_i);
    max_j = std::min(static_cast<int>(columns), max_j);

    RCLCPP_DEBUG(rclcpp::get_logger(
                     "nav2_costmap_2d"),
                 "update cost : (%d, %d))",
                 max_i, max_j);
    // xxx_i xxx_j are the cell's index.
    rclcpp::Clock clock;
    grid_map::Position pos;
    grid_map::Index submapStartIndex(min_i, min_j);
    grid_map::Index submapBufferSize(max_i - min_i, max_j - min_j);

    // sub map iterator
    for (grid_map::SubmapIterator iterator(map_, submapStartIndex, submapBufferSize);
         !iterator.isPastEnd(); ++iterator)
    {
      grid_map::Position position;

      // get world position of the iterator's cell.
      map_.getPosition(*iterator, position);
      unsigned int mx, my;

      RCLCPP_INFO_THROTTLE(rclcpp::get_logger(
                               "nav2_costmap_2d"),
                           clock, 1000, "position : (%f, %f))",
                           position.x(), position.y());

      // change wold  position to map frame(the map frame is the cost_map frame)
      if (!master_grid.worldToMap(position.x(), position.y(), mx, my))
      {
        RCLCPP_DEBUG(logger_, "Computing map coords failed");
        continue;
      }

      RCLCPP_INFO_THROTTLE(rclcpp::get_logger(
                               "nav2_costmap_2d"),
                           clock, 1000, "map position : (%d, %d))",
                           mx, my);

      RCLCPP_INFO_THROTTLE(rclcpp::get_logger(
                               "nav2_costmap_2d"),
                           clock, 1000, "cost : %f",
                           map_.at(layer, *iterator));

      // get cell index of the map's position
      int index = master_grid.getIndex(mx, my);

      RCLCPP_INFO_THROTTLE(rclcpp::get_logger(
                               "nav2_costmap_2d"),
                           clock, 1000, "index : %d",
                           index);

      // set cost of the corespond cell of cost_map.
      double cost = map_.at(layer, *iterator);
      if (cost > max_pass_threshold_ || cost < min_pass_threshold_)
      {
        master_array[index] = nav2_costmap_2d::LETHAL_OBSTACLE;
      }
      else
      {
        master_array[index] = nav2_costmap_2d::FREE_SPACE;
      }
    }
  }

} // namespace cyberdog_gridmap_costmap_plugin

// This is the macro allowing a cyberdog_gridmap_costmap_plugin::GridMapCostLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cyberdog_gridmap_costmap_plugin::GridMapCostLayer, nav2_costmap_2d::Layer)
