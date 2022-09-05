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

#ifndef MAP_LABEL_SERVER__LABEL_STORE_HPP_
#define MAP_LABEL_SERVER__LABEL_STORE_HPP_

#include <string>
#include <memory>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "map_label_server/filesystem.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "protocol/srv/get_map_label.hpp"
#include "protocol/srv/set_map_label.hpp"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"

namespace cyberdog
{
namespace navigation
{

class LabelStore
{
public:
  LabelStore();
  ~LabelStore();

  LabelStore(const LabelStore &) = delete;
  LabelStore & operator=(const LabelStore &) = delete;

  /**
   * @brief Add map's label
   * 
   * @param filename 
   * @param label_name 
   * @param label 
   */
 
  void AddLabel(
    const std::string & filename,
    const std::string & label_name,
    const protocol::msg::Label::SharedPtr label);

  void AddLabel(
    const std::string & filename,
    const std::string & label_name,
    const protocol::msg::Label::SharedPtr label,
    rapidjson::Document & doc);


  /**
   * @brief Delete map's label
   * 
   * @param filename 
   * @param label_name 
   */
  void DeleteLabel(
    const std::string & filename, 
    const std::string & label_name,
    rapidjson::Document & existed_doc);

  /**
   * @brief change map's label
   * 
   * @param filename 
   * @param label_name 
   * @param new_label_name 
   * @param label 
   */
  void ChangeLable(
    const std::string & old_label_name,
    const std::string & new_label_name,
    const protocol::msg::Label::SharedPtr & new_label,
     rapidjson::Document & existed_doc
    );


  /**
   * @brief Check map's label exist
   *
   * @param filename 
   * @param label_name 
   */
  bool IsLabelExist(
    const std::string & filename, 
    const std::string & label_name,
    rapidjson::Document & existed_doc);

  /**
   * @brief Set the Label object
   * 
   * @param filename 
   * @param label_name 
   * @param label 
   */
  void SetLabel(
    const std::string & filename,
    const std::string & label_name,
    const protocol::msg::Label::SharedPtr label);


  /**
   * @brief Create a Map Label File object
   * 
   * @param directory 
   * @param filename 
   * @return true 
   * @return false 
   */
  bool CreateMapLabelFile(const std::string & directory, const std::string & filename);

  /**
   * @brief Delete a Map's Label File object
   * 
   * @param filename 
   * @return true 
   * @return false 
   */
  bool DeleteMapLabelFile(const std::string & filename);

  /**
   * @brief Check a file is exist
   * 
   * @param filename 
   * @return true If file is exist
   * @return false If file not exist
   */
  bool IsExist(const std::string & filename);

  /**
   * @brief Get the Labels Filename From Map object
   * 
   * @param map_name Map's filename
   * @return std::string 
   */
  std::string GetLabelsFilenameFromMap(const std::string & map_name);

  /**
   * @brief Get map's label directory
   * 
   * @return std::string 
   */
  std::string map_label_directory() const;

  /**
   * @brief Set the Map Name object
   * 
   * @param label_filename 
   * @param map_filename 
   * @param doc 
   */
  void SetMapName(
    const std::string & label_filename,
    const std::string & map_filename,
    rapidjson::Document & doc);

  /**
   * @brief Load map's label from given directory
   * 
   * @param directory 
   * @return true 
   * @return false 
   */
  bool LoadLabels(const std::string & directory);

  /**
   * @brief Write json string format to label file
   * 
   * @param label_filename 
   * @param doc 
   */
  void Write(const std::string & label_filename, const rapidjson::Document & doc);

  /**
   * @brief Read label from json filename and save labels' into a vector 
   * 
   * @param label_filename 
   * @param labels 
   */
  void Read(const std::string & label_filename, std::vector<protocol::msg::Label> & labels);

  /**
   * @brief Read label from json filename 
   * 
   * @param label_filename 
   * @param doc 
   * @return true 
   * @return false 
   */
  bool ReadLabels(const std::string & label_filename, rapidjson::Document & doc);

  /**
   * @brief Debug for logic test
   * 
   */
  void Debug();

private:
  /**
   * @brief protocol::msg::Label convert to json format
   * 
   * @param label 
   * @return rapidjson::Document 
   */
  rapidjson::Document ToJson(const protocol::msg::Label::SharedPtr label);

  std::unordered_map<std::string, std::string> labels_name_;
  std::string map_label_directory_;
};
}   //  namespace navigation
}   //  namespace cyberdog

#endif  // MAP_LABEL_SERVER__LABEL_STORE_HPP_
