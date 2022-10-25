
// Copyright (c) 2021 Xiaomi Corporation
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

#include <gtest/gtest.h>

#include <vector>
#include <memory>
#include <string>
#include <unordered_map>


#include <rclcpp/rclcpp.hpp>  // NOLINT
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "cyberdog_maps_manager/maps_manager.hpp"

namespace cyberdog
{
namespace maps_manager
{

void Test_Request2Json()
{
  INFO("#################### [case 1] ##########################");
  MapsManager::CommandRequest request;
  std::vector<std::string> maps_name {
    "map1"
  };

  std::vector<uint64_t> ids {
    10001
  };

  request.maps_name = maps_name;
  // request.ids = ids;
  request.name_code = 1001;
  request.timestamp = 12345;

  auto manager = std::make_shared<MapsManager>();
  auto result = manager->ToString(request);
  INFO("json : %s", result.c_str());
}

void Test_RequestArray2Json()
{

  INFO("#################### [case 2 array] ##########################");
  MapsManager::CommandRequest request;
  std::vector<std::string> maps_name {
    "map1",
    "map2",
    "map3",
    "map4",
    "map5",
  };

  std::vector<uint64_t> ids {
    10001,
    20002,
    30003,
    40004,
    50005
  };

  request.maps_name = maps_name;
  request.ids = ids;
  request.name_code = 1001;
  request.timestamp = 12345;

  auto manager = std::make_shared<MapsManager>();
  auto result = manager->ToString(request);
  INFO("json : %s", result.c_str());
}


void Run()
{
  Test_Request2Json();
  Test_RequestArray2Json();
}

}  // namespace maps_manager
}  // namespace cyberdog

int main(int argc, char ** argv)
{
  LOGGER_MAIN_INSTANCE("MapManagerTest");
  rclcpp::init(argc, argv);
  cyberdog::maps_manager::Run();
  return 0;
}
