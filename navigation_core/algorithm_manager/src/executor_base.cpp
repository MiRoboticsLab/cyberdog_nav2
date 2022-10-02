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

#include <memory>
#include <vector>

#include "algorithm_manager/executor_base.hpp"

namespace cyberdog
{
namespace algorithm
{
  LifecyleNav2LifecyleMgrClientMap ExecutorBase::lifecycle_client_map_;
  std::unordered_map<LifecycleClientID, std::string> ExecutorBase::lifecycle_client_ids_;
}
}