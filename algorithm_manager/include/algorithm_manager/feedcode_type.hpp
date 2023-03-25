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

#ifndef ALGORITHM_MANAGER__FEEDCODE_TYPE_HPP_
#define ALGORITHM_MANAGER__FEEDCODE_TYPE_HPP_


namespace cyberdog
{
namespace algorithm
{

namespace relocalization
{
/*
地图检查服务feedback_code ：
  - 正在检查地图：3100
  - 地图检查成功： 3101
  - 地图检查不可用，请重新建图： 3102
  - 地图后台构建中，请稍后： 3110
  - 地图检查服务出现异常，请重启机器狗： 3111
启动视觉SLAM相关服务feedback_code ：
  - 正在启动重定位依赖服务：3103
  - 启动重定位依赖服务成功： 3104
  - 启动重定位依赖服务失败，请重新尝试导航功能： 3105
启动依赖节点feedback_code：
  - 正在启动传感器依赖节点： 1000
  - 启动传感器依赖节点成功： 1001
  - 启动传感器依赖节点失败： 1002
重定位feedback_code：
  - 重定位功能超时，请重新启动重定位： 3106
  - 重定位功能失败继续尝试，遥控狗向前走一段距离： 3107
  - 重定位功能定位成功，点击地图或地图中的标签进行导航：3108
  - 重定位功能定位失败，请重试导航功能：3109
*/

// 地图检查服务feedback_code
constexpr int kMapChecking = 3100;
constexpr int kMapCheckingSuccess = 3101;
constexpr int kMapCheckingError = 3102;
constexpr int kMapCheckingUnderGoing = 3110;
constexpr int kMapCheckingException = 3111;

// 启动视觉SLAM相关服务feedback_code
constexpr int kServiceStarting = 3103;
constexpr int kServiceStartingSuccess = 3104;
constexpr int kServiceStartingError = 3105;

// 重定位feedback_code
constexpr int kSLAMTimeout = 3106;
constexpr int kSLAMFailedContinueTrying = 3107;
constexpr int kSLAMSuccess = 3108;
constexpr int kSLAMError = 3109;
}  // namespace relocalization

namespace navigation
{
/**
成功
- 导航启动成功，设置目标点成功，正在规划路径： 300
- 正在导航中： 307
- 到达目标点：308

- 失败
- 地图不存在：301
- 底层导航失败：
- 底层导航功能服务连接失败，请重新发送目标：302
- 发送目标点失败，请重新发送目标：303
- 底层导航功能失败，请重新发送目标：304
- 目标点为空，请重新选择目标：305
- 规划路径失败，请重新选择目标： 306

- 地图检查服务feedback_code ：
  - 正在检查地图：309
  - 地图检查成功： 310
  - 地图不存在，请重新建图： 311
*/

constexpr int kSuccessStartNavigation = 300;      // 导航启动成功，设置目标点成功，正在规划路径： 300  // NOLINT
constexpr int kSuccessStartingNavigation = 307;   // 正在导航中： 307
constexpr int kSuccessArriveTargetGoal = 308;     // 到达目标点：308

constexpr int kErrorConnectActionServer = 302;  // 底层导航功能服务连接失败，请重新发送目标：302
constexpr int kErrorSendGoalTarget = 303;       // 发送目标点失败，请重新发送目标：303
constexpr int kErrorNavigationAbort = 304;      // 底层导航功能失败，请重新发送目标：304
constexpr int kErrorTargetGoalIsEmpty = 305;    // 目标点为空，请重新选择目标：305

constexpr int kMapChecking = 309;
constexpr int kMapCheckingSuccess = 310;
constexpr int kMapErrorNotExist = 311;
}  // namespace navigation

}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__FEEDCODE_TYPE_HPP_
