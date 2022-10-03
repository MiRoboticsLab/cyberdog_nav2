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
#include <iostream>

namespace cyberdog
{
namespace map_manager
{

struct Position
{
  Position(double xx, double yy, double zz)
  : x(xx), y(yy), z(zz) {}

  double x;
  double y;
  double z;

  double x() {return x;}
  double y() {return y;}
  double z() {return z;}
};

struct Quaternion
{
  Quaternion(double xx, double yy, double zz, double ww)
  : x(xx), y(yy), z(zz), w(ww) {}

  double x;
  double y;
  double z;
  double w;

  double x() {return x;}
  double y() {return y;}
  double z() {return z;}
  double w() {return w;}
};

struct LabelTag
{
  std::string tag_name;
  Position position;
  Quaternion quaternion;

  LabelTag(
    const std::string & t_tag_name, const Position & t_position,
    const Quaternion & t_quaternion)
  : tag_name{t_tag_name},
    position{t_position},
    quaternion{t_quaternion} {}

  void SetTagName(const std::string & t_tag_name)
  {
    tag_name = t_tag_name;
  }

  void SetPosition(double x, double y, double z)
  {
    position.x = x;
    position.y = y;
    position.z = z;
  }

  void SetPosition(const Position & t_position)
  {
    position = t_position;
  }

  void SetQuaternion(double x, double y, double z, double w)
  {
    quaternion.x = x;
    quaternion.y = y;
    quaternion.z = z;
    quaternion.w = w;
  }

  void SetQuaternion(const Quaternion & t_quaternion)
  {
    quaternion = t_quaternion;
  }

  std::string tag_name()
  {
    return tag_name;
  }

  Position position()
  {
    return position;
  }

  Quaternion quaternion()
  {
    return quaternion;
  }
};


std::ostream & operator<<(std::ostream & os, const LabelTag & label)
{
  os << "[ tag_name : " << label.tag_name() << "]" <<
    "\n" <<
    "   position : [" <<
    label.position.x() << "," <<
    label.position.y() << "," <<
    label.position.z() << "]" <<
    "\n" <<
    "   quaternion : [" <<
    label.quaternion.x() << "," <<
    label.quaternion.y() << "," <<
    label.quaternion.z() << "," <<
    label.quaternion.w() << "]";
  return os;
}
}  // namespace namespace map_manager
}  // namespace cyberdog

#endif  // CYBERDOG_MAP_MANAGER__LABEL_TAG_HPP_
