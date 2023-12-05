// Copyright (c) 2022，Horizon Robotics.
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

#include <iostream>
#include <sstream>

#include "sim_arm_target_location/positioning.h"

namespace sim_arm_target_location
{

  static std::string target_names[3] = {"num1_cube", "num2_cube", "num3_cube"};

  bool PositionSolving(const std::shared_ptr<std::vector<std::shared_ptr<Target2D>>> solving_target, const int num,
                       const std::shared_ptr<Location3D> target_location)
  {
    bool res = false;
    for (auto target : *solving_target)
    {
      if (target->class_name == target_names[num - 1])
      {
        //相机分辨率为1280*720
        //相机坐标系下的位置：像素坐标离图片中心的值 / 相机焦距(固定值：507.872) * 相机坐标系到物体中心的距离(固定值：0.517194)
        //相机相对机械臂的位置偏移，x轴为0.335294，y轴为0
        target_location->x = 0.335294 + (360 - target->y) / 507.872 * 0.517194;
        target_location->y = (640 - target->x) / 507.872 * 0.517194;
        target_location->z = 0.045;
        res = true;
        break;
      }
    }
    return res;
  }

} // namespace sim_arm_target_location
