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

#ifndef SIM_ARM_TARGET_LOCATION_POSITIONING_H
#define SIM_ARM_TARGET_LOCATION_POSITIONING_H

#include <memory>
#include <vector>

namespace sim_arm_target_location {

struct Target2D {
  float x;
  float y;
  std::string class_name;

  Target2D(float x, float y, std::string class_name_)
      : x(x),
        y(y),
        class_name(class_name_) {}
};

struct Location3D {
  float x;
  float y;
  float z;

  Location3D(float x, float y, float z)
      : x(x),
        y(y),
        z(z) {}
};

bool PositionSolving(const std::shared_ptr<std::vector<std::shared_ptr<Target2D>>> solving_target, const int num,
      const std::shared_ptr<Location3D> target_location);
      
}  // namespace sim_arm_target_location


#endif //SIM_ARM_TARGET_LOCATION_POSITIONING_H