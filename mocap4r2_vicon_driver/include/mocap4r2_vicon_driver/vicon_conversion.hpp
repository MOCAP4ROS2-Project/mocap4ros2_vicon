// Copyright 2026 Intelligent Robotics Lab
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

#ifndef MOCAP4R2_VICON_DRIVER__VICON_CONVERSION_HPP_
#define MOCAP4R2_VICON_DRIVER__VICON_CONVERSION_HPP_

#include <map>
#include <string>

#include "mocap4r2_control/frame_publisher.hpp"
#include "mocap4r2_vicon_driver/vicon_source.hpp"

namespace mocap4r2_vicon_driver
{

// Converts Vicon frames to MOCAP4ROS2 frames. Keeps stable numeric ids for the
// named markers and subjects of a session (Vicon only gives names).
class ViconConverter
{
public:
  // Every subject becomes a rigid body (its root segment); subjects with more
  // than one segment also become a skeleton.
  mocap4r2_control::MocapFrame convert(const ViconFrame & frame);

private:
  int32_t id_of(std::map<std::string, int32_t> & ids, const std::string & name);

  std::map<std::string, int32_t> subject_ids_;
  std::map<std::string, int32_t> marker_ids_;
};

}  // namespace mocap4r2_vicon_driver

#endif  // MOCAP4R2_VICON_DRIVER__VICON_CONVERSION_HPP_
