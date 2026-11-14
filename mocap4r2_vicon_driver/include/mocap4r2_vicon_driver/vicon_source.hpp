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

#ifndef MOCAP4R2_VICON_DRIVER__VICON_SOURCE_HPP_
#define MOCAP4R2_VICON_DRIVER__VICON_SOURCE_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace mocap4r2_vicon_driver
{

// One frame as read from a Vicon system (Nexus, Tracker, Shogun), in metres,
// Z-up (the SDK axis mapping is set to Forward-Left-Up).
struct ViconFrame
{
  struct Segment
  {
    std::string name;
    std::string parent;                 // empty for the root
    bool occluded{false};
    std::array<double, 3> position{};
    std::array<double, 4> rotation{0.0, 0.0, 0.0, 1.0};  // x, y, z, w
  };
  struct Marker
  {
    std::string name;
    std::string segment;
    bool occluded{false};
    std::array<double, 3> position{};
  };
  struct Subject
  {
    std::string name;
    std::string root_segment;
    double quality{-1.0};               // object quality, negative if unknown
    std::vector<Segment> segments;
    std::vector<Marker> markers;
  };

  uint32_t frame_number{0};
  double frame_rate{0.0};
  double latency{0.0};                  // seconds, as reported by the system
  std::vector<Subject> subjects;
  std::vector<std::array<double, 3>> unlabeled_markers;
};

// Source of Vicon frames: the DataStream SDK, or a fake one in tests.
class ViconSource
{
public:
  virtual ~ViconSource() = default;
  // Connect to host[:port]. Throws std::runtime_error on failure.
  virtual void connect(const std::string & host, std::chrono::milliseconds timeout) = 0;
  // Wait for the next frame. Returns false on timeout or when disconnected.
  virtual bool next_frame(ViconFrame & frame, std::chrono::milliseconds timeout) = 0;
  virtual bool connected() const = 0;
  virtual void disconnect() = 0;
  virtual std::string version() const = 0;
};

std::unique_ptr<ViconSource> make_sdk_source();

}  // namespace mocap4r2_vicon_driver

#endif  // MOCAP4R2_VICON_DRIVER__VICON_SOURCE_HPP_
