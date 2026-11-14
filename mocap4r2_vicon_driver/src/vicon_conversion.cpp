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

#include <cmath>
#include <map>
#include <string>

#include "mocap4r2_vicon_driver/vicon_conversion.hpp"

namespace mocap4r2_vicon_driver
{

namespace
{

geometry_msgs::msg::Pose to_pose(const ViconFrame::Segment & s)
{
  geometry_msgs::msg::Pose p;
  p.position.x = s.position[0];
  p.position.y = s.position[1];
  p.position.z = s.position[2];
  double q[4] = {s.rotation[0], s.rotation[1], s.rotation[2], s.rotation[3]};
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n > 1e-9 && std::isfinite(n)) {
    p.orientation.x = q[0] / n;
    p.orientation.y = q[1] / n;
    p.orientation.z = q[2] / n;
    p.orientation.w = q[3] / n;
  } else {
    p.orientation.w = 1.0;
  }
  return p;
}

bool finite(const std::array<double, 3> & v)
{
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

}  // namespace

int32_t
ViconConverter::id_of(std::map<std::string, int32_t> & ids, const std::string & name)
{
  const auto it = ids.find(name);
  if (it != ids.end()) {
    return it->second;
  }
  const auto id = static_cast<int32_t>(ids.size() + 1);
  ids[name] = id;
  return id;
}

mocap4r2_control::MocapFrame
ViconConverter::convert(const ViconFrame & frame)
{
  using mocap4r2_interfaces::msg::Marker;
  using mocap4r2_interfaces::msg::RigidBody;
  using mocap4r2_interfaces::msg::Segment;
  using mocap4r2_interfaces::msg::Skeleton;

  mocap4r2_control::MocapFrame out;
  out.frame_number = frame.frame_number;

  for (const auto & subject : frame.subjects) {
    std::vector<Marker> markers;
    for (const auto & m : subject.markers) {
      Marker mk;
      mk.name = subject.name + ":" + m.name;
      mk.id = id_of(marker_ids_, mk.name);
      mk.occluded = m.occluded || !finite(m.position);
      mk.position.x = m.position[0];
      mk.position.y = m.position[1];
      mk.position.z = m.position[2];
      mk.residual = -1.0f;
      markers.push_back(mk);
    }
    out.markers.insert(out.markers.end(), markers.begin(), markers.end());

    const ViconFrame::Segment * root = nullptr;
    for (const auto & seg : subject.segments) {
      if (seg.name == subject.root_segment) {
        root = &seg;
      }
    }
    if (!root && !subject.segments.empty()) {
      root = &subject.segments.front();
    }

    RigidBody rb;
    rb.id = id_of(subject_ids_, subject.name);
    rb.name = subject.name;
    rb.tracking_valid = root && !root->occluded && finite(root->position);
    if (root && rb.tracking_valid) {
      rb.pose = to_pose(*root);
    } else {
      rb.pose.orientation.w = 1.0;
    }
    rb.mean_error = -1.0f;  // Vicon reports a quality, not a metric error
    rb.markers = markers;
    out.rigid_bodies.push_back(rb);

    if (subject.segments.size() > 1) {
      Skeleton sk;
      sk.id = rb.id;
      sk.name = subject.name;
      sk.tracking_valid = true;
      std::map<std::string, int32_t> seg_ids;
      for (size_t i = 0; i < subject.segments.size(); i++) {
        seg_ids[subject.segments[i].name] = static_cast<int32_t>(i + 1);
      }
      for (const auto & seg : subject.segments) {
        Segment s;
        s.id = seg_ids[seg.name];
        s.name = seg.name;
        const auto parent = seg_ids.find(seg.parent);
        s.parent_id = parent != seg_ids.end() && seg.parent != seg.name ? parent->second :
          Segment::NO_PARENT;
        s.tracking_valid = !seg.occluded && finite(seg.position);
        if (s.tracking_valid) {
          s.pose = to_pose(seg);
        } else {
          s.pose.orientation.w = 1.0;
        }
        sk.tracking_valid = sk.tracking_valid && s.tracking_valid;
        sk.segments.push_back(s);
      }
      out.skeletons.push_back(sk);
    }
  }

  for (const auto & u : frame.unlabeled_markers) {
    Marker mk;
    mk.id = Marker::UNKNOWN_ID;
    mk.position.x = u[0];
    mk.position.y = u[1];
    mk.position.z = u[2];
    mk.residual = -1.0f;
    out.markers.push_back(mk);
  }
  return out;
}

}  // namespace mocap4r2_vicon_driver
