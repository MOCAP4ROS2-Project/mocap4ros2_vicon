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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "mocap4r2_vicon_driver/vicon_conversion.hpp"

using mocap4r2_interfaces::msg::Marker;
using mocap4r2_interfaces::msg::Segment;
using mocap4r2_vicon_driver::ViconConverter;
using mocap4r2_vicon_driver::ViconFrame;

namespace
{

ViconFrame::Segment seg(
  const std::string & name, const std::string & parent, double z, bool occluded = false)
{
  ViconFrame::Segment s;
  s.name = name;
  s.parent = parent;
  s.position = {1.0, 2.0, z};
  s.rotation = {0.0, 0.0, std::sin(0.3), std::cos(0.3)};
  s.occluded = occluded;
  return s;
}

ViconFrame::Marker mk(const std::string & name, double x, bool occluded = false)
{
  ViconFrame::Marker m;
  m.name = name;
  m.position = {x, 0.0, 0.0};
  m.occluded = occluded;
  return m;
}

ViconFrame two_subjects()
{
  ViconFrame f;
  f.frame_number = 500;
  ViconFrame::Subject robot;
  robot.name = "robot";
  robot.root_segment = "robot";
  robot.segments = {seg("robot", "", 0.1)};
  robot.markers = {mk("m1", 0.1), mk("m2", 0.2, true)};
  ViconFrame::Subject human;
  human.name = "alice";
  human.root_segment = "pelvis";
  human.segments = {seg("thigh", "pelvis", 0.8), seg("pelvis", "", 1.0), seg("shin", "thigh", 0.4)};
  human.markers = {mk("LASI", 0.3)};
  f.subjects = {robot, human};
  f.unlabeled_markers = {{5.0, 5.0, 5.0}};
  return f;
}

}  // namespace

TEST(ViconConversion, subjects_become_bodies_with_their_own_markers)
{
  ViconConverter c;
  const auto out = c.convert(two_subjects());
  EXPECT_EQ(out.frame_number, 500u);
  ASSERT_EQ(out.rigid_bodies.size(), 2u);
  const auto & robot = out.rigid_bodies[0];
  EXPECT_EQ(robot.name, "robot");
  EXPECT_TRUE(robot.tracking_valid);
  EXPECT_DOUBLE_EQ(robot.pose.position.z, 0.1);
  EXPECT_NEAR(robot.pose.orientation.z, std::sin(0.3), 1e-12);
  // Only its own markers (the old driver gave every segment all markers).
  ASSERT_EQ(robot.markers.size(), 2u);
  EXPECT_EQ(robot.markers[0].name, "robot:m1");
  EXPECT_TRUE(robot.markers[1].occluded);
  const auto & alice = out.rigid_bodies[1];
  EXPECT_EQ(alice.name, "alice");
  EXPECT_DOUBLE_EQ(alice.pose.position.z, 1.0);  // root segment, not the first one
  ASSERT_EQ(alice.markers.size(), 1u);
  EXPECT_LT(alice.mean_error, 0.0f);

  ASSERT_EQ(out.markers.size(), 4u);
  EXPECT_EQ(out.markers[3].id, Marker::UNKNOWN_ID);
  EXPECT_TRUE(out.markers[3].name.empty());
}

TEST(ViconConversion, multi_segment_subjects_are_skeletons)
{
  ViconConverter c;
  const auto out = c.convert(two_subjects());
  ASSERT_EQ(out.skeletons.size(), 1u);  // the robot has one segment
  const auto & sk = out.skeletons[0];
  EXPECT_EQ(sk.name, "alice");
  ASSERT_EQ(sk.segments.size(), 3u);
  EXPECT_EQ(sk.segments[0].name, "thigh");
  EXPECT_EQ(sk.segments[0].parent_id, sk.segments[1].id);  // parent pelvis
  EXPECT_EQ(sk.segments[1].parent_id, Segment::NO_PARENT);
  EXPECT_EQ(sk.segments[2].parent_id, sk.segments[0].id);
  EXPECT_TRUE(sk.tracking_valid);
}

TEST(ViconConversion, ids_are_stable_across_frames_and_order_changes)
{
  ViconConverter c;
  auto f = two_subjects();
  const auto a = c.convert(f);
  std::swap(f.subjects[0], f.subjects[1]);
  const auto b = c.convert(f);
  EXPECT_EQ(a.rigid_bodies[0].id, b.rigid_bodies[1].id);
  EXPECT_EQ(a.rigid_bodies[0].markers[0].id, b.rigid_bodies[1].markers[0].id);
  EXPECT_NE(a.rigid_bodies[0].id, a.rigid_bodies[1].id);
}

TEST(ViconConversion, occluded_root_and_invalid_values)
{
  ViconConverter c;
  auto f = two_subjects();
  f.subjects[0].segments[0].occluded = true;
  f.subjects[1].segments[2].occluded = true;
  f.subjects[1].markers[0].position[0] = std::numeric_limits<double>::quiet_NaN();
  auto out = c.convert(f);
  EXPECT_FALSE(out.rigid_bodies[0].tracking_valid);
  EXPECT_DOUBLE_EQ(out.rigid_bodies[0].pose.orientation.w, 1.0);  // no stale pose
  EXPECT_FALSE(out.skeletons[0].segments[2].tracking_valid);
  EXPECT_FALSE(out.skeletons[0].tracking_valid);
  EXPECT_TRUE(out.rigid_bodies[1].markers[0].occluded);  // NaN marker flagged

  f = two_subjects();
  f.subjects[0].segments[0].position[1] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(c.convert(f).rigid_bodies[0].tracking_valid);

  f = two_subjects();
  f.subjects[0].segments[0].rotation = {0.0, 0.0, 0.0, 0.0};
  out = c.convert(f);
  EXPECT_DOUBLE_EQ(out.rigid_bodies[0].pose.orientation.w, 1.0);
  f.subjects[0].segments[0].rotation = {0.0, 0.0, 3.0, 4.0};
  out = c.convert(f);
  EXPECT_NEAR(out.rigid_bodies[0].pose.orientation.z, 0.6, 1e-12);
}

TEST(ViconConversion, missing_root_and_empty_subjects)
{
  ViconConverter c;
  ViconFrame f;
  ViconFrame::Subject s;
  s.name = "box";
  s.root_segment = "unknown";
  s.segments = {seg("a", "", 0.5)};
  ViconFrame::Subject empty;
  empty.name = "nothing";
  f.subjects = {s, empty};
  const auto out = c.convert(f);
  EXPECT_DOUBLE_EQ(out.rigid_bodies[0].pose.position.z, 0.5);  // first segment used
  EXPECT_FALSE(out.rigid_bodies[1].tracking_valid);
  EXPECT_TRUE(out.skeletons.empty());
}
