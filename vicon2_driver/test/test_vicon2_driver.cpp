// Copyright (c) 2020, Intelligent Robotics Lab
// Copyright (c) 2020, Airelectronics.
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

#include <string>
#include <list>
#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "vicon2_driver/vicon2_driver.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using std::placeholders::_1;

class TestViconDriver : public ViconDriverNode
{
public:
  TestViconDriver()
  : ref_stream_mode_(stream_mode_),
    ref_host_name_(host_name_),
    ref_tf_ref_frame_id_(tf_ref_frame_id_),
    ref_tracked_frame_suffix_(tracked_frame_suffix_),
    ref_publish_markers_(publish_markers_),
    ref_marker_data_enabled_(marker_data_enabled_),
    ref_unlabeled_marker_data_enabled_(unlabeled_marker_data_enabled_),
    ref_lastFrameNumber_(lastFrameNumber_),
    ref_frameCount_(frameCount_),
    ref_droppedFrameCount_(droppedFrameCount_),
    ref_n_markers_(n_markers_),
    ref_n_unlabeled_markers_(n_unlabeled_markers_),
    ref_qos_history_policy_(qos_history_policy_),
    ref_qos_reliability_policy_(qos_reliability_policy_),
    ref_qos_depth_(qos_depth_)
  {
  }

  void test_init_parameters()
  {
    initParameters();
  }

  std::string & ref_stream_mode_;
  std::string & ref_host_name_;
  std::string & ref_tf_ref_frame_id_;
  std::string & ref_tracked_frame_suffix_;
  bool & ref_publish_markers_;
  bool & ref_marker_data_enabled_;
  bool & ref_unlabeled_marker_data_enabled_;
  int & ref_lastFrameNumber_;
  int & ref_frameCount_;
  int & ref_droppedFrameCount_;
  int & ref_n_markers_;
  int & ref_n_unlabeled_markers_;
  std::string & ref_qos_history_policy_;
  std::string & ref_qos_reliability_policy_;
  int & ref_qos_depth_;
};

TEST(UtilsTest, test_vicon2_params)
{
  auto vicon2_node = std::make_shared<TestViconDriver>();
  auto test_node = rclcpp::Node::make_shared("vicon2_test_node");

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::executor::ExecutorArgs(), 4);
  exe.add_node(vicon2_node->get_node_base_interface());
  exe.add_node(test_node->get_node_base_interface());

  auto set_parameters_results = vicon2_node->set_parameters(
  {
    rclcpp::Parameter("stream_mode", "ServerPush"),
    rclcpp::Parameter("host_name", "123.456.789:123"),
    rclcpp::Parameter("tf_ref_frame_id", "my_world"),
    rclcpp::Parameter("tracked_frame_suffix", "my_vicon"),
    rclcpp::Parameter("publish_markers", true),
    rclcpp::Parameter("marker_data_enabled", true),
    rclcpp::Parameter("unlabeled_marker_data_enabled", true),
    rclcpp::Parameter("lastFrameNumber", 1),
    rclcpp::Parameter("frameCount", 2),
    rclcpp::Parameter("droppedFrameCount", 3),
    rclcpp::Parameter("n_markers", 4),
    rclcpp::Parameter("n_unlabeled_markers", 5),
    rclcpp::Parameter("qos_history_policy", "keep_last"),
    rclcpp::Parameter("qos_reliability_policy", "reliable"),
    rclcpp::Parameter("qos_depth", 6),
  });

  vicon2_node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE));
  exe.spin_some();

  EXPECT_EQ(State::PRIMARY_STATE_INACTIVE, vicon2_node->get_current_state().id());

  vicon2_node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE));

  ASSERT_EQ(vicon2_node->ref_stream_mode_, "ServerPush");
  ASSERT_EQ(vicon2_node->ref_host_name_, "123.456.789:123");
  ASSERT_EQ(vicon2_node->ref_tf_ref_frame_id_, "my_world");
  ASSERT_EQ(vicon2_node->ref_tracked_frame_suffix_, "my_vicon");
  ASSERT_EQ(vicon2_node->ref_publish_markers_, true);
  ASSERT_EQ(vicon2_node->ref_marker_data_enabled_, true);
  ASSERT_EQ(vicon2_node->ref_unlabeled_marker_data_enabled_, true);
  ASSERT_EQ(vicon2_node->ref_lastFrameNumber_, 1);
  ASSERT_EQ(vicon2_node->ref_frameCount_, 2);
  ASSERT_EQ(vicon2_node->ref_droppedFrameCount_, 3);
  ASSERT_EQ(vicon2_node->ref_n_markers_, 4);
  ASSERT_EQ(vicon2_node->ref_n_unlabeled_markers_, 5);
  ASSERT_EQ(vicon2_node->ref_qos_history_policy_, "keep_last");
  ASSERT_EQ(vicon2_node->ref_qos_reliability_policy_, "reliable");
  ASSERT_EQ(vicon2_node->ref_qos_depth_, 6);
}


int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
