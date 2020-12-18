// Copyright 2019 Intelligent Robotics Lab
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
//
// Author: David Vargas Frutos <david.vargas@urjc.es>

#ifndef VICON2_DRIVER__VICON2_DRIVER_HPP_
#define VICON2_DRIVER__VICON2_DRIVER_HPP_

#include <iostream>
#include <sstream>
#include <map>
#include <string>
#include <memory>
#include <chrono>
#include <vector>

#include "rclcpp/time.hpp"

#include "mocap_msgs/msg/marker.hpp"
#include "mocap_msgs/msg/markers.hpp"
#include "std_msgs/msg/empty.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_logging.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "tf2/buffer_core.h"
#include "tf2_ros/transform_broadcaster.h"

#include "DataStreamClient.h"

#include "mocap_control/ControlledLifecycleNode.hpp"


class ViconDriverNode : public rclcpp_lifecycle::LifecycleNode, public mocap_control::ControlledLifecycleNode
{
public:
  explicit ViconDriverNode(
    const rclcpp::NodeOptions options =
    rclcpp::NodeOptions().parameter_overrides(
      std::vector<rclcpp::Parameter> {
    rclcpp::Parameter("use_sim_time", true)
  }));

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);
  bool connect_vicon();
  void set_settings_vicon();
  void start_vicon();
  bool stop_vicon();
  void initParameters();

protected:
  ViconDataStreamSDK::CPP::Client client;
  // rclcpp::Node::SharedPtr vicon_node;
  // std::shared_ptr<rclcpp::SyncParametersClient> parameters_client;
  rclcpp::Time now_time;
  std::string myParam;
  // rclcpp::Publisher<mocap4ros_msgs::msg::Markers>::SharedPtr marker_pub_;
  rclcpp_lifecycle::LifecyclePublisher<mocap_msgs::msg::Markers>::SharedPtr marker_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string stream_mode_;
  std::string host_name_;
  std::string tf_ref_frame_id_;
  std::string tracked_frame_suffix_;
  bool publish_markers_;
  bool marker_data_enabled_;
  bool unlabeled_marker_data_enabled_;
  int lastFrameNumber_;
  int frameCount_;
  int droppedFrameCount_;
  int n_markers_;
  int n_unlabeled_markers_;
  std::string qos_history_policy_;
  std::string qos_reliability_policy_;
  int qos_depth_;

  void process_frame();
  void process_markers(const rclcpp::Time & frame_time, unsigned int vicon_frame_num);
  void marker_to_tf(
    mocap_msgs::msg::Marker marker,
    int marker_num, const rclcpp::Time & frame_time);

  void control_start() override;
  void control_stop() override;

  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr update_pub_;
};

static
std::map<std::string, rmw_qos_reliability_policy_t> name_to_reliability_policy_map = {
  {"reliable", RMW_QOS_POLICY_RELIABILITY_RELIABLE},
  {"best_effort", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT}
};

static
std::map<std::string, rmw_qos_history_policy_t> name_to_history_policy_map = {
  {"keep_last", RMW_QOS_POLICY_HISTORY_KEEP_LAST},
  {"keep_all", RMW_QOS_POLICY_HISTORY_KEEP_ALL}
};

#endif  // VICON2_DRIVER__VICON2_DRIVER_HPP_
