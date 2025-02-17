// Copyright 2022 Intelligent Robotics Lab
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


#ifndef MOCAP4R2_VICON_DRIVER__MOCAP4R2_VICON_DRIVER_HPP_
#define MOCAP4R2_VICON_DRIVER__MOCAP4R2_VICON_DRIVER_HPP_

#include <iostream>
#include <sstream>
#include <map>
#include <string>
#include <memory>
#include <chrono>
#include <vector>

#include "ViconDataStreamSDK_CPP/DataStreamClient.h"

#include "rclcpp/rclcpp.hpp"
#include "mocap4r2_control/ControlledLifecycleNode.hpp"

#include "mocap_interfaces/msg/marker.hpp"
#include "mocap_interfaces/msg/marker_array.hpp"
#include "mocap_interfaces/msg/rigid_body_array.hpp"

namespace mocap4r2_vicon_driver
{

class ViconDriverNode : public mocap4r2_control::ControlledLifecycleNode
{
public:
  ViconDriverNode();

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
  void initParameters();

protected:
  ViconDataStreamSDK::CPP::Client client;
  rclcpp_lifecycle::LifecyclePublisher<mocap_interfaces::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp_lifecycle::LifecyclePublisher<mocap_interfaces::msg::RigidBodyArray>::SharedPtr
    rigid_bodies_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string stream_mode_;
  std::string host_name_;
  std::string frame_id_;
  int frameCount_ {0};

  void process_frame();

  void control_start(const mocap4r2_control_msgs::msg::Control::SharedPtr msg) override;
  void control_stop(const mocap4r2_control_msgs::msg::Control::SharedPtr msg) override;


  std::string Enum2String(const ViconDataStreamSDK::CPP::Direction::Enum i_Direction);
  std::string Enum2String(const ViconDataStreamSDK::CPP::Result::Enum i_result);
};

}  // namespace mocap4r2_vicon_driver

#endif  // MOCAP4R2_VICON_DRIVER__MOCAP4R2_VICON_DRIVER_HPP_
