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

#ifndef MOCAP4R2_VICON_DRIVER__VICON_DRIVER_HPP_
#define MOCAP4R2_VICON_DRIVER__VICON_DRIVER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "mocap4r2_control/frame_publisher.hpp"
#include "mocap4r2_control/mocap_system_node.hpp"
#include "mocap4r2_vicon_driver/vicon_conversion.hpp"
#include "mocap4r2_vicon_driver/vicon_source.hpp"

namespace mocap4r2_vicon_driver
{

// MOCAP4ROS2 driver of Vicon systems (Nexus, Tracker, Shogun) through the
// DataStream SDK 1.13, with a dedicated acquisition thread.
class ViconDriver : public mocap4r2_control::MocapSystemNode
{
public:
  using SourceFactory = std::function<std::unique_ptr<ViconSource>()>;

  explicit ViconDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  // For tests: a driver reading from another source.
  ViconDriver(const rclcpp::NodeOptions & options, SourceFactory factory);
  ~ViconDriver() override;

protected:
  CallbackReturn on_system_configure() override;
  CallbackReturn on_system_activate() override;
  CallbackReturn on_system_deactivate() override;
  CallbackReturn on_system_cleanup() override;
  void on_system_shutdown() override;
  double nominal_frame_rate() const override {return frame_rate_;}
  std::string server() const override {return server_label_;}
  bool connected() const override {return source_ && source_->connected();}

private:
  void acquisition_loop();
  void stop_thread();

  SourceFactory factory_;
  std::unique_ptr<ViconSource> source_;
  std::unique_ptr<mocap4r2_control::FramePublisher> publisher_;
  ViconConverter converter_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<double> frame_rate_{0.0};
  bool compensate_latency_{true};
  std::string server_label_;
};

}  // namespace mocap4r2_vicon_driver

#endif  // MOCAP4R2_VICON_DRIVER__VICON_DRIVER_HPP_
