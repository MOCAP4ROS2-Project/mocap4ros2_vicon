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

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "mocap4r2_vicon_driver/vicon_driver.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace mocap4r2_vicon_driver
{

ViconDriver::ViconDriver(const rclcpp::NodeOptions & options)
: ViconDriver(options, &make_sdk_source)
{
}

ViconDriver::ViconDriver(const rclcpp::NodeOptions & options, SourceFactory factory)
: MocapSystemNode("vicon_driver", "vicon", options),
  factory_(std::move(factory))
{
  declare_parameter<std::string>("host_name", "192.168.10.1:801");
  declare_parameter<double>("timeout", 2.0);
  declare_parameter<bool>("compensate_latency", true);
  mocap4r2_control::declare_frame_publisher_parameters(*this);
}

ViconDriver::~ViconDriver()
{
  stop_thread();
  source_.reset();
}

ViconDriver::CallbackReturn
ViconDriver::on_system_configure()
{
  const auto host = get_parameter("host_name").as_string();
  const double timeout = get_parameter("timeout").as_double();
  if (host.empty() || timeout <= 0.0) {
    RCLCPP_ERROR(get_logger(), "host_name must not be empty and timeout must be > 0");
    return CallbackReturn::FAILURE;
  }
  compensate_latency_ = get_parameter("compensate_latency").as_bool();
  converter_ = ViconConverter();
  source_ = factory_();
  source_->connect(
    host, std::chrono::milliseconds(static_cast<int64_t>(timeout * 1000.0)));  // throws
  server_label_ = "Vicon DataStream at " + host;
  RCLCPP_INFO(
    get_logger(), "connected to Vicon DataStream at %s (SDK %s)", host.c_str(),
    source_->version().c_str());
  publisher_ = std::make_unique<mocap4r2_control::FramePublisher>(*this);
  return CallbackReturn::SUCCESS;
}

ViconDriver::CallbackReturn
ViconDriver::on_system_activate()
{
  running_ = true;
  thread_ = std::thread(&ViconDriver::acquisition_loop, this);
  return CallbackReturn::SUCCESS;
}

ViconDriver::CallbackReturn
ViconDriver::on_system_deactivate()
{
  stop_thread();
  return CallbackReturn::SUCCESS;
}

ViconDriver::CallbackReturn
ViconDriver::on_system_cleanup()
{
  stop_thread();
  if (source_) {
    source_->disconnect();
  }
  source_.reset();
  publisher_.reset();
  return CallbackReturn::SUCCESS;
}

void
ViconDriver::on_system_shutdown()
{
  stop_thread();
  source_.reset();
}

void
ViconDriver::stop_thread()
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
}

void
ViconDriver::acquisition_loop()
{
  ViconFrame frame;
  uint64_t last = 0;
  bool first = true;
  while (running_) {
    // Short timeout so that deactivation is never blocked by a silent server.
    if (!source_->next_frame(frame, std::chrono::milliseconds(100))) {
      if (!source_->connected()) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "connection to Vicon lost");
      }
      continue;
    }
    const auto received = now();
    if (!first && frame.frame_number == last) {
      continue;  // same frame delivered twice
    }
    first = false;
    last = frame.frame_number;
    if (frame.frame_rate > 0.0) {
      frame_rate_ = frame.frame_rate;
    }
    auto out = converter_.convert(frame);
    const double latency = compensate_latency_ && frame.latency > 0.0 ? frame.latency : 0.0;
    out.capture_time = received - rclcpp::Duration::from_seconds(latency);
    publisher_->publish(std::move(out));
  }
}

}  // namespace mocap4r2_vicon_driver

RCLCPP_COMPONENTS_REGISTER_NODE(mocap4r2_vicon_driver::ViconDriver)
