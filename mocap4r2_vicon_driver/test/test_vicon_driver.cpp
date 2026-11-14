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

#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "mocap4r2_interfaces/msg/rigid_bodies.hpp"
#include "mocap4r2_vicon_driver/vicon_driver.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using mocap4r2_interfaces::msg::RigidBodies;
using mocap4r2_vicon_driver::ViconDriver;
using mocap4r2_vicon_driver::ViconFrame;
using mocap4r2_vicon_driver::ViconSource;

// Shared state of the fake Vicon system, visible to the test.
struct FakeVicon
{
  std::mutex mutex;
  std::deque<ViconFrame> frames;
  bool reachable{true};
  std::atomic<int> connects{0};
  std::atomic<int> disconnects{0};
  std::atomic<bool> stalled{false};  // connected but no frames
};

class FakeSource : public ViconSource
{
public:
  explicit FakeSource(std::shared_ptr<FakeVicon> v)
  : v_(std::move(v)) {}
  void connect(const std::string & host, std::chrono::milliseconds) override
  {
    if (!v_->reachable || host.empty()) {
      throw std::runtime_error("unreachable");
    }
    v_->connects++;
    connected_ = true;
  }
  bool next_frame(ViconFrame & f, std::chrono::milliseconds timeout) override
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      if (!v_->stalled) {
        std::lock_guard<std::mutex> lock(v_->mutex);
        if (!v_->frames.empty()) {
          f = v_->frames.front();
          v_->frames.pop_front();
          return true;
        }
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }
  bool connected() const override {return connected_;}
  void disconnect() override
  {
    if (connected_) {
      v_->disconnects++;
    }
    connected_ = false;
  }
  std::string version() const override {return "fake";}

private:
  std::shared_ptr<FakeVicon> v_;
  bool connected_{false};
};

class ViconDriverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    vicon_ = std::make_shared<FakeVicon>();
    client_ = rclcpp::Node::make_shared("vicon_test_client");
    sub_ = client_->create_subscription<RigidBodies>(
      "/rigid_bodies", rclcpp::QoS(100).reliable(),
      [this](RigidBodies::ConstSharedPtr m) {bodies_.push_back(*m);});
    executor_.add_node(client_);
  }

  void TearDown() override
  {
    if (driver_) {
      executor_.remove_node(driver_->get_node_base_interface());
    }
  }

  std::shared_ptr<ViconDriver> make(std::vector<rclcpp::Parameter> params = {})
  {
    auto v = vicon_;
    driver_ = std::make_shared<ViconDriver>(
      rclcpp::NodeOptions().parameter_overrides(params),
      [v] {return std::make_unique<FakeSource>(v);});
    executor_.add_node(driver_->get_node_base_interface());
    return driver_;
  }

  void push(uint32_t n, double latency = 0.0)
  {
    ViconFrame f;
    f.frame_number = n;
    f.frame_rate = 200.0;
    f.latency = latency;
    ViconFrame::Subject s;
    s.name = "robot";
    s.root_segment = "robot";
    ViconFrame::Segment seg;
    seg.name = "robot";
    seg.position = {1.0, 2.0, 0.1};
    s.segments = {seg};
    f.subjects = {s};
    std::lock_guard<std::mutex> lock(vicon_->mutex);
    vicon_->frames.push_back(f);
  }

  bool spin_until(const std::function<bool()> & cond, std::chrono::seconds timeout = 10s)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some(10ms);
      if (cond()) {
        return true;
      }
    }
    return cond();
  }

  std::shared_ptr<FakeVicon> vicon_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr client_;
  std::shared_ptr<ViconDriver> driver_;
  rclcpp::Subscription<RigidBodies>::SharedPtr sub_;
  std::vector<RigidBodies> bodies_;
};

TEST_F(ViconDriverTest, unreachable_system_fails_configure)
{
  vicon_->reachable = false;
  auto d = make();
  d->trigger_transition(Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(d->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(ViconDriverTest, frames_keep_sdk_numbers_and_skip_duplicates)
{
  auto d = make({{"compensate_latency", true}});
  d->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_TRUE(spin_until([&] {return sub_->get_publisher_count() > 0;}));
  d->trigger_transition(Transition::TRANSITION_ACTIVATE);
  for (uint32_t n : {10u, 11u, 11u, 12u, 15u}) {
    push(n, 0.02);
  }
  ASSERT_TRUE(spin_until([&] {return bodies_.size() >= 4;}));
  spin_until([] {return false;}, 1s);
  std::vector<uint64_t> numbers;
  for (const auto & b : bodies_) {
    numbers.push_back(b.frame_number);
  }
  EXPECT_EQ(numbers, (std::vector<uint64_t>{10, 11, 12, 15}));
  EXPECT_EQ(bodies_[0].rigid_bodies[0].name, "robot");
  const double age = (client_->now() - rclcpp::Time(bodies_.back().header.stamp)).seconds();
  EXPECT_GE(age, 0.019);  // latency compensated
  // The gap 12 -> 15 is counted as dropped frames.
  EXPECT_EQ(d->stats().take(0.0).total_dropped, 2u);
}

TEST_F(ViconDriverTest, deactivation_is_not_blocked_by_a_silent_system)
{
  auto d = make();
  d->trigger_transition(Transition::TRANSITION_CONFIGURE);
  d->trigger_transition(Transition::TRANSITION_ACTIVATE);
  vicon_->stalled = true;
  std::this_thread::sleep_for(200ms);
  const auto t0 = std::chrono::steady_clock::now();
  d->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 2s);
  EXPECT_EQ(d->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(ViconDriverTest, cleanup_disconnects_and_reconfigure_works)
{
  auto d = make();
  for (int i = 0; i < 3; i++) {
    d->trigger_transition(Transition::TRANSITION_CONFIGURE);
    ASSERT_EQ(d->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
    d->trigger_transition(Transition::TRANSITION_ACTIVATE);
    d->trigger_transition(Transition::TRANSITION_DEACTIVATE);
    d->trigger_transition(Transition::TRANSITION_CLEANUP);
  }
  EXPECT_EQ(vicon_->connects, 3);
  EXPECT_EQ(vicon_->disconnects, 3);
}

TEST_F(ViconDriverTest, invalid_parameters)
{
  for (const auto & p : std::vector<std::vector<rclcpp::Parameter>>{
    {{"host_name", ""}}, {{"timeout", -1.0}}})
  {
    auto d = make(p);
    d->trigger_transition(Transition::TRANSITION_CONFIGURE);
    EXPECT_EQ(d->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
    executor_.remove_node(d->get_node_base_interface());
    driver_.reset();
  }
  EXPECT_EQ(vicon_->connects, 0);
}

TEST_F(ViconDriverTest, destroy_while_streaming)
{
  auto d = make();
  d->trigger_transition(Transition::TRANSITION_CONFIGURE);
  d->trigger_transition(Transition::TRANSITION_ACTIVATE);
  for (uint32_t n = 1; n < 200; n++) {
    push(n);
  }
  executor_.remove_node(d->get_node_base_interface());
  driver_.reset();
  d.reset();
  SUCCEED();
}

TEST(ViconSdk, connecting_to_nothing_fails_quickly)
{
  // The real SDK against a closed port.
  auto d = std::make_shared<ViconDriver>(
    rclcpp::NodeOptions().parameter_overrides(
      {{"host_name", "127.0.0.1:1"}, {"timeout", 0.5}}));
  const auto t0 = std::chrono::steady_clock::now();
  d->trigger_transition(Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(d->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 10s);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
