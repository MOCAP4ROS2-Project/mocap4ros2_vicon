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

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "DataStreamClient.h"
#include "mocap4r2_vicon_driver/vicon_source.hpp"

namespace mocap4r2_vicon_driver
{

namespace
{

namespace sdk = ViconDataStreamSDK::CPP;

class SdkViconSource : public ViconSource
{
public:
  ~SdkViconSource() override {disconnect();}

  void connect(const std::string & host, std::chrono::milliseconds timeout) override
  {
    client_.SetConnectionTimeout(
      static_cast<unsigned int>(std::max<int64_t>(timeout.count(), 10)));
    const auto r = client_.Connect(host).Result;
    if (r != sdk::Result::Success) {
      throw std::runtime_error("cannot connect to Vicon DataStream at " + host);
    }
    // Pre-fetch keeps GetFrame short (never blocked by a silent server), and a
    // buffer of one frame never delivers stale data after a pause.
    client_.SetStreamMode(sdk::StreamMode::ClientPullPreFetch);
    client_.SetBufferSize(1);
    client_.SetAxisMapping(sdk::Direction::Forward, sdk::Direction::Left, sdk::Direction::Up);
    client_.EnableSegmentData();
    client_.EnableMarkerData();
    client_.EnableUnlabeledMarkerData();
  }

  bool next_frame(ViconFrame & f, std::chrono::milliseconds timeout) override
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (client_.IsConnected().Connected) {
      if (client_.GetFrame().Result == sdk::Result::Success) {
        const auto n = client_.GetFrameNumber().FrameNumber;
        if (!have_last_ || n != last_) {
          have_last_ = true;
          last_ = n;
          read(f);
          return true;
        }
      }
      if (std::chrono::steady_clock::now() > deadline) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }

  bool connected() const override {return client_.IsConnected().Connected;}

  void disconnect() override
  {
    if (client_.IsConnected().Connected) {
      client_.Disconnect();
    }
  }

  std::string version() const override
  {
    const auto v = client_.GetVersion();
    return std::to_string(v.Major) + "." + std::to_string(v.Minor) + "." +
           std::to_string(v.Point);
  }

private:
  void read(ViconFrame & f)
  {
    f = ViconFrame();
    f.frame_number = client_.GetFrameNumber().FrameNumber;
    f.frame_rate = client_.GetFrameRate().FrameRateHz;
    f.latency = client_.GetLatencyTotal().Total;

    const unsigned int n_subjects = client_.GetSubjectCount().SubjectCount;
    for (unsigned int s = 0; s < n_subjects; s++) {
      ViconFrame::Subject subject;
      subject.name = client_.GetSubjectName(s).SubjectName;
      subject.root_segment = client_.GetSubjectRootSegmentName(subject.name).SegmentName;
      const auto q = client_.GetObjectQuality(subject.name);
      if (q.Result == sdk::Result::Success) {
        subject.quality = q.Quality;
      }
      const unsigned int n_segments = client_.GetSegmentCount(subject.name).SegmentCount;
      for (unsigned int g = 0; g < n_segments; g++) {
        ViconFrame::Segment seg;
        seg.name = client_.GetSegmentName(subject.name, g).SegmentName;
        seg.parent = client_.GetSegmentParentName(subject.name, seg.name).SegmentName;
        const auto t = client_.GetSegmentGlobalTranslation(subject.name, seg.name);
        const auto r = client_.GetSegmentGlobalRotationQuaternion(subject.name, seg.name);
        seg.occluded = t.Occluded || r.Occluded || t.Result != sdk::Result::Success;
        for (int i = 0; i < 3; i++) {
          seg.position[i] = t.Translation[i] / 1000.0;  // mm -> m
        }
        for (int i = 0; i < 4; i++) {
          seg.rotation[i] = r.Rotation[i];
        }
        subject.segments.push_back(seg);
      }
      const unsigned int n_markers = client_.GetMarkerCount(subject.name).MarkerCount;
      for (unsigned int m = 0; m < n_markers; m++) {
        ViconFrame::Marker mk;
        mk.name = client_.GetMarkerName(subject.name, m).MarkerName;
        mk.segment = client_.GetMarkerParentName(subject.name, mk.name).SegmentName;
        const auto t = client_.GetMarkerGlobalTranslation(subject.name, mk.name);
        mk.occluded = t.Occluded || t.Result != sdk::Result::Success;
        for (int i = 0; i < 3; i++) {
          mk.position[i] = t.Translation[i] / 1000.0;
        }
        subject.markers.push_back(mk);
      }
      f.subjects.push_back(subject);
    }
    const unsigned int n_unlabeled = client_.GetUnlabeledMarkerCount().MarkerCount;
    for (unsigned int u = 0; u < n_unlabeled; u++) {
      const auto t = client_.GetUnlabeledMarkerGlobalTranslation(u);
      f.unlabeled_markers.push_back(
        {t.Translation[0] / 1000.0, t.Translation[1] / 1000.0, t.Translation[2] / 1000.0});
    }
  }

  sdk::Client client_;
  bool have_last_{false};
  unsigned int last_{0};
};

}  // namespace

std::unique_ptr<ViconSource>
make_sdk_source()
{
  return std::make_unique<SdkViconSource>();
}

}  // namespace mocap4r2_vicon_driver
