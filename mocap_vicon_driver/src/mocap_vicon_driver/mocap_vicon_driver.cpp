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


#include "mocap_vicon_driver/mocap_vicon_driver.hpp"

using namespace std::chrono_literals;


// The vicon driver node has differents parameters to initialized with the mocap_vicon_driver_params.yaml
ViconDriverNode::ViconDriverNode()
: ControlledLifecycleNode("mocap_vicon_driver_node")
{
  declare_parameter<std::string>("stream_mode", "ClientPull");
  declare_parameter<std::string>("host_name", "192.168.10.1:801");
  declare_parameter<std::string>("frame_id", "vicon_world");
}

// In charge of choose the different driver options related and provided by the Vicon SDK
void ViconDriverNode::set_settings_vicon()
{
  ViconDataStreamSDK::CPP::Result::Enum result(ViconDataStreamSDK::CPP::Result::Unknown);
  if (stream_mode_ == "ServerPush") {
    result = client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush).Result;
  } else if (stream_mode_ == "ClientPull") {
    result = client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull).Result;
  } else {
    RCLCPP_FATAL(get_logger(), "Unknown stream mode -- options are ServerPush, ClientPull");
    rclcpp::shutdown();
  }

  RCLCPP_INFO(
    get_logger(), "Setting Stream Mode to %s : %s",
    stream_mode_.c_str(), Enum2String(result).c_str());

  client.SetAxisMapping(
    ViconDataStreamSDK::CPP::Direction::Forward,
    ViconDataStreamSDK::CPP::Direction::Left, ViconDataStreamSDK::CPP::Direction::Up);
  ViconDataStreamSDK::CPP::Output_GetAxisMapping _Output_GetAxisMapping = client.GetAxisMapping();

  RCLCPP_INFO(
    get_logger(),
    "Axis Mapping: X-%s Y-%s Z-%s",
    Enum2String(_Output_GetAxisMapping.XAxis).c_str(),
    Enum2String(_Output_GetAxisMapping.YAxis).c_str(),
    Enum2String(_Output_GetAxisMapping.ZAxis).c_str());

  client.EnableSegmentData();

  RCLCPP_INFO(
    get_logger(), "IsSegmentDataEnabled? %s",
    client.IsSegmentDataEnabled().Enabled ? "true" : "false");

  ViconDataStreamSDK::CPP::Output_GetVersion _Output_GetVersion = client.GetVersion();

  RCLCPP_INFO(
    get_logger(), "Version: %d.%d.%d",
    _Output_GetVersion.Major,
    _Output_GetVersion.Minor,
    _Output_GetVersion.Point
  );

}


// In charge of the transition of the lifecycle node
void ViconDriverNode::control_start(const mocap_control_msgs::msg::Control::SharedPtr msg)
{
}

// In charge of the transition of the lifecycle node
void ViconDriverNode::control_stop(const mocap_control_msgs::msg::Control::SharedPtr msg)
{
}

// In charge of get the Vicon information and convert it to vicon_msgs
void ViconDriverNode::process_frame()
{
  if (marker_pub_->get_subscription_count() == 0) {
    return;
  }

  ViconDataStreamSDK::CPP::Output_GetFrame ans = client.GetFrame();

  if (ans.Result == ViconDataStreamSDK::CPP::Result::Success) {
    ViconDataStreamSDK::CPP::Output_GetFrameNumber OutputFrameNum = client.GetFrameNumber();
    ViconDataStreamSDK::CPP::Output_GetFrameRate OutputFrameRate = client.GetFrameRate();

    client.EnableMarkerData();
    client.EnableUnlabeledMarkerData();

    mocap_msgs::msg::Markers markers_msg;
    markers_msg.header.stamp = now();
    markers_msg.frame_number = frameCount++;

    unsigned int SubjectCount = client.GetSubjectCount().SubjectCount;
    for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount; ++SubjectIndex)
    {
      std::string this_subject_name = client.GetSubjectName(SubjectIndex).SubjectName;

      unsigned int num_subject_markers = client.GetMarkerCount(this_subject_name).MarkerCount;
      n_markers_ += num_subject_markers;

      for (unsigned int MarkerIndex = 0; MarkerIndex < num_subject_markers; ++MarkerIndex)
      {
        mocap_msgs::msg::Marker this_marker;
        this_marker.id_type = mocap_msgs::msg::Marker::USE_NAME;
        this_marker.marker_name = client.GetMarkerName(this_subject_name, MarkerIndex).MarkerName;

        Output_GetMarkerGlobalTranslation _Output_GetMarkerGlobalTranslation =
          client.GetMarkerGlobalTranslation(this_subject_name, this_marker.marker_name);

        this_marker.translation.x = _Output_GetMarkerGlobalTranslation.Translation[0];
        this_marker.translation.y = _Output_GetMarkerGlobalTranslation.Translation[1];
        this_marker.translation.z = _Output_GetMarkerGlobalTranslation.Translation[2];

        markers_msg.markers.push_back(this_marker);
      }

      marker_pub_->publish(markers_msg);
    }
  }
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


// The next Callbacks are used to manage behavior in the different states of the lifecycle node.
CallbackReturnT
ViconDriverNode::on_configure(const rclcpp_lifecycle::State &)
{
  initParameters();

  marker_pub_ = create_publisher<mocap_msgs::msg::Markers>("/markers", 1000);

  auto stat = client.Connect(host_name_).Result;

  if (cstat == ViconDataStreamSDK::CPP::Result::Success && lient.IsConnected().Connected) {
    RCLCPP_INFO(get_logger(), "... connected!");
    return CallbackReturnT::SUCCESS;
  } else {
    RCLCPP_ERROR(get_logger(), "... not connected :(  [%s]", Enum2String(stat).c_str());
    return CallbackReturnT::FAILURE;
  }
}

CallbackReturnT
ViconDriverNode::on_activate(const rclcpp_lifecycle::State &)
{
  update_pub_->on_activate();
  marker_pub_->on_activate();

  set_settings_vicon();

  timer_ = create_wall_timer(10ms, std::bind(&ViconDriverNode::process_frame, this));

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ViconDriverNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  update_pub_->on_deactivate();
  marker_pub_->on_deactivate();

  timer_ = nullptr;

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ViconDriverNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  client.Disconnect();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ViconDriverNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  client.Disconnect();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ViconDriverNode::on_error(const rclcpp_lifecycle::State &)
{
  client.Disconnect();
  return CallbackReturnT::SUCCESS;
}


// Init the necessary parameters to use the Vicon SDK.
void ViconDriverNode::initParameters()
{
  get_parameter<std::string>("stream_mode", stream_mode_);
  get_parameter<std::string>("host_name", host_name_);
  get_parameter<std::string>("frame_id", frame_id_);
}

// Transform the Vicon SDK enumerations to strings
std::string Enum2String(const ViconDataStreamSDK::CPP::Direction::Enum i_Direction)
{
  switch (i_Direction) {
    case ViconDataStreamSDK::CPP::Direction::Forward:
      return "Forward";
    case ViconDataStreamSDK::CPP::Direction::Backward:
      return "Backward";
    case ViconDataStreamSDK::CPP::Direction::Left:
      return "Left";
    case ViconDataStreamSDK::CPP::Direction::Right:
      return "Right";
    case ViconDataStreamSDK::CPP::Direction::Up:
      return "Up";
    case ViconDataStreamSDK::CPP::Direction::Down:
      return "Down";
    default:
      return "Unknown";
  }
}

// Transform the Vicon SDK enumerations to strings
std::string Enum2String(const ViconDataStreamSDK::CPP::Result::Enum i_result)
{
  switch (i_result) {
    case ViconDataStreamSDK::CPP::Result::ClientAlreadyConnected:
      return "ClientAlreadyConnected";
    case ViconDataStreamSDK::CPP::Result::ClientConnectionFailed:
      return "ClientConnectionFailed";
    case ViconDataStreamSDK::CPP::Result::CoLinearAxes:
      return "CoLinearAxes";
    case ViconDataStreamSDK::CPP::Result::InvalidDeviceName:
      return "InvalidDeviceName";
    case ViconDataStreamSDK::CPP::Result::InvalidDeviceOutputName:
      return "InvalidDeviceOutputName";
    case ViconDataStreamSDK::CPP::Result::InvalidHostName:
      return "InvalidHostName";
    case ViconDataStreamSDK::CPP::Result::InvalidIndex:
      return "InvalidIndex";
    case ViconDataStreamSDK::CPP::Result::InvalidLatencySampleName:
      return "InvalidLatencySampleName";
    case ViconDataStreamSDK::CPP::Result::InvalidMarkerName:
      return "InvalidMarkerName";
    case ViconDataStreamSDK::CPP::Result::InvalidMulticastIP:
      return "InvalidMulticastIP";
    case ViconDataStreamSDK::CPP::Result::InvalidSegmentName:
      return "InvalidSegmentName";
    case ViconDataStreamSDK::CPP::Result::InvalidSubjectName:
      return "InvalidSubjectName";
    case ViconDataStreamSDK::CPP::Result::LeftHandedAxes:
      return "LeftHandedAxes";
    case ViconDataStreamSDK::CPP::Result::NoFrame:
      return "NoFrame";
    case ViconDataStreamSDK::CPP::Result::NotConnected:
      return "NotConnected";
    case ViconDataStreamSDK::CPP::Result::NotImplemented:
      return "NotImplemented";
    case ViconDataStreamSDK::CPP::Result::ServerAlreadyTransmittingMulticast:
      return "ServerAlreadyTransmittingMulticast";
    case ViconDataStreamSDK::CPP::Result::ServerNotTransmittingMulticast:
      return "ServerNotTransmittingMulticast";
    case ViconDataStreamSDK::CPP::Result::Success:
      return "Success";
    case ViconDataStreamSDK::CPP::Result::Unknown:
      return "Unknown";
    default:
      return "unknown";
  }
}
