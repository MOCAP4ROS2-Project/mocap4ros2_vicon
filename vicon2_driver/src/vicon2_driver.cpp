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

#include "vicon2_driver/vicon2_driver.hpp"

// Transform the Vicon SDK enumerations to strings
string Enum2String(const Direction::Enum i_Direction)
{
  switch (i_Direction) {
    case Direction::Forward:
      return "Forward";
    case Direction::Backward:
      return "Backward";
    case Direction::Left:
      return "Left";
    case Direction::Right:
      return "Right";
    case Direction::Up:
      return "Up";
    case Direction::Down:
      return "Down";
    default:
      return "Unknown";
  }
}

// Transform the Vicon SDK enumerations to strings
string Enum2String(const Result::Enum i_result)
{
  switch (i_result) {
    case Result::ClientAlreadyConnected:
      return "ClientAlreadyConnected";
    case Result::ClientConnectionFailed:
      return "ClientConnectionFailed";
    case Result::CoLinearAxes:
      return "CoLinearAxes";
    case Result::InvalidDeviceName:
      return "InvalidDeviceName";
    case Result::InvalidDeviceOutputName:
      return "InvalidDeviceOutputName";
    case Result::InvalidHostName:
      return "InvalidHostName";
    case Result::InvalidIndex:
      return "InvalidIndex";
    case Result::InvalidLatencySampleName:
      return "InvalidLatencySampleName";
    case Result::InvalidMarkerName:
      return "InvalidMarkerName";
    case Result::InvalidMulticastIP:
      return "InvalidMulticastIP";
    case Result::InvalidSegmentName:
      return "InvalidSegmentName";
    case Result::InvalidSubjectName:
      return "InvalidSubjectName";
    case Result::LeftHandedAxes:
      return "LeftHandedAxes";
    case Result::NoFrame:
      return "NoFrame";
    case Result::NotConnected:
      return "NotConnected";
    case Result::NotImplemented:
      return "NotImplemented";
    case Result::ServerAlreadyTransmittingMulticast:
      return "ServerAlreadyTransmittingMulticast";
    case Result::ServerNotTransmittingMulticast:
      return "ServerNotTransmittingMulticast";
    case Result::Success:
      return "Success";
    case Result::Unknown:
      return "Unknown";
    default:
      return "unknown";
  }
}

// The vicon driver node has differents parameters to initialized with the vicon2_driver_params.yaml
ViconDriverNode::ViconDriverNode(const rclcpp::NodeOptions node_options)
: ControlledLifecycleNode("vicon2_driver_node", node_options)
{
  declare_parameter<std::string>("stream_mode", "ClientPull");
  declare_parameter<std::string>("host_name", "192.168.10.1:801");
  declare_parameter<std::string>("tf_ref_frame_id", "vicon_world");
  declare_parameter<std::string>("tracked_frame_suffix", "vicon");
  declare_parameter<bool>("publish_markers", false);
  declare_parameter<bool>("marker_data_enabled", false);
  declare_parameter<bool>("unlabeled_marker_data_enabled", false);
  declare_parameter<int>("lastFrameNumber", 0);
  declare_parameter<int>("frameCount", 0);
  declare_parameter<int>("droppedFrameCount", 0);
  declare_parameter<int>("n_markers", 0);
  declare_parameter<int>("n_unlabeled_markers", 0);
  declare_parameter<std::string>("qos_history_policy", "keep_all");
  declare_parameter<std::string>("qos_reliability_policy", "best_effort");
  declare_parameter<int>("qos_depth", 10);
}

// In charge of choose the different driver options related and provided by the Vicon SDK
void ViconDriverNode::set_settings_vicon()
{
  Result::Enum result(Result::Unknown);
  if (stream_mode_ == "ServerPush") {
    result = client.SetStreamMode(StreamMode::ServerPush).Result;
  } else if (stream_mode_ == "ClientPull") {
    result = client.SetStreamMode(StreamMode::ClientPull).Result;
  } else {
    RCLCPP_FATAL(get_logger(), "Unknown stream mode -- options are ServerPush, ClientPull");
    rclcpp::shutdown();
  }

  RCLCPP_INFO(
    get_logger(), "Setting Stream Mode to %s : %s",
    stream_mode_.c_str(), Enum2String(result).c_str());

  client.SetAxisMapping(
    Direction::Forward,
    Direction::Left, Direction::Up);
  Output_GetAxisMapping _Output_GetAxisMapping = client.GetAxisMapping();

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

  Output_GetVersion _Output_GetVersion = client.GetVersion();

  RCLCPP_INFO(
    get_logger(), "Version: %d.%d.%d",
    _Output_GetVersion.Major,
    _Output_GetVersion.Minor,
    _Output_GetVersion.Point
  );

  unlabeled_counter_ = 0;

}

// Start the vicon_driver_node if the Vicon system is OK.
void ViconDriverNode::start_vicon()
{
  set_settings_vicon();
  // rclcpp::WallRate d(1.0 / 240.0);
  auto period = std::chrono::milliseconds(100);
  rclcpp::Rate d(period);
  while (rclcpp::ok()) {
    Output_GetFrame ans = client.GetFrame();
    while (ans.Result != Result::Success && rclcpp::ok()) {
      RCLCPP_WARN(get_logger(), "getFrame returned [%s]", Enum2String(ans.Result).c_str());
      d.sleep();
      ans = client.GetFrame();
    } 
    now_time_ = this->now();
    process_frame();
  }
}

// Stop the vicon_driver_node if the lifecycle node state is shutdown.
bool ViconDriverNode::stop_vicon()
{
  RCLCPP_INFO(get_logger(), "Disconnecting from Vicon DataStream SDK");
  client.Disconnect();
  RCLCPP_INFO(get_logger(), "... disconnected");
  return true;
}

// In charge of the transition of the lifecycle node
void ViconDriverNode::control_start(const mocap_control_msgs::msg::Control::SharedPtr msg) {
  trigger_transition(rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
}

// In charge of the transition of the lifecycle node
void ViconDriverNode::control_stop(const mocap_control_msgs::msg::Control::SharedPtr msg) {
  trigger_transition(rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
}

// In charge of get the Vicon information and convert it to vicon_msgs
void ViconDriverNode::process_frame()
{
  static rclcpp::Time lastTime;
  Output_GetFrameNumber OutputFrameNum = client.GetFrameNumber();
  Output_GetFrameRate OutputFrameRate = client.GetFrameRate();
  RCLCPP_WARN(get_logger(), "Frame rate: %f", OutputFrameRate.FrameRateHz);

  int frameDiff = 0;
  if (lastFrameNumber_ != 0) {
    frameDiff = OutputFrameNum.FrameNumber - lastFrameNumber_;
    frameCount_ += frameDiff;
    if ((frameDiff) > 1) {
      droppedFrameCount_ += frameDiff;
      double droppedFramePct = static_cast<double>(droppedFrameCount_ / frameCount_ * 100);

      RCLCPP_DEBUG(
        get_logger(),
        "%d more (total %d / %d, %f %%) frame(s) dropped. Consider adjusting rates",
        frameDiff, droppedFrameCount_, frameCount_, droppedFramePct);
    }
  }
  lastFrameNumber_ = OutputFrameNum.FrameNumber;

  if (frameDiff != 0) {
    rclcpp::Duration vicon_latency(std::chrono::duration<double>(client.GetLatencyTotal().Total));
    if (publish_markers_) {
      process_markers(now_time_ - vicon_latency, lastFrameNumber_);
    }
    lastTime = now_time_;
  }
}

// Transform the information provided by the Vicon system into vicon_msgs and publish the information
void ViconDriverNode::process_markers(const rclcpp::Time & frame_time, unsigned int vicon_frame_num)
{
  int marker_cnt = 0;
  if (!marker_data_enabled_) {
    marker_data_enabled_ = true;
    client.EnableMarkerData();

    RCLCPP_INFO(
      get_logger(), "IsMarkerDataEnabled? %s",
      client.IsMarkerDataEnabled().Enabled ? "true" : "false");
  }

  if (!unlabeled_marker_data_enabled_) {
    unlabeled_marker_data_enabled_ = true;
    client.EnableUnlabeledMarkerData();

  RCLCPP_INFO(
      get_logger(), "IsUnlabeledMarkerDataEnabled? %s",
      client.IsUnlabeledMarkerDataEnabled().Enabled ? "true" : "false");
  }

  n_markers_ = 0;
  mocap_msgs::msg::Markers markers_msg;
  markers_msg.header.stamp = frame_time;
  markers_msg.frame_number = vicon_frame_num;

  // Count the number of subjects
  unsigned int SubjectCount = client.GetSubjectCount().SubjectCount;
  RCLCPP_DEBUG(get_logger(), "Subjects: (%u)", SubjectCount);
  for( unsigned int SubjectIndex = 0 ; SubjectIndex < SubjectCount ; ++SubjectIndex )
  {
    // Get the subject name
    std::string SubjectName = client.GetSubjectName( SubjectIndex ).SubjectName;
    unsigned int MarkerCount = client.GetMarkerCount(SubjectName).MarkerCount;
    RCLCPP_DEBUG( get_logger(), "# Markers: %d", MarkerCount);

    n_markers_ += MarkerCount;
    // n_unlabeled_markers_ = MarkerCount;
    for (unsigned int MarkerIndex = 0; MarkerIndex < MarkerCount; ++MarkerIndex)
    {
      // Get the marker name
      std::string MarkerName = client.GetMarkerName( SubjectName, MarkerIndex ).MarkerName;
      // Get the global marker translationSegmentPublisher
      Output_GetMarkerGlobalTranslation
        _Output_GetMarkerGlobalTranslation =
        client.GetMarkerGlobalTranslation(SubjectName,MarkerName );

      if (_Output_GetMarkerGlobalTranslation.Result ==  Result::Success)
      {
        mocap_msgs::msg::Marker this_marker;
        this_marker.translation.x = _Output_GetMarkerGlobalTranslation.Translation[0];
        this_marker.translation.y = _Output_GetMarkerGlobalTranslation.Translation[1];
        this_marker.translation.z = _Output_GetMarkerGlobalTranslation.Translation[2];
        //this_marker.occluded = _Output_GetMarkerGlobalTranslation.Occluded;
        //this_marker.marker_name = MarkerName;
        //this_marker.subject_name = SubjectName;
        //this_marker.segment_name = 
        //                       client.GetMarkerParentName(SubjectName, MarkerName).SegmentName;

        this_marker.index = getMarkerIndex(MarkerName);
        markers_msg.markers.push_back(this_marker);

        marker_to_tf(this_marker, marker_cnt, frame_time, MarkerName);
        marker_cnt++;
      } else {
        RCLCPP_WARN(
          get_logger(),
          "GetUnlabeledMarkerGlobalTranslation failed (result = %s)",
          Enum2String(_Output_GetMarkerGlobalTranslation.Result).c_str());
      }
    }
    
    if (!marker_pub_->is_activated()) {
      RCLCPP_WARN(
        get_logger(),
        "Lifecycle publisher is currently inactive. Messages are not published.");
    }
    marker_pub_->publish(markers_msg);
  }
}

// 
int ViconDriverNode::getMarkerIndex(std::string marker_name)
{
    int ans;

    // unlabelled markers will get an unique number 
    if (marker_name.size()==0){
          ans = unlabeled_counter_++;
    } else {

      auto search = markers_list_.find(marker_name);
      if (search != markers_list_.end()) {
          ans = search->second;
      } else {
          ans = markers_list_.size();
          markers_list_.insert({marker_name, ans});        
      }     

    }
    return ans;
}

// Transform and publish the information previously procesed by the process_markers and converted in ROS-TFs.
void ViconDriverNode::marker_to_tf(
  mocap_msgs::msg::Marker marker,
  int marker_num, const rclcpp::Time & frame_time, std::string marker_name)
{
  tf2::Transform transform;
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  string tracked_frame;
  geometry_msgs::msg::TransformStamped tf_msg;

  //if (marker.occluded){
  //  return;
  //}

  // occluded markers are reported at 0,0,0
  if ( (marker.translation.x == 0.0) & (marker.translation.y == 0.0) & (marker.translation.z == 0.0) ){
    return;
  }



  transform.setOrigin(
    tf2::Vector3(
      marker.translation.x / 1000,
      marker.translation.y / 1000,
      marker.translation.z / 1000));
  transform.setRotation(tf2::Quaternion(0, 0, 0, 1));

  stringstream marker_num_str;
  marker_num_str << marker_num;
  if (marker_name.size()==0){
    tracked_frame = tracked_frame_suffix_ + "/marker_tf_" + marker_num_str.str();
  } else{
    tracked_frame = tracked_frame_suffix_ + "/" + marker_name;
  }

  tf_msg.header.stamp = frame_time;
  tf_msg.header.frame_id = tf_ref_frame_id_;
  tf_msg.child_frame_id = tracked_frame;
  tf_msg.transform.translation.x = transform.getOrigin().x();
  tf_msg.transform.translation.y = transform.getOrigin().y();
  tf_msg.transform.translation.z = transform.getOrigin().z();
  tf_msg.transform.rotation.x = transform.getRotation().x();
  tf_msg.transform.rotation.y = transform.getRotation().y();
  tf_msg.transform.rotation.z = transform.getRotation().z();
  tf_msg.transform.rotation.w = transform.getRotation().w();

  transforms.push_back(tf_msg);
  tf_broadcaster_->sendTransform(transforms);
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


// The next Callbacks are used to manage behavior in the different states of the lifecycle node.
CallbackReturnT
ViconDriverNode::on_configure(const rclcpp_lifecycle::State &)
{
  initParameters();

  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

  auto rmw_qos_history_policy = name_to_history_policy_map.find(qos_history_policy_);
  auto rmw_qos_reliability_policy = name_to_reliability_policy_map.find(qos_reliability_policy_);
  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
      // The history policy determines how messages are saved until taken by
      // the reader.
      // KEEP_ALL saves all messages until they are taken.
      // KEEP_LAST enforces a limit on the number of messages that are saved,
      // specified by the "depth" parameter.
      rmw_qos_history_policy->second,
      // Depth represents how many messages to store in history when the
      // history policy is KEEP_LAST.
      qos_depth_
  ));
  // The reliability policy can be reliable, meaning that the underlying transport layer will try
  // ensure that every message gets received in order, or best effort, meaning that the transport
  // makes no guarantees about the order or reliability of delivery.
  qos.reliability(rmw_qos_reliability_policy->second);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
    "/vicon2_driver/change_state");

  marker_pub_ = create_publisher<mocap_msgs::msg::Markers>(
    tracked_frame_suffix_ + "/markers", 100);

  update_pub_ = create_publisher<std_msgs::msg::Empty>(
    "/vicon2_driver/update_notify", qos);

  RCLCPP_INFO(get_logger(), "Configured!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ViconDriverNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  update_pub_->on_activate();
  marker_pub_->on_activate();
  connect_vicon();
  RCLCPP_INFO(get_logger(), "Activated!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ViconDriverNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  update_pub_->on_deactivate();
  marker_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ViconDriverNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  /* Clean up stuff */
  RCLCPP_INFO(get_logger(), "Cleaned up!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ViconDriverNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
  /* Shut down stuff */
  RCLCPP_INFO(get_logger(), "Shutted down!\n");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ViconDriverNode::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

  return CallbackReturnT::SUCCESS;
}

// In charge of find and connect the driver with the Vicon SDK.
bool ViconDriverNode::connect_vicon()
{
  RCLCPP_WARN(
    get_logger(),
    "Trying to connect to Vicon DataStream SDK at %s ...", host_name_.c_str());

  auto stat = client.Connect(host_name_).Result;
  if (stat == Result::Success) {
    RCLCPP_INFO(get_logger(), "... connected!");
    start_vicon();
  } else {
    RCLCPP_INFO(get_logger(), "... not connected :(  [%s]", Enum2String(stat).c_str());
  }

  return client.IsConnected().Connected;
}

// Init the necessary parameters to use the Vicon SDK.
void ViconDriverNode::initParameters()
{
  get_parameter<std::string>("stream_mode", stream_mode_);
  get_parameter<std::string>("host_name", host_name_);
  get_parameter<std::string>("tf_ref_frame_id", tf_ref_frame_id_);
  get_parameter<std::string>("tracked_frame_suffix", tracked_frame_suffix_);
  get_parameter<bool>("publish_markers", publish_markers_);
  get_parameter<bool>("marker_data_enabled", marker_data_enabled_);
  get_parameter<bool>("unlabeled_marker_data_enabled", unlabeled_marker_data_enabled_);
  get_parameter<int>("lastFrameNumber", lastFrameNumber_);
  get_parameter<int>("frameCount", frameCount_);
  get_parameter<int>("droppedFrameCount", droppedFrameCount_);
  get_parameter<int>("n_markers", n_markers_);
  get_parameter<int>("n_unlabeled_markers", n_unlabeled_markers_);
  get_parameter<std::string>("qos_history_policy", qos_history_policy_);
  get_parameter<std::string>("qos_reliability_policy", qos_reliability_policy_);
  get_parameter<int>("qos_depth", qos_depth_);


  RCLCPP_INFO(
    get_logger(),
    "Param stream_mode: %s", stream_mode_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Param host_name: %s", host_name_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Param tf_ref_frame_id: %s", tf_ref_frame_id_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Param tracked_frame_suffix: %s", tracked_frame_suffix_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Param publish_markers: %s", publish_markers_ ? "true" : "false");
  RCLCPP_INFO(
    get_logger(),
    "Param marker_data_enabled: %s", marker_data_enabled_ ? "true" : "false");
  RCLCPP_INFO(
    get_logger(),
    "Param unlabeled_marker_data_enabled: %s", unlabeled_marker_data_enabled_ ? "true" : "false");
  RCLCPP_INFO(
    get_logger(),
    "Param lastFrameNumber: %d", lastFrameNumber_);
  RCLCPP_INFO(
    get_logger(),
    "Param frameCount: %d", frameCount_);
  RCLCPP_INFO(
    get_logger(),
    "Param droppedFrameCount: %d", droppedFrameCount_);
  RCLCPP_INFO(
    get_logger(),
    "Param n_markers: %d", n_markers_);
  RCLCPP_INFO(
    get_logger(),
    "Param n_unlabeled_markers: %d", n_unlabeled_markers_);
  RCLCPP_INFO(
    get_logger(),
    "Param qos_history_policy: %s", qos_history_policy_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Param qos_reliability_policy: %s", qos_reliability_policy_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Param qos_depth: %d", qos_depth_);
}
