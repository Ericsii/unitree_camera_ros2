// Copyright 2025 Yunlong Feng
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

#pragma once

#include <memory>
#include <string>

#include <camera_info_manager/camera_info_manager.hpp>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

namespace unitree_camera_ros2 {

class UnitreeCameraNode : public rclcpp::Node {
public:
  explicit UnitreeCameraNode(const rclcpp::NodeOptions &options);
  ~UnitreeCameraNode() override;

  GstFlowReturn gst_callback(GstAppSink *sink);

private:
  void get_parameters();

  bool init_gstreamer();
  void start_pipeline();
  void stop_pipeline();

  std::string camera_name_;
  std::string camera_frame_id_;
  std::string camera_info_url_;

  std::unique_ptr<image_transport::CameraPublisher> camera_publisher_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  std::string udp_multicast_ip_;
  int udp_multicast_port_;
  std::string multicast_interface_;
  GstElement *pipeline_;
  GstElement *appsink_;
  GstAppSinkCallbacks appsink_callbacks_;
};
} // namespace unitree_camera_ros2