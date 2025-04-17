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

#include <unitree_camera_ros2/unitree_camera_node.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace unitree_camera_ros2 {

UnitreeCameraNode::UnitreeCameraNode(const rclcpp::NodeOptions &options)
    : Node("unitree_camera_node", options) {
  this->get_parameters();

  camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(
          this, camera_name_, camera_info_url_);
  if (camera_info_manager_->loadCameraInfo(camera_info_url_)) {
    RCLCPP_INFO(this->get_logger(), "Camera info loaded successfully");
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to load camera info from URL: %s",
                camera_info_url_.c_str());
  }
  camera_publisher_ = std::make_unique<image_transport::CameraPublisher>(
      image_transport::create_camera_publisher(
          this, camera_name_ + "/image_raw",
          rclcpp::SensorDataQoS().get_rmw_qos_profile()));

  if (!init_gstreamer()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize GStreamer");
    return;
  }
  start_pipeline();
}

UnitreeCameraNode::~UnitreeCameraNode() {
  stop_pipeline();
  RCLCPP_INFO(this->get_logger(), "GStreamer resources released");
}

void UnitreeCameraNode::get_parameters() {
  // Get parameters from the parameter server
  camera_name_ = this->declare_parameter("camera_name", "unitree_camera");
  camera_frame_id_ = this->declare_parameter("camera_frame_id",
                                             "unitree_camera_optical_frame");
  camera_info_url_ = this->declare_parameter("camera_info_url", "");
  udp_multicast_ip_ = this->declare_parameter("udp_multicast_ip", "230.1.1.1");
  udp_multicast_port_ = this->declare_parameter("udp_multicast_port", 1720);
  multicast_interface_ = this->declare_parameter("multicast_interface", "eth0");

  // Log the parameters
  RCLCPP_INFO(this->get_logger(), "Camera Name: %s", camera_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Camera Frame ID: %s",
              camera_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "UDP Multicast address: %s:%d",
              udp_multicast_ip_.c_str(), udp_multicast_port_);
  RCLCPP_INFO(this->get_logger(), "Multicast Interface: %s",
              multicast_interface_.c_str());
}

static GstFlowReturn trampoline_new_sample(GstAppSink *appsink,
                                           gpointer user_data) {
  if (!user_data) {
    g_printerr("Error: user_data is NULL in trampoline_new_sample\n");
    return GST_FLOW_ERROR;
  }
  // Cast user_data back to the std::function pointer
  auto *node = static_cast<UnitreeCameraNode *>(user_data);

  // Call the stored C++ lambda/function
  try {
    return node->gst_callback(appsink);
  } catch (const std::exception &e) {
    g_printerr("Exception caught in C++ callback: %s\n", e.what());
    return GST_FLOW_ERROR;
  } catch (...) {
    g_printerr("Unknown exception caught in C++ callback.\n");
    return GST_FLOW_ERROR;
  }
}

bool UnitreeCameraNode::init_gstreamer() {
  // Initialize GStreamer
  gst_init(nullptr, nullptr);

  // Create the GStreamer pipeline
  std::string pipeline_str =
      "udpsrc address=" + udp_multicast_ip_ +
      " port=" + std::to_string(udp_multicast_port_) +
      " multicast-iface=" + multicast_interface_ +
      " ! queue ! application/x-rtp, media=video, encoding-name=H264"
      " ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert"
      " ! video/x-raw,format=BGR"
      " ! appsink name=appsink emit-signals=true";

  GError *error = nullptr;
  // Create the GStreamer pipeline
  pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
  if (error) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create GStreamer pipeline: %s",
                 error->message);
    g_error_free(error);
    return false;
  }

  appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink");
  if (!appsink_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get appsink element");
    gst_object_unref(pipeline_);
    return false;
  }
  gst_app_sink_set_emit_signals(GST_APP_SINK(appsink_), true);
  gst_app_sink_set_drop(GST_APP_SINK(appsink_), true);
  gst_app_sink_set_drop(GST_APP_SINK(appsink_), TRUE);
  gst_app_sink_set_max_buffers(GST_APP_SINK(appsink_), 1);

  // Initialize callbacks with all required fields
  appsink_callbacks_.new_sample = trampoline_new_sample;

  // Set user data to this instance
  gst_app_sink_set_callbacks(GST_APP_SINK(appsink_), &appsink_callbacks_, this,
                             nullptr);

  return true;
}

void UnitreeCameraNode::start_pipeline() {
  // Start the GStreamer pipeline
  gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (gst_element_get_state(pipeline_, nullptr, nullptr, GST_CLOCK_TIME_NONE) ==
      GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set pipeline to PLAYING state");
  }
  RCLCPP_INFO(this->get_logger(), "GStreamer pipeline started");
}

void UnitreeCameraNode::stop_pipeline() {
  // Stop the GStreamer pipeline
  gst_element_set_state(pipeline_, GST_STATE_NULL);
  if (pipeline_ != nullptr) {
    gst_object_unref(pipeline_);
  }
  pipeline_ = nullptr;
  RCLCPP_INFO(this->get_logger(), "GStreamer pipeline stopped");
  if (appsink_ != nullptr) {
    gst_object_unref(appsink_);
  }
  appsink_ = nullptr;
}

GstFlowReturn UnitreeCameraNode::gst_callback(GstAppSink *sink) {
  // Get the sample from the appsink
  GstSample *sample = gst_app_sink_pull_sample(sink);
  if (!sample) {
    RCLCPP_ERROR(this->get_logger(), "Failed to pull sample from appsink");
    return GST_FLOW_ERROR;
  }

  // Process the sample
  GstBuffer *buffer = gst_sample_get_buffer(sample);
  GstMapInfo map;
  if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    // Get the image info
    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *caps_struct = gst_caps_get_structure(caps, 0);
    int width, height;
    gst_structure_get_int(caps_struct, "width", &width);
    gst_structure_get_int(caps_struct, "height", &height);

    // Create an OpenCV image from the buffer
    cv::Mat image(height, width, CV_8UC3, (char *)map.data, map.size);

    // Create image message
    sensor_msgs::msg::Image msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = camera_frame_id_;
    cv_bridge::CvImage cv_image{msg.header, "bgr8",
                                image.clone()}; // Check the lifetime of image
    cv_image.toImageMsg(msg);

    // Publish the image
    camera_publisher_->publish(msg, camera_info_manager_->getCameraInfo());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to map buffer");
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    return GST_FLOW_ERROR;
  }

  // Free the sample
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}
} // namespace unitree_camera_ros2

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(unitree_camera_ros2::UnitreeCameraNode)
