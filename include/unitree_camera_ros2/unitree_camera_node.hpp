#pragma once

#include <memory>
#include <string>

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

  std::shared_ptr<image_transport::CameraPublisher> camera_publisher_;

  std::string udp_multicast_ip_;
  int udp_multicast_port_;
  std::string multicast_interface_;
  GstElement *pipeline_;
  GstElement *appsink_;
  GstAppSinkCallbacks appsink_callbacks_;
};
} // namespace unitree_camera_ros2