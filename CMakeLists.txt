cmake_minimum_required(VERSION 3.20)
project(unitree_camera_ros2)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------- Find thirdparty library -------- #
# ROS packages
set(ros_dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  cv_bridge
  camera_info_manager
)

find_package(ament_cmake REQUIRED)

foreach(ros_dep ${ros_dependencies})
  find_package(${ros_dep} REQUIRED)
endforeach()

# Thirdparty packages
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)

pkg_search_module(gstreamer REQUIRED IMPORTED_TARGET gstreamer-1.0>=1.4)
pkg_search_module(gstreamer-sdp REQUIRED IMPORTED_TARGET gstreamer-sdp-1.0>=1.4)
pkg_search_module(gstreamer-app REQUIRED IMPORTED_TARGET gstreamer-app-1.0>=1.4)
pkg_search_module(gstreamer-video REQUIRED IMPORTED_TARGET gstreamer-video-1.0>=1.4)

set(other_dependencies
  PkgConfig::gstreamer
  PkgConfig::gstreamer-sdp
  PkgConfig::gstreamer-app
  PkgConfig::gstreamer-video
  ${OpenCV_LIBRARIES}
)

# ------------- Build library ------------- #
add_library(${PROJECT_NAME} SHARED
  src/unitree_camera_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  ${other_dependencies}
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "unitree_camera_ros2::UnitreeCameraNode"
  EXECUTABLE unitree_camera_node
)

ament_target_dependencies(${PROJECT_NAME} ${ros_dependencies})

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${ros_dependencies})

# ---------------- Install --------------- #
install(DIRECTORY include/
  DESTINATION include
)

# Install  directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()