#pragma once

#include <string>
#include <ros/node_handle.h>

struct FrameID {
  std::string base_frame;
  std::string map_frame;
  std::string sensor_frame;

  static FrameID loadConfig(const ros::NodeHandle &nh) {
    FrameID frame;
    frame.base_frame = nh.param<std::string>("base_frame", "base_link");
    frame.map_frame = nh.param<std::string>("map_frame", "map");
    frame.sensor_frame = nh.param<std::string>("sensor_frame", "");
    return frame;
  }
};
