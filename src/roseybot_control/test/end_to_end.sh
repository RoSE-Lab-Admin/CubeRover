#!/bin/bash


timeout 30 ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
"{header: {frame_id: base_link}, twist: {linear: {x: 0.10}}}"
    