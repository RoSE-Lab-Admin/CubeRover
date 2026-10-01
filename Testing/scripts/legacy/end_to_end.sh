#!/bin/bash

LINEAR_VEL_X=2
PUB_PER_SEC=10
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link'}, twist: {linear: {x: $LINEAR_VEL_X}}}" --rate $PUB_PER_SEC
