#!/bin/bash
ros2 launch nav2_bringup bringup_launch.py \
  map:=/ros2_ws/may4.yaml \
  params_file:=/ros2_ws/src/go2_nav/params/nav2_humble_params.yaml \
  use_sim_time:=false