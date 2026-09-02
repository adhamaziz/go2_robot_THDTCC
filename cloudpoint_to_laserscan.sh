ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args \
  -r cloud_in:=/utlidar/cloud_base \
  -r scan:=/scan \
  -p target_frame:=base_link \
  -p transform_tolerance:=0.02 \
  -p min_height:=-0.05 \
  -p max_height:=1.5 \
  -p angle_min:=-3.14159 \
  -p angle_max:=3.14159 \
  -p angle_increment:=0.0087 \
  -p scan_time:=0.1 \
  -p range_min:=0.2 \
  -p range_max:=30.0 \
  -p use_inf:=true \
  -p use_sim_time:=false
  
  # -p target_frame:=base_link \
  # -p transform_tolerance:=0.1 \
  # -p min_height:=0.0 \
  # -p max_height:=0.5 \
  # -p angle_min:=-3.14159 \
  # -p angle_max:=3.14159 \
  # -p angle_increment:=0.0087 \
  # -p scan_time:=0.1 \
  # -p range_min:=0.1 \
  # -p range_max:=30.0 \
  # -p use_inf:=true

# -p min_height:=-0.2 \
# -p transform_tolerance:=0.01 \