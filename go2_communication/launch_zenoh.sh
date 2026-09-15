docker run --rm --init --net host \
  --name zenoh-bridge-pc \
  -e RUST_LOG=info \
  -e ROS_DISTRO=humble \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///tmp/cyclonedds.xml \
  -v $(pwd)/cyclonedds.xml:/tmp/cyclonedds.xml \
  -v $(pwd)/zenoh_pc_config.json5:/tmp/zenoh_pc_config.json5 \
  eclipse/zenoh-bridge-ros2dds:nightly \
  -c /tmp/zenoh_pc_config.json5
