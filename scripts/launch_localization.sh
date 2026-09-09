ros2 launch go2_nav navigation.launch.py \
    map:=/ros2_ws/maps/benchmark_sep8.yaml \
    params_file:=/ros2_ws/src/go2_nav/params/nav2_params.yaml \
    rviz:=True \
    use_sim_time:=False