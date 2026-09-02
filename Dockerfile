FROM ros:humble-ros-base-jammy

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rviz2 \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-nav2-rviz-plugins \
    ros-humble-nav2-bringup \
    ros-humble-ros2-control \
    ros-humble-navigation2 \
    ros-humble-ros2-controllers \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-teleop-twist-keyboard \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-image-geometry \
    ros-humble-rqt-image-view \
    nlohmann-json3-dev \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    ultralytics \
    opencv-contrib-python \
    transforms3d \
    scipy

# Fixes: TypeError: canonicalize_version() got an unexpected keyword argument
# 'strip_trailing_zero' -- setuptools>=71.0.0 calls a packaging API argument
# that only exists in packaging>=22.0. Without this, colcon build fails on
# any package with a Python setup.py (ament_cmake_python), e.g. unitree_api,
# go2_interfaces, multirobot_map_merge, explore_lite_msgs.
RUN pip3 install --no-cache-dir --upgrade 'packaging>=22.0'

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /ros2_ws

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc

CMD ["/bin/bash"]
