# Go2 Setup & Operations Guide

Personal reference for connecting to, launching, and running perception/navigation
on the Go2 EDU + Jetson Orin Nano setup.

---

## 1. Network Access

**WiFi**
- SSID: `DroneBlocks-Go2-001`
- Password: `00000000`

**SSH into the Go2**
```bash
ssh unitree@192.168.123.18
# password: 123
```
(Alternate/legacy: `ssh unitree@10.42.0.1`, password `123` -- not currently used.)

**Laptop static IP (for the Go2's network interface)**
- Address: `192.168.123.222`
- Netmask: `255.255.255.0`
- Routes:
  - `192.168.123.18 / 255.255.255.255`
  - `192.168.123.161 / 255.255.255.255`
- "Use this connection only for resources on its network": **Yes**

**IP forwarding + NAT (laptop) -- must re-run after every laptop reboot**
```bash
# 1. Enable IPv4 forwarding
sudo sysctl -w net.ipv4.ip_forward=1

# 2. Masquerade outbound traffic through your internet dongle
sudo iptables -t nat -A POSTROUTING -o wlxcc641aee9de0 -j MASQUERADE

# 3. Allow forwarding between interfaces
sudo iptables -A FORWARD -i wlo1 -o wlxcc641aee9de0 -j ACCEPT
sudo iptables -A FORWARD -i wlxcc641aee9de0 -o wlo1 -m state --state RELATED,ESTABLISHED -j ACCEPT
```
On the Go2 itself (route back to laptop, adjust IP to your laptop's static IP):
```bash
sudo ip route add default via 192.168.123.xxx
```

**Time sync** (if things seem off, e.g. TF/timestamps):
```bash
sudo ntpdate -u 192.168.123.222
timedatectl
```

⚠️ **Open investigation (unresolved):** seen a ~2 hour TF timestamp gap between
`controller_server`/`explore_node`'s "requested time" and the "latest data"
time, on the real robot. NTP sync above is one candidate fix, but the leading
suspect is actually `use_sim_time: true` left over in a real-robot launch
config that was copied/adapted from the simulation setup -- any node with
that flag waits on a `/clock` topic that nothing on real hardware publishes.
Check before trusting any real Nav2 goal:
```bash
grep -rn "use_sim_time" ~/go2_ws/scripts/ ~/go2_ws/src/go2_nav/ 2>/dev/null
```
Every occurrence on the real robot should read `false`.

---

## 2. ROS_DOMAIN_ID -- check before combining workflows

⚠️ **Unresolved discrepancy**: your navigation notes set `export ROS_DOMAIN_ID=1`,
but the YOLO and ArUco containers (below) were built, tested, and confirmed
working end-to-end using the **default `ROS_DOMAIN_ID=0`** (unset). If you run
navigation and perception at the same time, make sure both sides agree on one
domain ID, or they won't discover each other's topics -- same failure mode as
the Cyclone DDS mismatch we debugged. Check with:
```bash
echo $ROS_DOMAIN_ID
```

---

## 3. Every-Session Launch Order

### 3.1 Connect
1. Connect laptop to `DroneBlocks-Go2-001` WiFi.
2. Re-apply IP forwarding + NAT rules on the laptop (section 1), if rebooted.
3. `ssh unitree@192.168.123.18` (pw `123`).

### 3.2 Zenoh bridge (both sides, every session)
On the Go2:
```bash
cd ~/zenoh_bridge
source launch.sh
```
On the laptop:
```bash
cd go2_communication
sh launch_zenoh.sh
# or: sh launch_zenoh_pc.sh   (direct)
# use launch_zenoh_jazzy.sh if using the brostrend WiFi adapter
```

⚠️ **The bridge only relays topics explicitly listed in its allowlist config**
(`zenoh_bridge_go2` config, `plugins.ros2dds.allow.publishers`). A topic can
be publishing perfectly correctly on the Go2 and still be completely invisible
from the laptop if it's not in this list -- this happened with the ArUco
topics (fixed, see section 4.2) and will happen again for any new topic you
add (e.g. the eventual `/yolo/target_pose` 3D trigger). If a topic you expect
to see from the laptop doesn't show up in `ros2 topic list`, **check the
allowlist before assuming the node itself is broken.**

### 3.3 Bringup (Go2)
```bash
ros2 launch go2_bringup go2.launch.py
# with Hesai lidar:
ros2 launch go2_bringup go2.launch.py lidar:=True
```

### 3.4 Localization / Navigation (optional, Go2 side)
```bash
cd go2_ws/scripts
source launch_localization.sh
source launch_navigation.sh
```
Or from the laptop side:
```bash
cd scripts
source launch_navigation_ib.sh
# launches navigation, rviz2, and map after pose estimation
```

### 3.5 RealSense camera (Go2)
```bash
cd ~/go2_ws/scripts
sh launch_realsense.sh
```
Check the camera is detected first if in doubt:
```bash
rs-enumerate-devices
```
(Install reference: https://github.com/realsenseai/librealsense/blob/master/doc/installation_jetson.md)

⚠️ **Open question, not yet confirmed:** does this driver publish any
`/camera/depth/...` topic? `ros2 topic list | grep -i depth` on the Go2 to
check. This determines whether the real YOLO detector can do 3D
back-projection (like the simulation version does) or needs a different,
depth-free approach (2D bounding-box centering) to trigger the FSM.

### 3.6 YOLO detection (Go2, Docker)
```bash
cd ~/go2_yolo_docker
./launch.sh --person          # or ./launch.sh for all 80 COCO classes
./run_yolo.sh                 # for the fire extinguisher fine-tuned model
```
See section 4.1 for full detail.

### 3.7 ArUco detection (Go2, Docker)
```bash
cd ~/go2_aruco_docker
./launch.sh --dict DICT_4X4_50 --marker-size 0.05
```
See section 4.2 for full detail.

### 3.8 View results (laptop)
```bash
ros2 run rqt_image_view rqt_image_view /go2/yolo/image_annotated/compressed
ros2 run rqt_image_view rqt_image_view /go2/aruco/image_annotated
# or rviz2, Add -> By topic -> the compressed/image topic you want
```

---

## 4. Perception Pipelines (Docker, on the Go2's Jetson)

Both containers live on the Go2 and share the same base image
(`go2-ros-base:latest`, built once from `Dockerfile.base`). The laptop only
*views* their output over the zenoh bridge -- no perception code runs on the
laptop itself.

### 4.1 YOLO Detection

Lives in `~/go2_yolo_docker/`. **Files:** `Dockerfile.subscriber`,
`entrypoint.sh`, `yolo_subscriber_node.py`, `run_yolo.sh`.

**Currently 2D-only**: publishes bounding boxes and an annotated image, but
does **not** do depth back-projection to a 3D pose -- unlike the simulation's
`yolo_detector.py`, which publishes `/yolo/target_pose` for the SWAP FSM to
consume. This is the main gap blocking a fully autonomous real-robot cycle;
see the open question in 3.5 above.

**One-time build:**
```bash
cd ~/go2_yolo_docker
docker build -f Dockerfile.subscriber -t go2-yolo:latest .
```
Slow (~30-45 min) -- compiles PyTorch's CUDA wheel and torchvision from
source (no prebuilt wheels for this JetPack version), plus `rmw_cyclonedds_cpp`.

**Every session, after realsense is running:**
```bash
./launch.sh                    # all 80 COCO classes
./launch.sh --person           # class 0 (person) only
./run_yolo.sh                  # fire-extinguisher fine-tuned model (default:
                                # looks for models/fire_extinguisher_best.pt
                                # relative to the script's own location)
```

Publishes:
- `/go2/yolo/image_annotated/compressed` (`sensor_msgs/CompressedImage`)
- `/go2/yolo/detections` (`vision_msgs/Detection2DArray`)

COCO class IDs for other filters: `0`=person, `1`=bicycle, `2`=car, `15`=cat,
`16`=dog, `56`=chair, `67`=cell phone. Full list:
https://docs.ultralytics.com/datasets/detect/coco/

### 4.2 ArUco Detection

Lives in `~/go2_aruco_docker/`. **Files:** `Dockerfile.aruco`,
`entrypoint.sh`, `aruco_detector_node.py`, `launch.sh`.

**Important -- different message shape from the simulation's ArUco node.**
The simulation package (`ros_aruco_opencv`) bundles each marker's ID and pose
together in one `ArucoDetection` message. This real-robot node instead
publishes **two separate, parallel-indexed topics**:
- `/go2/aruco/poses` (`geometry_msgs/PoseArray`) -- poses only, camera frame
- `/go2/aruco/marker_ids` (`std_msgs/Int32MultiArray`) -- IDs only, same order
- `/go2/aruco/image_annotated` (`sensor_msgs/Image`)

Any consumer (FSM, logger, gauge reader) written against the simulation's
message format needs adapting to read this pair and match them by array
index, not a straight drop-in port. `swap_state_machine_real.py` already does
this; `inspection_logger.py`/`gauge_reader.py` do not yet (open task).

**Config, check against your actual printed markers before trusting IDs:**
- Dictionary: `DICT_4X4_50` (default) -- simulation uses `DICT_6X6_250`, a
  different family entirely. These are not interchangeable.
- Marker size: `0.05` m / 5cm (default) -- simulation uses 0.15m. Wrong size
  here doesn't break ID detection but will corrupt the solved 3D pose.

**One-time build:**
```bash
cd ~/go2_aruco_docker
docker build -f Dockerfile.aruco -t go2-aruco:latest .
```
Faster than YOLO's build -- no GPU/CUDA compilation needed, ArUco detection
is lightweight CPU work.

**Every session, after realsense is running:**
```bash
./launch.sh                              # DICT_4X4_50, 5cm markers, defaults
./launch.sh --marker-size 0.10           # if using 10cm markers
./launch.sh --dict DICT_6X6_250          # if markers match the sim dictionary instead
```

### Troubleshooting cheat sheet (things that broke once, in build/run order)

| Symptom | Cause | Fix |
|---|---|---|
| `apt-get update` fails, `EXPKEYSIG` | ROS2 apt signing key expired | Dockerfile refreshes the key from rosdistro before installing anything |
| `Unable to locate package ros-humble-*` | Humble has no official apt binaries for Ubuntu 20.04/Focal | Use the `humble-desktop` base image (built from source), not `humble-ros-base` |
| `pip install numpy==1.26.1` fails | Board runs Python 3.8, that numpy needs 3.9+ | Pinned to `numpy==1.24.4` |
| `ImportError: TypeAlias` during torchvision build | Old system `typing_extensions` | `pip3 install 'typing_extensions>=4.5.0'` before building torch/torchvision |
| `ModuleNotFoundError: torchvision` after a "successful" build | `setup.py install` silently didn't register the package | Use `pip3 install .` instead, with a self-check at the end of that build step |
| `NotImplementedError: torchvision::nms ... CUDA backend` | torchvision built CPU-only ops | `FORCE_CUDA=1` + `TORCH_CUDA_ARCH_LIST="8.7"` (Orin's compute capability) during the build |
| `exec: --: invalid option` on container start | Args passed after image name replace `CMD` instead of appending | Use `ENTRYPOINT` (`entrypoint.sh`) instead of `CMD`, so trailing args append |
| `InvalidParameterTypeException: classes_filter` | ROS2 YAML-parses `-p key:=0` as an int, param is declared as string | `launch.sh` wraps the value in extra single quotes to force string parsing |
| Topics never show up on the host at all | DDS **vendor** mismatch: Go2 uses Cyclone DDS, container defaulted to Fast-DDS | Built `rmw_cyclonedds_cpp` from source in the image, set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| Topics visible everywhere, but `rqt_image_view` shows a blank gray gradient | QoS reliability mismatch -- publisher was `BEST_EFFORT`, viewer defaults to `RELIABLE` (incompatible, fails silently) | Publish with `RELIABLE` QoS; keep the *subscription* to the camera topic `BEST_EFFORT` to match the camera driver's convention |
| pip install times out mid-download | Flaky Jetson network connection | `--default-timeout=180 --retries 10` on pip installs |
| A topic exists on the Jetson but never appears in `ros2 topic list` on the laptop | Zenoh bridge allowlist doesn't include it | Add the topic to `plugins.ros2dds.allow.publishers` in the bridge config, restart the bridge |
| `cv2` import fails, "partially initialized module", or `circular import` | Mixed install of base image's Tegra-optimized OpenCV + a pip-installed one on top | Fully uninstall + `rm -rf` every existing `cv2*` before a clean `pip install opencv-contrib-python` |
| `computeCircumscribedCost`: inflation radius smaller than circumscribed radius, planner fails almost every goal ("exceeded maximum iterations") | `inflation_radius` set below the robot's actual footprint corner-to-center distance | Set `inflation_radius` above the circumscribed radius with real margin (e.g. 0.4 for a ~0.334 circumscribed radius) -- easy to silently reintroduce after any costmap tuning pass, re-check the log for this exact error after changing `inflation_radius` |

---

## 5. SLAM / Mapping

```bash
# Go2: bringup already running (3.3)
# Laptop:
ros2 launch slam_toolbox online_sync_launch.py slam_params_file:=/root/ros2_ws/mapper_params_online_sync.yaml
rviz2
# set Fixed Frame to "map", teleop around to build the map, watch in rviz2

# Save the map:
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/aug31
```

---

## 6. Misc

**Check Go2 system load**
```bash
htop
```

**Hesai lidar web config** -- with Go2 connected to internet, browse to:
```
192.168.123.20
```

**Docker on the laptop side (go2_humble container)**
```bash
xhost +local:root
docker compose run --rm --remove-orphans go2_humble bash
```

---

## 7. Exploration setup

```bash
# Go2: bringup already running (3.3)
# Laptop:
cd scripts
sh launch_slam_online.sh
# Another terminal
ros2 launch go2_nav nav_slam_launch.py params_file:=/ros2_ws/src/go2_nav/params/nav2_explore_params.yaml

# visualize nav2, occupancy grid
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz

# launch main launch file for frontier exploration
ros2 launch go2_nav explore_launch.py

# Save the map:
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/1_date
```

---

## 8. Full Pipeline — every subsystem at once

This is the complete real-robot stack, all terminals, in the order they need
to come up. Some steps below are still open work (marked ⚠️) -- run what's
ready, and treat the marked steps as the current blockers to a fully
autonomous cycle.

```bash
# --- Go2 side ---
# T1: zenoh bridge
cd ~/zenoh_bridge && source launch.sh

# T2: bringup
ros2 launch go2_bringup go2.launch.py

# T3: localization + navigation
cd ~/go2_ws/scripts
source launch_localization.sh
source launch_navigation.sh

# T4: realsense
cd ~/go2_ws/scripts
sh launch_realsense.sh

# T5: YOLO (fine-tuned model)
cd ~/go2_yolo_docker
./run_yolo.sh

# T6: ArUco
cd ~/go2_aruco_docker
./launch.sh --dict DICT_4X4_50 --marker-size 0.05

# --- Laptop side ---
# T7: zenoh bridge
cd go2_communication && sh launch_zenoh.sh

# T8: exploration
cd scripts
sh launch_slam_online.sh
# (separate terminal)
ros2 launch go2_nav nav_slam_launch.py params_file:=/ros2_ws/src/go2_nav/params/nav2_explore_params.yaml
ros2 launch go2_nav explore_launch.py

# T9: view perception outputs
ros2 run rqt_image_view rqt_image_view /go2/yolo/image_annotated/compressed
ros2 run rqt_image_view rqt_image_view /go2/aruco/image_annotated

# T10: NOT YET RUN ON HARDWARE -- swap_state_machine_real.py
# Verify real TF frame names first:
ros2 run tf2_tools view_frames
ros2 action list | grep -E "navigate_to_pose|spin"
python3 swap_state_machine_real.py --ros-args \
  -p target_frame:=<confirmed> \
  -p robot_base_frame:=<confirmed> \
  -p camera_frame:=<confirmed> \
  -p nbv_orbit_radius_m:=1.0 \
  -p nbv_orbit_num_waypoints:=6

# T11: watch state transitions
ros2 topic echo /current_behaviour

# T12: manual trigger, until the real 3D YOLO trigger exists --
# safety: clear open area, E-stop ready, supervised, low speed
ros2 topic pub --once /yolo/target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: '<confirmed_map_frame>'}, pose: {position: {x: <real_x>, y: <real_y>, z: 0.0}}}"
```

**Before trusting any of T10-T12:** confirm the TF timestamp gap (section 1)
is resolved, and confirm the zenoh allowlist (section 3.2) includes
everything the FSM needs to see.
