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

---

## 2. ROS_DOMAIN_ID -- check before combining workflows

⚠️ **Unresolved discrepancy**: your navigation notes set `export ROS_DOMAIN_ID=1`,
but the YOLO container (below) was built, tested, and confirmed working end-to-end
using the **default `ROS_DOMAIN_ID=0`** (unset). If you run navigation and YOLO
detection at the same time, make sure both sides agree on one domain ID, or
they won't discover each other's topics -- same failure mode as the Cyclone DDS
mismatch we debugged. Check with:
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

### 3.6 YOLO detection (Go2, Docker) -- see section 4 for full detail
```bash
cd ~/go2_yolo_docker
./launch.sh --person          # or ./launch.sh for all 80 COCO classes
./run_yolo.sh # for the fire extinguisher fine tuned model
```

### 3.7 View results (laptop)
```bash
ros2 run rqt_image_view rqt_image_view /go2/yolo/image_annotated/compressed
# or rviz2, Add -> By topic -> /go2/yolo/image_annotated -> Image
```
---

## 4. YOLO Detection Pipeline (Docker, on the Go2's Jetson)

Lives entirely on the Go2 in `~/go2_yolo_docker/`. The laptop only *views* the
output over the zenoh bridge -- no YOLO code runs on the laptop.

**Files:** `Dockerfile.subscriber`, `entrypoint.sh`, `yolo_subscriber_node.py`, `launch.sh`

**One-time build** (only needed again if you edit the Dockerfile or the node):
```bash
cd ~/go2_yolo_docker
docker build -f Dockerfile.subscriber -t go2-yolo:latest .
```
This is slow (~30-45 min total) because it compiles PyTorch's CUDA wheel,
builds torchvision from source (no prebuilt wheel exists for JetPack 5), and
builds `rmw_cyclonedds_cpp` from source (to match the Go2's native DDS vendor).

**Every session, after realsense is running (3.5 above):**
```bash
./launch.sh                    # all 80 COCO classes
./launch.sh --person           # class 0 (person) only -- fastest, cleanest output
./launch.sh --classes 0,2      # person + car
./launch.sh --conf 0.6 --person
./launch.sh --build            # force a rebuild first, then launch
./run_yolo.sh # for the fire extinguisher fine tuned model
```

Publishes:
- `/go2/yolo/image_annotated/compressed` (`sensor_msgs/Image`) -- for viewing
- `/go2/yolo/detections` (`vision_msgs/Detection2DArray`) -- structured boxes/classes/scores

COCO class IDs if you want other filters: `0`=person, `1`=bicycle, `2`=car,
`15`=cat, `16`=dog, `56`=chair, `67`=cell phone. Full list:
https://docs.ultralytics.com/datasets/detect/coco/

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
