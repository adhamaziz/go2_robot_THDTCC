# Go2 Setup & Operations Guide

Personal reference for connecting to, launching, and running perception/navigation
on the Go2 EDU + Jetson Orin Nano setup.

---

## 1. Network Access

**WiFi**
- SSID: `go2 & go2_5g`
- Password: `12345678`

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
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -o wlxcc641aee9de0 -j MASQUERADE
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

✅ **RESOLVED -- the ~2 hour TF timestamp gap.** Turned out to be two separate
things, both fixed:
1. A genuine leftover `use_sim_time: true` in the ArUco launch config
   (`~/go2_aruco_docker/`), copied from a sim setup -- flipped to `false`.
   Re-check periodically with:
   ```bash
   grep -rn "use_sim_time" ~/go2_ws/scripts/ ~/go2_ws/src/go2_nav/ ~/go2_aruco_docker/ ~/go2_yolo_docker/ 2>/dev/null
   ```
   Every occurrence on the real robot should read `false`.
2. **The actual root cause of the 2-hour gap**: the laptop's own system clock
   had drifted ~2 hours fast. `ntpdate -u` (above) syncs the *Go2* from the
   *laptop*, so a wrong laptop clock pushes the Go2 onto the same wrong time.
   Fix the laptop's clock first (`timedatectl set-ntp true`, or
   `timedatectl set-time` manually if offline), *then* re-run `ntpdate` on
   the Go2.

⚠️ **A different, much smaller TF timing issue remains, and is expected --
not a bug**: occasional `Lookup would require extrapolation into the
future`/`into the past` warnings on `map`-frame lookups, with gaps typically
under 1 second. This happens because **AMCL only republishes `map->odom`
periodically** (a few Hz, tied to scan-matching), not continuously -- so a
perception node running faster than AMCL's update rate will regularly ask
for a `map` transform slightly newer than AMCL has provided yet. This is
normal AMCL behavior, not a network/clock/allowlist problem. Nodes that need
a `map`-frame pose from a fast sensor loop should fall back to the *latest
available* transform (`rclpy.time.Time()`) when the exact-timestamp lookup
fails, rather than treating it as a hard error -- see `yolo_subscriber_node.py`
(section 5.1) for the reference implementation of this pattern.

---

## 2. ROS_DOMAIN_ID

✅ **RESOLVED -- this is fine as-is, not a bug.** The Go2's zenoh bridge runs
on ROS domain `0` (YOLO/ArUco containers, bringup); the laptop's zenoh
bridge runs on domain `1` (SLAM/Nav2/FSM). **These do not need to match.**
Each `zenoh-bridge-ros2dds` instance only needs to know its own local ROS
domain -- zenoh itself is the transport connecting the two domains together,
regardless of their numbers. Confirmed working in this configuration
end-to-end (YOLO depth back-projection, ArUco poses, and Nav2 goals all
cross the bridge correctly with domains 0 and 1 on either side).
```bash
echo $ROS_DOMAIN_ID   # just to see what's set in a given shell -- doesn't need to be consistent across machines
```

---

## 3. TF Tree -- confirmed real frame chain

✅ Confirmed via `tf2_ros tf2_echo` and `tf2_tools view_frames` on real
hardware this session:
```
map -> odom -> base_link -> camera_link -> camera_color_frame -> camera_color_optical_frame
```
- `target_frame = map`
- `robot_base_frame = base_link`
- `camera_frame = camera_color_optical_frame`

These are the confirmed, working defaults in `swap_state_machine_real.py`
and `yolo_subscriber_node.py`. Re-verify only if the URDF or the camera's
physical mount changes.

**Why this needed a fix**: the RealSense driver publishes its own internal
chain (`camera_link -> camera_color_frame -> camera_color_optical_frame`)
via `/tf_static` regardless of the rest of the robot -- but nothing
originally connected `camera_link` to `base_link`, because the D435i was
physically bolted onto the Go2's head *after* `go2_description`'s URDF was
written, and never added to it. Two disconnected TF trees, `tf2_echo`
reporting "two or more unconnected trees."

**Fix applied**: `go2_description/urdf/realsense_d435i.urdf.xacro` -- a
static joint from `base_link` to a bare `camera_link` stub (no
visual/collision; the RealSense driver already publishes everything
downstream of `camera_link` itself). Included in the actual live top-level
file, `go2_description/urdf/go2_description.urdf` (**not**
`go2_description_urdf.xacro` -- that one is a different, unused
sim/alternate file that happened to look similar), right next to the
existing Hesai lidar include:
```xml
<xacro:include filename="$(find go2_description)/urdf/hesai_xt32.urdf.xacro"/>
<xacro:include filename="$(find go2_description)/urdf/realsense_d435i.urdf.xacro"/>
```
⚠️ **The mount offset in that xacro is a placeholder** (`xyz="0.285 0.0
0.08"`, borrowed from the Head_upper joint's x and the Hesai's z, "same
level as the lidar, mounted on the head"), not a tape-measured value.
Accurate enough to unblock TF and get a working pipeline, **not** accurate
enough to trust for fine standoff distances -- measure the real offset and
update this file when there's time.

After editing, always:
```bash
cd ~/go2_ws && colcon build --packages-select go2_description
ros2 launch go2_bringup go2.launch.py   # relaunch so robot_state_publisher picks up the change
```

---

## 4. RealSense -- depth, TF publishing, and a real USB bandwidth ceiling

### 4.1 Depth is ON (was previously OFF)

Originally `enable_depth:=false, align_depth.enable:=false` in
`launch_realsense.sh` -- deliberately disabled at some point, probably to
save bandwidth. **Now required**, since the real YOLO 3D trigger (section
5.1) needs it. Confirmed working topics once enabled:
```
/camera/aligned_depth_to_color/image_raw   -- pixel-aligned to color, use color intrinsics directly
/camera/depth/image_rect_raw               -- unaligned
/camera/aligned_depth_to_color/camera_info
```
⚠️ Depth encoding is **16UC1 (integer millimetres)**, not float metres --
confirmed on this hardware. Getting this unit conversion wrong silently
produces poses ~1000x too far away with no error thrown. Handled in
`yolo_subscriber_node.py`, but worth remembering if writing any other
consumer of these topics directly.

### 4.2 `publish_tf` must be explicit

The RealSense driver needs `publish_tf:=true` passed explicitly -- without
it, none of `camera_link -> camera_color_frame -> camera_color_optical_frame`
gets published at all (confirmed via `ros2 topic echo /tf_static --once`
returning nothing). Current known-good launch:
```bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.profile:=424x240x6 \
  depth_module.profile:=424x240x15 \
  enable_depth:=true \
  align_depth.enable:=true \
  publish_tf:=true
```

### 4.3 ⚠️ Hardware ceiling: RealSense is on a USB2 port, not USB3

Confirmed via `lsusb -t`:
```
Bus 02.Port 1: Dev 1, Class=root_hub, ..., 10000M   <- real USB3 controller exists on the SoC
Bus 01.Port 1: Dev 1, Class=root_hub, ..., 480M     <- RealSense is here, on every externally accessible port tried
    |__ Port 2/3: Dev 2, Class=Video, Driver=uvcvideo, 480M
```
Tried two different physical ports; both landed on the same USB2-only Bus
01. **The expansion/dock module's externally exposed ports appear to only
expose USB2** -- Bus 02 (the real USB3 controller) has no accessible port on
this hardware. This is very likely a hardware limitation of this specific
expansion module, not a cable/port-selection problem -- don't burn more time
hunting for a USB3 port here unless a genuinely different physical
connector (e.g. a USB-C port) turns up that hasn't been tried.

**Symptom this causes**: `Non-sequential Video and Metadata buffers` /
`Video frame dropped, video and metadata buffers inconsistency` errors,
appearing intermittently and sometimes clearing up on their own -- classic
USB2 bandwidth saturation when streaming color + depth simultaneously
(roughly double the bandwidth demand vs. color alone). When it happens,
YOLO's `image_callback` starves for clean frames, so `/go2/yolo/*` topics
go quiet even though the node is alive and logs started up normally.

**Mitigation**: lower resolution/FPS to fit inside USB2's real ceiling (see
4.2's launch command above -- `424x240x6` color / `424x240x15` depth).
⚠️ Not every profile combination is valid -- `424x240x6` for **depth**
specifically was silently rejected and fell back to `640x480x15` (check the
launch log for `Given value, ... is invalid` after any profile change).
Depth currently stays at `424x240x15`; only color's FPS was reduced.

This doesn't eliminate the bandwidth ceiling, just reduces how often it's
hit. If frame drops recur, check `ros2 topic hz` on both color and depth
topics before assuming something else broke.

---

## 5. Perception Pipelines (Docker, on the Go2's Jetson)

Both containers live on the Go2 and share the same base image
(`go2-ros-base:latest`, built once from `Dockerfile.base`). The laptop only
*views*/*consumes* their output over the zenoh bridge -- no perception code
runs on the laptop itself.

### 5.1 YOLO Detection -- 3D back-projection DONE (previously 2D-only)

Lives in `~/go2_yolo_docker/`. **Files:** `Dockerfile.subscriber`,
`entrypoint.sh`, `yolo_subscriber_node.py`, `run_yolo.sh`.

✅ **Ported from the sim's `yolo_detector.py`** into the already-tested
`yolo_subscriber_node.py` (rather than writing a third parallel node) --
depth back-projection, TF transform to `map`, publishing
`/yolo/target_pose` (`geometry_msgs/PoseStamped`) for the SWAP FSM. Real
robustness details baked in that the sim version didn't need:
- **16UC1 mm -> m conversion**, explicit encoding check (section 4.1).
- **Min/max depth sanity bounds** (`0.15m`-`8.0m` by default) -- rejects the
  zero/garbage readings real depth sensors regularly produce at edges and
  reflective surfaces.
- **Exact-stamp TF lookup with a "latest available" fallback** (section 1) --
  handles AMCL's slower update rate gracefully instead of failing most frames.

**One-time build:**
```bash
cd ~/go2_yolo_docker
docker build -f Dockerfile.subscriber -t go2-yolo:latest .
```
Slow (~30-45 min) -- compiles PyTorch's CUDA wheel and torchvision from
source (no prebuilt wheels for this JetPack version), plus `rmw_cyclonedds_cpp`.

**Every session, after RealSense is running (with depth on, section 4):**
```bash
./run_yolo.sh
```
New params (all also settable via env var, see the script):
```
DEPTH_TOPIC=/camera/aligned_depth_to_color/image_raw
CAMERA_INFO_TOPIC=/camera/color/camera_info
TARGET_POSE_TOPIC=/yolo/target_pose
TARGET_FRAME=map
CAMERA_FRAME=camera_color_optical_frame
```
Publishes (existing, unchanged):
- `/go2/yolo/image_annotated/compressed` (`sensor_msgs/CompressedImage`)
- `/go2/yolo/detections` (`vision_msgs/Detection2DArray`)

Plus (new):
- `/yolo/target_pose` (`geometry_msgs/PoseStamped`, frame_id `map`)

⚠️ **Must be in the zenoh allowlist** on the Go2 side (publishers) to reach
the laptop -- confirmed added, see section 6.

COCO class IDs for other filters: `0`=person, `1`=bicycle, `2`=car, `15`=cat,
`16`=dog, `56`=chair, `67`=cell phone. Full list:
https://docs.ultralytics.com/datasets/detect/coco/

### 5.2 ArUco Detection

Lives in `~/go2_aruco_docker/`. **Files:** `Dockerfile.aruco`,
`entrypoint.sh`, `aruco_detector_node.py`, `launch.sh`.

✅ Confirmed working end-to-end this session: marker ID detected
consistently, pose values plausible against real distance, and critically
-- **`frame_id: camera_color_optical_frame`** on the published poses, which
matches exactly what `swap_state_machine_real.py`'s `start_precise_inspect()`
expects (it does its own `lookup_transform` + `do_transform_pose` on the
assumption the incoming pose is camera-relative, not already world-frame).

Publishes:
- `/go2/aruco/poses` (`geometry_msgs/PoseArray`) -- poses only, camera frame
- `/go2/aruco/marker_ids` (`std_msgs/Int32MultiArray`) -- IDs only, same order
- `/go2/aruco/image_annotated` (`sensor_msgs/Image`)

**Config, check against your actual printed markers before trusting IDs:**
- Dictionary: `DICT_4X4_50` (default) -- simulation uses `DICT_6X6_250`, a
  different family entirely. These are not interchangeable.
- Marker size: `0.05` m / 5cm (default) -- simulation uses 0.15m.

**One-time build:**
```bash
cd ~/go2_aruco_docker
docker build -f Dockerfile.aruco -t go2-aruco:latest .
```

**Every session, after RealSense is running:**
```bash
./launch.sh --dict DICT_4X4_50 --marker-size 0.05
```

`inspection_logger.py`/`gauge_reader.py` still need adapting to the
poses+ids pair format (open task, unchanged from before).

---

## 6. Zenoh Bridge -- allowlists, both directions now

⚠️ **The bridge only relays topics explicitly listed in its allowlist
config.** A topic can be publishing perfectly correctly on one side and
still be completely invisible on the other if it's not listed. This bit us
twice this session (ArUco topics, then `/yolo/target_pose` + `/tf`).

**Go2 side config** (`zenoh_bridge_go2`) -- confirmed working, both directions:
```json5
allow: {
  publishers: [
    "/scan", "/tf", "/tf_static", "/odom", "/map", "/map_metadata",
    "/map_updates", "/robot_description", "/joint_states", "/amcl_pose",
    "/particlecloud", "/particle_cloud", "/plan", "/local_plan",
    "/transformed_global_plan", "/received_global_plan",
    "/global_costmap/**", "/local_costmap/**", "/cost_cloud", "/marker",
    "/evaluation", "/**/transition_event", "/behavior_tree_log",
    "/navigate_to_pose/_action/**", "/follow_waypoints/_action/**",
    "/dock_robot/_action/**",
    "/go2/yolo/image_annotated/compressed",
    "/go2/aruco/image_annotated", "/go2/aruco/marker_ids", "/go2/aruco/poses",
    "/yolo/target_pose"
  ],
  subscribers: [
    "/cmd_vel", "/initialpose", "/goal_pose", "/clicked_point",
    "/navigate_to_pose/_action/**", "/follow_waypoints/_action/**",
    "/dock_robot/_action/**",
    "/tf", "/tf_static"
  ]
}
```
⚠️ **Why `/tf`/`/tf_static` are needed under BOTH publishers AND
subscribers**: the Go2's own body TF (legs, camera, lidar) needs to go
*out* to the laptop (publishers) -- but the laptop-computed `map->odom`
(from AMCL, which only runs on the laptop per the machine split in section
7) needs to come back *in* to the Go2 (subscribers), or perception nodes
running on the Go2 (YOLO, ArUco) will never see `map` exist at all, even
though it's real one hop away. This is safe -- `/tf` is designed as a merged
topic with multiple independent broadcasters, same as how body TF and
map/odom TF already coexist locally.

**Laptop side config** (`zenoh_bridge_pc`) -- confirmed fine as-is, **no
changes needed**: it has no `allow` block at all, which means unrestricted
(everything relayed both directions by default) -- it was never the
bottleneck. Domain `1` here vs. `0` on the Go2 is also fine (section 2).

**Every session:**
```bash
# Go2:
cd ~/zenoh_bridge && source launch.sh
# laptop:
cd go2_communication && sh launch_zenoh.sh
```

---

## 7. Machine Split -- what runs where

Standing decision for this project:

| Go2 (Jetson) | Laptop |
|---|---|
| Zenoh bridge | Zenoh bridge |
| `go2_bringup` | SLAM (mapping phase) / AMCL localization (task phase) |
| RealSense | Nav2 (planner, controller, BT navigator) |
| YOLO container | Checkpoint patrol / frontier exploration |
| ArUco container | SWAP FSM (`swap_state_machine_real.py`) |
| | rviz2 |

`swap_state_machine_real.py` lives in `go2_nav/launch/` on the laptop (not
in either Jetson docker) -- it only talks to the Go2 over topics relayed
through zenoh, no reason to run it on the Jetson itself.

---

## 8. Two-Phase Workflow: Map once, then task repeatedly

⚠️ **Important lesson from this session**: running the SWAP FSM against
**live/online SLAM** (`slam_toolbox`) causes the orbit target to visibly
drift further and further from the real object over a single cycle. Online
SLAM continuously *retroactively corrects* the `map` frame's origin as it
gets new information (loop closures, graph optimization) -- normal and
desirable while actively mapping, but the FSM stores a target `(x,y)` in
`map` frame once and computes every subsequent waypoint relative to that
frozen number. If `map` keeps moving underneath it while the cycle is still
running, the stored target silently drifts away from the real object with
every SLAM correction. `Localization: inactive` in rviz during this failure
is a red herring, not the bug -- that field only ever activates for an
AMCL-based stack, so it's expected to read inactive while running
`slam_toolbox` online.

**Fix: split into two distinct phases. Map once, save it, then always
localize + task against the frozen map.**

### Phase 1 -- Mapping (run once per environment, or whenever it changes)
```bash
# Go2: bringup already running
# Laptop:
cd scripts && sh launch_slam_online.sh
ros2 launch go2_nav nav_slam_launch.py params_file:=/ros2_ws/src/go2_nav/params/nav2_explore_params.yaml
ros2 launch go2_nav explore_launch.py

# once coverage looks complete in rviz:
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/m7_test
```
Then **Ctrl+C everything** on the laptop (SLAM, nav_slam_launch, explore) --
none of it runs again in Phase 2.

**Getting checkpoint coordinates for Phase 2** while you have the map open:
use rviz's **"Publish Point"** tool, click each spot, read off:
```bash
ros2 topic echo /clicked_point
```

### Phase 2 -- Localize + Task (run repeatedly, this is the real test)
```bash
# laptop -- localization against the SAVED map, not live SLAM
cd scripts && source launch_navigation_ib.sh
```
Give a fresh **2D Pose Estimate** in rviz, confirm `Localization: active`
and holding steady (not drifting) before continuing.

```bash
# laptop -- checkpoint patrol instead of frontier explore (see section 9)
cd ~/ros2_ws/src/go2_nav/launch
python3 checkpoint_patrol_node.py --ros-args \
  -p "checkpoints:=2.267,1.045,0.0, 0.154,0.663,0.0, 3.320,-1.878,0.0, 3.492,-5.986,0.0" \
  -p random_order:=true

# Go2: realsense, YOLO, ArUco (sections 4, 5)

# laptop: the FSM
python3 swap_state_machine_real.py --ros-args \
  -p target_frame:=map -p robot_base_frame:=base_link \
  -p camera_frame:=camera_color_optical_frame \
  -p nbv_orbit_radius_m:=1.0 -p nbv_orbit_num_waypoints:=6
```

**Trade-off worth remembering**: a frozen map means Nav2 can't discover or
adapt to anything that's changed in the environment since it was made --
fine for short supervised tests, not for indefinite autonomous operation
without periodically re-mapping.

---

## 9. Checkpoint Patrol (Phase 2 replacement for frontier exploration)

`go2_nav/launch/checkpoint_patrol_node.py`. Frontier exploration
(`explore_launch.py`) doesn't work once you have a complete static map --
it drives toward *unknown* space, and there isn't any left, so it just sits
idle. This node instead cycles through a fixed list of checkpoints via
`NavigateToPose`, giving YOLO/ArUco something to scan while driving.

- **Honors the same `/explore/resume` `Bool` convention** the FSM already
  publishes (`False`=pause, `True`=resume) -- when the FSM finds a target
  and pauses exploration, patrol actually cancels its in-flight goal and
  stops, instead of fighting the FSM's own approach goal for control.
- `checkpoints` param: flattened `x,y,yaw_deg` triples, comma-separated.
- `random_order` (default `true`) or sequential cycling.
- Imports `yaw_to_quaternion` directly from `swap_state_machine_real.py` --
  **must live in the same directory** (`go2_nav/launch/`) to resolve that
  import; Python auto-adds the script's own folder to the path when run
  directly, no packaging changes needed.

```bash
python3 checkpoint_patrol_node.py --ros-args \
  -p "checkpoints:=<x1>,<y1>,<yaw1>, <x2>,<y2>,<yaw2>, ..." \
  -p random_order:=true
```

---

## 10. SWAP FSM Robustness -- consecutive-detection confirmation

`swap_state_machine_real.py`'s `yolo_cb()` no longer commits to a real
Nav2 approach goal on the very first `/yolo/target_pose` message received.
**Why**: a single noisy depth pixel (reflection, object edge, or a
stale/extrapolated TF sample) can produce one wildly-wrong pose -- observed
directly during testing, one frame jumped ~2m from every neighbouring
frame before immediately snapping back.

**New behavior**: requires `yolo_confirm_count` (default `3`) consecutive
detections, each within `yolo_confirm_radius_m` (default `0.3`) of the
previous one, before averaging them and calling `start_approach()`. Any
detection further than the radius resets the buffer rather than being
averaged in as if it were valid.
```bash
-p yolo_confirm_count:=3 -p yolo_confirm_radius_m:=0.3   # tunable at launch
```
⚠️ Adds roughly half a second of latency at typical camera frame rates
before the approach triggers -- fine for a scanning/stationary target, but
worth dropping to `2` if it feels sluggish with the robot moving.

---

## 11. Isolated Testing Protocol -- run these one at a time, in order

Don't run the full pipeline together until each piece is confirmed
individually -- otherwise a failure could be coming from any of three
places and there's no way to tell which. **Keep Phase 2 localization active
(section 8) for every stage below.**

### Stage A -- YOLO + depth estimation only
Launch: bringup, RealSense, YOLO. **Not running**: ArUco, patrol, FSM.
```bash
ros2 topic hz /camera/color/image_raw          # Go2: camera healthy?
ros2 topic hz /go2/yolo/image_annotated/compressed   # Go2: YOLO processing frames?
ros2 topic hz /go2/yolo/image_annotated/compressed   # laptop: survives the bridge?
ros2 topic echo /yolo/target_pose              # laptop: the actual output
```
**Pass**: clean bounding box on the target; pose values stable when the
camera is held still (confirmed: sub-cm jitter over several seconds of
holding steady), roughly matching a tape-measure distance.

### Stage B -- ArUco detection only
Launch: bringup, RealSense, ArUco. **Not running**: YOLO, patrol, FSM.
```bash
ros2 topic echo /go2/aruco/marker_ids
ros2 topic echo /go2/aruco/poses
```
**Pass**: correct marker ID, plausible position, `frame_id:
camera_color_optical_frame` (confirmed -- matches what the FSM expects).

### Stage C -- NBV orbit movement only (fake trigger, real navigation)
Launch: bringup, localization active, Nav2, the FSM. **Not running**: YOLO,
ArUco, patrol.
```bash
python3 swap_state_machine_real.py --ros-args \
  -p target_frame:=map -p robot_base_frame:=base_link \
  -p camera_frame:=camera_color_optical_frame \
  -p nbv_orbit_radius_m:=1.0 -p nbv_orbit_num_waypoints:=6

ros2 topic echo /current_behaviour
ros2 topic pub --once /yolo/target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: <real_x>, y: <real_y>, z: 0.0}}}"
```
**Pass**: `B1_EXPLORE -> B2_APPROACH`, smooth drive to standoff, all N orbit
waypoints executed cleanly, ends in "orbit exhausted, no ArUco tag found"
(expected -- ArUco isn't running in this stage).

### Stage D -- Full integration, still no patrol
Launch: everything from A+B+C together. **Still not running**: patrol --
walk the robot near the target yourself. Tests the real handoff: YOLO
trigger -> orbit -> ArUco -> standoff, all live.

### Stage E -- Add patrol back in
Only once D passes cleanly. Full autonomous version, section 8's Phase 2
launch as written.

---

## 12. Misc

**Check Go2 system load**
```bash
htop
```
Worth checking if you see `Behavior Tree tick rate 100.00 was exceeded`
warnings from Nav2 -- that's a CPU/timing complaint (the BT executor
couldn't complete a tick within its 10ms/100Hz budget), unrelated to
costmap/goal-tolerance params. Usually survivable on its own; only chase it
if paired with actually-late robot behavior (sluggish reactions, delayed
recoveries). If it needs fixing, lower `bt_loop_duration` in the nav params
(e.g. `20` for 50Hz) rather than trying to force 100Hz on hardware that
can't sustain it.

**Hesai lidar web config** -- with Go2 connected to internet, browse to:
```
192.168.123.20
```

**Docker on the laptop side (go2_humble container)**
```bash
xhost +local:root
docker compose run --rm --remove-orphans go2_humble bash
```

**Viewing compressed image topics** -- subscribe to the *base* topic name
(e.g. `/go2/yolo/image_annotated`, not `.../compressed`) and pick Transport
Hint: compressed from the dropdown in rviz/rqt_image_view -- typing the
`/compressed` suffix directly sometimes fails to resolve even when the
topic is healthy.

---

## 13. Troubleshooting cheat sheet (things that broke once, in the order encountered)

| Symptom | Cause | Fix |
|---|---|---|
| `apt-get update` fails, `EXPKEYSIG` | ROS2 apt signing key expired | Dockerfile refreshes the key from rosdistro before installing anything |
| `Unable to locate package ros-humble-*` | Humble has no official apt binaries for Ubuntu 20.04/Focal | Use the `humble-desktop` base image (built from source), not `humble-ros-base` |
| `pip install numpy==1.26.1` fails | Board runs Python 3.8, that numpy needs 3.9+ | Pinned to `numpy==1.24.4` |
| `ImportError: TypeAlias` during torchvision build | Old system `typing_extensions` | `pip3 install 'typing_extensions>=4.5.0'` before building torch/torchvision |
| `ModuleNotFoundError: torchvision` after a "successful" build | `setup.py install` silently didn't register the package | Use `pip3 install .` instead, with a self-check at the end of that build step |
| `NotImplementedError: torchvision::nms ... CUDA backend` | torchvision built CPU-only ops | `FORCE_CUDA=1` + `TORCH_CUDA_ARCH_LIST="8.7"` (Orin's compute capability) during the build |
| `exec: --: invalid option` on container start | Args passed after image name replace `CMD` instead of appending | Use `ENTRYPOINT` (`entrypoint.sh`) instead of `CMD`, so trailing args append |
| `InvalidParameterTypeException: classes_filter` | ROS2 YAML-parses `-p key:=0` as an int, param is declared as string | `launch.sh`/`run_yolo.sh` wraps the value in extra single quotes to force string parsing |
| Topics never show up on the host at all | DDS **vendor** mismatch: Go2 uses Cyclone DDS, container defaulted to Fast-DDS | Built `rmw_cyclonedds_cpp` from source in the image, set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| Topics visible everywhere, but `rqt_image_view` shows a blank gray gradient | QoS reliability mismatch -- publisher was `BEST_EFFORT`, viewer defaults to `RELIABLE` (incompatible, fails silently) | Publish with `RELIABLE` QoS; keep the *subscription* to the camera topic `BEST_EFFORT` to match the camera driver's convention |
| pip install times out mid-download | Flaky Jetson network connection | `--default-timeout=180 --retries 10` on pip installs |
| A topic exists on the Jetson but never appears in `ros2 topic list` on the laptop | Zenoh bridge allowlist doesn't include it | Add the topic to `plugins.ros2dds.allow.publishers`, restart the bridge (section 6) |
| `cv2` import fails, "partially initialized module", or `circular import` | Mixed install of base image's Tegra-optimized OpenCV + a pip-installed one on top | Fully uninstall + `rm -rf` every existing `cv2*` before a clean `pip install opencv-contrib-python` |
| `computeCircumscribedCost`: inflation radius smaller than circumscribed radius, planner fails almost every goal | `inflation_radius` set below the robot's actual footprint corner-to-center distance | Set `inflation_radius` above the circumscribed radius with real margin (e.g. 0.4 for a ~0.334 circumscribed radius) -- re-check this exact error after any costmap tuning pass |
| ~2 hour TF gap between requested/latest time | Laptop system clock drifted ~2hrs fast, pushed onto the Go2 via `ntpdate` | Fix laptop clock first (`timedatectl`), then re-sync Go2 (section 1) |
| `camera_color_optical_frame` missing from `view_frames` entirely | RealSense driver's own TF wasn't being published (`publish_tf` not explicitly set) | Add `publish_tf:=true` to the RealSense launch (section 4.2) |
| `tf2_echo base_link camera_link` -- "two or more unconnected trees" | URDF never had a joint connecting the camera (bolted on after the URDF was written) | Added `realsense_d435i.urdf.xacro`, static joint from `base_link` (section 3) |
| `map` frame "does not exist" on the Go2-side perception nodes | Zenoh allowlist only had `/tf`/`/tf_static` under publishers, not subscribers -- laptop's AMCL-computed `map` never routed back to the Go2 | Add `/tf`, `/tf_static` to the **subscribers** list too (section 6) |
| Occasional "extrapolation into the future/past", gaps under ~1s | Normal: AMCL updates `map->odom` slower than a fast perception loop asks for it | Fall back to latest-available TF (`rclpy.time.Time()`) when exact-stamp lookup fails (section 1, section 5.1) |
| `Non-sequential Video and Metadata buffers` / dropped frames | RealSense on a USB2-only port -- confirmed hardware ceiling on this expansion module | Lower resolution/FPS to fit (section 4.3); stop hunting for a USB3 port that likely doesn't exist externally on this hardware |
| YOLO target orbit drifts further and further from the real object each cycle | Running the FSM against **live/online SLAM** -- `map` frame keeps moving under a stored target coordinate | Two-phase workflow: map once, save it, localize (AMCL) + task against the frozen map (section 8) |
| Frontier exploration (`explore_launch.py`) never drives anywhere in Phase 2 | No unknown space left to explore once the map is complete -- expected, not a bug | Use `checkpoint_patrol_node.py` instead in Phase 2 (section 9) |
| One `/yolo/target_pose` message received twice, identical timestamp+values | Suspected zenoh-level duplicate delivery over multiple network paths in peer mode; confirmed `Publisher count: 1` via `ros2 topic info --verbose`, so NOT a duplicate node/bridge | Harmless for the FSM's confirmation logic (an exact duplicate trivially passes the "within radius" check) -- deprioritized, not worth chasing further under time pressure |