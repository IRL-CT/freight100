# Freight100 Autonomous Mobile Robot — System Overview

> A complete technical profile of this robot, captured live from the running system.
> Unit identity: **`freight1794`** (system hostname `MORPHEUS`).

This is a **Fetch Robotics / Zebra Freight100** differential-drive autonomous mobile base
(AMR) running **ROS 1 (Noetic)** on **Ubuntu 20.04**, with the stock Fetch driver stack
underneath a custom **Robust.AI "LIFE"** autonomy application on top.

---

## 1. Platform at a Glance

| Property | Value |
|---|---|
| Robot class | Freight100 base (mobile base, **no arm**) |
| Robot type param | `robot/type = freight` |
| Unit name / robot ID | `freight1794` |
| System hostname | `MORPHEUS` |
| OS | Ubuntu 20.04 LTS (Focal), kernel 5.4.0 |
| Middleware | ROS 1 **Noetic** (ROS_DISTRO=noetic, Python 3) |
| ROS master | `http://localhost:11311` (running) |
| Base drive | Differential drive, 2 powered wheels |
| Wheel track width | `0.37476 m` |
| Max linear velocity | `1.5 m/s` (base_controller `max_velocity_x`) |
| Max linear acceleration | `2.0 m/s²` |
| Fetch driver stack version | `fetch_bringup` 0.9.3 |

---

## 2. Software Architecture

Two layers run together:

### Layer A — Stock Fetch/Freight driver stack
Launched from `/opt/ros/noetic/share/freight_bringup/launch/freight.launch`.

- **`robot_driver`** (`fetch_drivers`) — talks to the base motor-control boards over the
  internal wired network (`10.42.42.1/24`). Publishes battery, IMU, odometry, joint states,
  robot state, breaker/LED/charge action interfaces. Loads firmware from
  `fetch_drivers/firmware.tar.gz`.
- **`base_controller`** — `robot_controllers/DiffDriveBaseController`, autostarts to publish
  odometry; has a `laser_safety_dist` of 1.5 m (slows before collisions).
- **`robot_state_publisher`**, **`graft`** (odometry EKF fusion), robot description/URDF
  from `fetch_description/robots/freight.urdf`.

### Layer B — Robust.AI "LIFE" autonomy stack
Runs under user `robustai` from `/home/robustai/LIFE` (launched via
`robust_launch roscore.launch`). This is the intelligent application layer:

- **Behavior-tree autonomy**: `btree`, `btree_viz`, `pybtrees_nav` — goal-driven navigation
  via behavior trees.
- **SLAM / mapping**: **RTAB-Map** (`rtabmap`) with 3 stereo syncs; produces occupancy grids,
  octomaps, point-cloud maps, global/local paths, localization pose.
- **Scene-graph planner**: `local_scene_graph` + `scene_graph/*` layers (waypoint graph,
  navigation zones, portals, route planning).
- **Motion arbitration**: `movement_mux_node` (see §4).
- **Localization**: `graft`, `localization_status`, `robot_pose_publisher`, `stamp_pose`.
- **Fleet integration**: `fetchcore/connected`, `fetchcore/error_status_requests`
  (FetchCore fleet-management hooks).
- **App directory** `/home/robustai/LIFE/apps`: `navigation`, `sound_effects`,
  `example_app`, `condition_example`, `btree_utils`.

---

## 3. Sensor Suite (live on the ROS graph)

| Sensor | Nodes / topics |
|---|---|
| **2D safety lidar** | SICK **TiM551** (`sick_tim551_2050001`) → `/base_scan`, `/base_scan_raw`; self-filtered variants |
| **Depth cameras (Intel RealSense)** | Multi-camera D435 rig via `robust_multicam_realsense` / `rs_camera_manager`: `left_cam`, `right_cam`, `top_mounted_camera_d435` — each publishes depth + IR stereo + a derived `scan` (depthimage_to_laserscan) |
| **Merged perception** | `multicam_scan_merger` → `/multicam/merged_cloud`, `/multicam/merged_scan`; free-space detection (`freespace`, `potential_field`, `multicam/fs/*`) |
| **IMU** | Dual IMU (`imu1`, `imu2`) with gyro-offset/temperature; fused `/imu` |
| **Wheel odometry** | `/odom`, EKF-fused `/odom_combined` (via `graft`) |

---

## 4. How the Robot Is Driven (control pipeline)

All motion is arbitrated by **`movement_mux_node`**, which selects among three input
sources and forwards a single velocity command to the base driver:

```
[Physical joystick]  /robust/joy  ─────────────┐
[Autonomous nav]     /robust/nav_cmd_vel  ──────┼─►  movement_mux_node  ─►  /robust/cmd_vel  ─►  robot_driver  ─►  wheels
[Direct teleop]      /robust/teleop_stamped_movement_cmd ─┘
```

**Mux inputs / outputs**
- Subscribes: `/robust/joy` (`sensor_msgs/Joy`), `/robust/nav_cmd_vel`
  (`geometry_msgs/Twist`), `/robust/teleop_stamped_movement_cmd`
  (`robust_ros_msgs/StampedMovementCommand`)
- Publishes: `/robust/cmd_vel` (`geometry_msgs/Twist`) → consumed by `robot_driver`;
  plus `/robust/mux_status`.

### Three ways to operate

1. **Physical joystick (intended manual method).** A `joy_node` is running and feeds
   `/robust/joy` into the mux. Per Fetch docs (PS4 controller):
   **hold the deadman (button 10)**, then **right stick = forward/back, left stick = turn**.
   On this robot the joystick is routed through the Robust.AI mux rather than stock
   `fetch_teleop`.

2. **Autonomous navigation.** The behavior-tree / scene-graph planner publishes
   `/robust/nav_cmd_vel`; navigation goals are sent via e.g. `/pybtrees_nav/goal_wrt_map`.

3. **Direct velocity command (developer/debug).** Publish `geometry_msgs/Twist` to
   `/robust/nav_cmd_vel`. Takes effect only when the runstop is released **and** the mux
   grants that input priority.

> ⚠️ **This is a physical robot that can move and cause injury or damage.** Any motion test
> should be done only with the runstop released, a person in line-of-sight, the area clear,
> and the robot off its charging dock.

📄 **See [OPERATING_TESTS.md](OPERATING_TESTS.md)** for the safe, bounded, closed-loop motion
test procedure actually used on this unit (reusable script + verified results).

---

## 5. Safety & Power

### Runstop (emergency stop)
- Reported in `/robot_state` as `runstopped`. The runstop opens the motor breakers — **no
  software command will move the robot while it is engaged.**
- It is a **hardware safety**: released by physically twisting the red runstop button or
  toggling the aux/wireless runstop. Software **cannot** clear a hardware runstop.
- Software runstop (stock Fetch): publish to `/enable_software_runstop`, or press both right
  triggers (buttons 9 & 11) when teleop runstop is enabled (`-t` flag).

### Breakers (from `/robot_state`)
`computer_breaker`, `battery_breaker`, `supply_breaker`, `base_breaker`,
`aux_breaker_1`, `aux_breaker_2` — each with temperature + max-rated-temperature telemetry.
Controllable via the `robot_driver` `/breakers` and `/breaker_settings` action servers.

### Power / battery (`/battery_state`, power_msgs)
- Main battery nominal ~24 V (observed pack voltage ≈ 28 V on charge), supply rail ~48 V.
- Fields: `charge_level`, `is_charging`, `is_charger_detected`, `total_capacity`,
  `current_capacity`, `battery_voltage`, `supply_voltage`, `charger_voltage`.
- Charging/docking managed via `dock/result`, `charge_lockout` action, and the
  `fetch_open_auto_dock` / `fetch_auto_dock_msgs` packages.

---

## 6. Networking

| Interface | Address | Role |
|---|---|---|
| `eth1` | `10.42.42.1/24` | **Internal robot bus** — link to base MCBs / driver hardware |
| `wlan0` | DHCP (Wi-Fi) | Operator / fleet network |
| `eth0` | down | External wired (unused/no carrier) |
| `docker0` + several `br-*` | 172.x | Docker bridges (containerized Robust.AI services) |

Wi-Fi status is surfaced on `/wifi/status/led`. Remote-access tooling present on the box
includes cloudflared, teleport, and a Nebula-style overlay client (`dnclient`).

---

## 7. Key ROS Interfaces (cheat sheet)

**State (read-only)**
- `/robot_state` — runstop, ready, faulted, per-board & per-breaker telemetry
- `/battery_state` — charge %, charging status, voltages
- `/odom`, `/odom_combined` — pose/velocity
- `/base_scan`, `/multicam/merged_scan` — obstacle scans
- `/robust/mux_status` — which input currently owns motion
- `/diagnostics`, `/diagnostics_agg` — system health

**Command / control**
- `/robust/nav_cmd_vel` (Twist) — velocity into the mux
- `/robust/joy` (Joy) — joystick into the mux
- `/pybtrees_nav/goal_wrt_map` — navigation goal
- `/breakers`, `/breaker_settings`, `/charge_lockout`, `/led_settings` — driver action servers
- `/query_controller_states` — start/stop controllers
- `/enable_software_runstop` (stock Fetch) — software e-stop

> Note: several `robust_ros_msgs/*` message types (e.g. `StampedMovementMuxStatus`) are
> custom to the Robust.AI stack and require that workspace sourced to deserialize.

---

## 8. Live Node List (running processes)

```
/RosServiceAdapter        /diagnostic_aggregator     /robot_state_publisher
/btree  /btree_viz         /freespace                 /robot_pose_publisher
/pybtrees_nav (btree)      /graft                     /robust_multicam_realsense
/movement_mux_node         /handlebar_state_to_bool   /rs_camera_manager
/joy_control_node          /localization_status       /rtabmap + stereo_sync0..2
/joy_node                  /local_scene_graph         /sick_tim551_2050001 + filter
/multicam_scan_merger      /odom_relay                /static_transform_publisher_map_carto
/potential_field           /robot_driver              /robust_monitor / rosbag record
```

---

## 9. Official Documentation

- **Fetch & Freight Manual (Noetic):** https://fetchrobotics.github.io/docs/
- **Teleop tutorial (deadman + stick mapping, software runstop):** https://fetchrobotics.github.io/docs/teleop.html
- **Robot hardware overview:** https://docs.fetchrobotics.com/robot_hardware.html
- **Manual source (GitHub):** https://github.com/fetchrobotics/docs
- **Freight100 spec sheet (Zebra):** https://www.barcodesinc.com/media/pdf/Zebra/freight100-base-spec-2020-v3.pdf

> The official manuals describe the **stock** Fetch topics (`/cmd_vel`,
> `/enable_software_runstop`, `fetch_teleop`). This unit runs the **Robust.AI overlay**, so
> the live control topics are the `/robust/*` set above and standard tutorials will not map
> one-to-one.

---

## 10. Quick Start (operator)

```bash
# Point your machine at the robot's ROS master
export ROS_MASTER_URI=http://<robot-ip>:11311
source /opt/ros/noetic/setup.bash

# Confirm it's alive and safe to drive
rostopic echo -n1 /robot_state      # check: runstopped=false, ready=true, faulted=false
rostopic echo -n1 /battery_state    # check charge level / off-dock

# Manual drive: use the physical joystick — hold deadman (btn 10),
#   right stick = forward/back, left stick = turn.

# Send an autonomous nav goal instead:
#   publish to /pybtrees_nav/goal_wrt_map (via the Robust.AI app)
```

**Before driving:** release the hardware runstop, take the robot off its dock, and keep the
area clear with an operator in line-of-sight.

For a repeatable, bounded way to command motion (with a ready-to-run script and the results of
the test drives performed on this unit), see **[OPERATING_TESTS.md](OPERATING_TESTS.md)**.

### Web teleop

[`web_teleop.py`](web_teleop.py) serves a self-contained browser joystick (touch + WASD) that
drives the base through the movement mux — no rosbridge required.

```bash
source /opt/ros/noetic/setup.bash
python3 web_teleop.py          # then open http://<robot-ip>:8090
```

Safety is enforced server-side: hard speed caps (0.3 m/s / 0.6 rad/s), a 400 ms deadman
watchdog (release, tab close, or Wi-Fi drop → immediate zero), and live runstop/battery/odom
status. The hardware runstop always overrides. Amber **"no data"** chips mean the base driver
is publishing nothing — the robot cannot move in that state no matter what teleop sends.

📄 **See [OPERATING_TESTS.md §6](OPERATING_TESTS.md#6-web-teleop--running--verifying-web_teleoppy)**
for the full run & verify procedure (status chips, `/status` endpoint, drive checks), and
**[§7](OPERATING_TESTS.md#7-troubleshooting--teleop-runs-but-the-robot-does-not-move-root-caused-2026-07-17)**
for the "teleop runs but robot does not move" troubleshooting guide — including the base-driver
crash root-caused and fixed on 2026-07-17 (`robot_driver` SIGABRT on unwritable
`/var/log/ros/logpro`).

---

*Generated from live inspection of the running system.*
