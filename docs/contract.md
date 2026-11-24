# Inter-Group Contract

This document is the exact specification of what each group publishes and consumes. If a message is not listed here, it does not exist. Any change must be submitted via Pull Request and approved by at least one member of each group (`overhead`, `fleet`, `manipulator`).

## Conventions

- **Frame `arena`**: the origin is at the inner bottom-left corner of the overhead image, on the floor; `x` points toward Perry and `y` points toward the yellow sorting box, both on the floor; `z` points upward. See [`arena.md`](arena.md).
- **Units:** meters, radians, and seconds (ROS REP 103). Existing `multirobot_interfaces` messages already use meters. User interfaces may display centimeters.
- **Heading `theta`:** measured counterclockwise from `+x`, in the range `(-pi, pi]`.
- **ROS Domain ID:** `0`, same as stage 1. Distribution: ROS 2 Lyrical.
- **Clocks:** Raspberry Pi boards do not have a battery-backed RTC, so their wall clocks may differ from each other and from the overhead camera host. No safety rule may compare timestamps across different machines. Each node measures message age using its own local clock starting from reception time.

## Overhead Publishes

### `/world/state` · topic · target 20 Hz (minimum 15 Hz)

```text
WorldState
  header              stamp, frame_id = "arena"
  cubes[]             CubeObservation
    id                uint32   stable while the cube remains visible
    color             string   "blue" | "yellow" | "red"
    x, y              float64  m, cube center on the floor
    confidence        float32  0..1
  robots[]            RobotObservation
    name              string   "phineas" | "ferb"
    x, y              float64  m, robot center projected onto the floor (parallax-corrected)
    theta             float64  rad, scoop heading, CCW from +x in (-pi, pi]
    confidence        float32  0..1
```

### `/world/sensors` · topic · same rate (20 Hz)

Virtual sensors computed by `overhead` as recommended by the professor:

```text
VirtualSensors
  header              stamp, frame_id = "arena"
  robot_distance       float64  m, distance between robot centers (-1 if < 2 robots)
  robots_in_zone[]     string   robots currently inside the delivery zone region
  per_robot[]          RobotSensors
    name               string   "phineas" | "ferb"
    nearest_red_id     uint32   0 if no red cube is visible
    nearest_red_dist   float64  m (-1 if no red cube is visible)
    events[]           string   e.g. "entered:zone", "left:zone", "near:red:7", "near:robot"
```

> Overhead derives collision alerts (`robot_distance < 0.34 m` / `"near:robot"`) from the existing physical roof triangles (D3)—no physical red/green lights are added to the chassis. Each robot resolves evasion locally with the virtual red/green navigation-light rule, planning and validating clearance from red cubes (`≥ 0.12 m`) and borders before moving. (Adding `"near:robot"` to `overhead_core` requires coordination with `overhead`.)

### `/world/get` · service (`tsa_interfaces/srv/GetWorld`)

Empty request. Responds with the latest `WorldState` and `VirtualSensors` pair for on-demand queries.

### `/world/image` · topic · optional (5 Hz)

A `sensor_msgs/msg/CompressedImage` (JPEG, rectified top-down arena view) for the `viewer_node` at low rate (5 Hz is sufficient).

## Each Robot (Phineas / Ferb Pi 5) Publishes

Each robot runs its own planner, peer-to-peer traffic logic, controller, and HAL locally on its Raspberry Pi 5. Velocity commands `(vx, vy, wz)` are internal between the onboard controller and HAL (with a 300 ms local watchdog) rather than cross-machine topics.

### `/phineas/plan`, `/ferb/plan` · topic · ~5–10 Hz

Each robot publishes its own `tsa_interfaces/msg/RobotPlan` containing its current state (`IDLE`, `GO_TO_APPROACH`, `PUSH`, `REQUEST_ZONE`, `DELIVER`, `LEAVE`, `WAIT`, `STOPPED`, `RECOVER`), `target_cube_id`, `target_pose` (`geometry_msgs/Pose2D`), and `path` (`geometry_msgs/Point[]`). Used by the peer robot for distributed traffic coordination, by `overhead`'s `viewer_node`, and for logging.

> Note: `interfaces/tsa_interfaces/` currently also includes `FleetPlan` and `overhead`'s `viewer_node` currently subscribes to `/fleet/plan`; migrating the interface package and `viewer_node` to per-robot plan topics requires a cross-group PR.

### `/phineas/status`, `/ferb/status` · topic · 5 Hz

```text
RobotStatus
  line_detected   bool
  battery_v       float32  volts
  cmd_age_ms      uint32   age of the last internal velocity command received by the HAL in ms
  mode            string   "run" | "stopped_watchdog" | "stopped_line" | "manual"
```

## Manipulator (Unchanged from Stage 1)

Uses the existing `multirobot_interfaces` contract already tested across all three robots:

- `/zone/request`, `/zone/release`: services. Each robot calls them **directly** from its own Raspberry Pi 5, passing its own name as requester.
- `/zone/occupant`: heartbeat at 2 Hz. If 2 s pass without receiving it, no robot enters the delivery zone.
- `/perry/state`: arm state (`gate_closed` = delivery zone closed).
- Delivery zone lease duration: 60 s.

## Suggested QoS Profiles

| Topic / Service | QoS |
| --- | --- |
| `/world/state`, `/world/sensors`, `/world/image` | best effort, keep last 1 (sensor/stream data: latest sample wins) |
| `/<robot>/status`, `/<robot>/plan` | reliable, keep last 1 |
| `/world/get`, zone services | default service QoS |
