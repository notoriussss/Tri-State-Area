# Architecture

## Roles

| Group | Responsibility | Runs on |
| --- | --- | --- |
| Overhead | Sees: calibrates the camera, detects cubes and robots, publishes the world, computes virtual sensors, and displays the on-screen virtual projection (viewer). Only provides data, never gives orders | Camera PC or Pi |
| Fleet | Decides and moves: autonomous mission, peer-to-peer traffic, path control, HAL, watchdog, and line sensor reflexes for Phineas and Ferb | Each robot's Pi 5 |
| Manipulator | Improves Perry (gripper, speed, precision), maintains the delivery-zone arbiter and cube count | Perry's Pi |

In short: `overhead` sees everything but gives no commands, while `fleet` makes decisions and acts but relies on `overhead` for global vision.

## Paradigm

A **hybrid, decentralized** paradigm is used. Each robot plans autonomously on its own Raspberry Pi 5 using the full overhead view of the arena, while retaining two local safety reflexes:

- If it detects the black border line, it stops.
- If 300 ms pass without a new local control command to the HAL, it stops (watchdog).

## Layers

| Layer | Contents | Owner |
| --- | --- | --- |
| C.A. (control application) | Mission (which cube each robot takes), peer-to-peer traffic (who goes and who waits), sorting (Perry) | fleet (each Pi), manipulator |
| A · S · A,S V · C | Actuators (each robot's velocity), sensors (overhead camera, line sensor), virtual sensors and actuators, communication (the contract) | fleet, overhead, all |
| HAL (hardware abstraction) | `drive(vx, vy, wz)`, `stop()`, `line()`. Phineas and Ferb differ only in their configuration file | fleet |
| RT (runtime) | Loops at ~20 Hz on overhead and each robot's Pi; 300 ms internal HAL command watchdog on each robot | all |
| HW | Camera (1 m height) + camera host (PC or Pi), 2 × (Pi 5, Fusion HAT+, 4 TT motors, TCRT5000), Perry | — |

VCI appears in the Sand Box diagram. We assume it stands for *Virtual Communication Interface*, though this is not yet confirmed.

## Repository Layout

The repository is a single ROS 2 colcon monorepo without a top-level `src/` directory. Packages and folders are organized by owning group and modularity:

```text
Tri-State-Area/
├── README.md
├── docs/
│   ├── architecture.md
│   ├── contract.md
│   ├── arena.md
│   ├── decisions.md
│   └── open-questions.md
├── interfaces/
│   ├── multirobot_interfaces/    [untouched from stage 1]
│   └── tsa_interfaces/           [shared ROS 2 message and service definitions]
├── overhead/
│   ├── overhead_core/            [pure Python, no rclpy: geometry, parallax, cubes, triangle, tracker, virtual sensors]
│   └── overhead_ros/             [thin ROS 2 nodes: overhead_node and viewer_node]
├── fleet/
│   └── README.md                 [placeholder: fleet group owns this folder and internal layout]
├── manipulator/
│   └── README.md                 [placeholder: Perry arm code from Robot_Claw, zone arbiter unchanged]
├── sim/
│   └── README.md                 [placeholder: fake world publishing the overhead contract]
└── .github/
    └── CODEOWNERS                [overhead -> @overhead-team, fleet -> @fleet-team, manipulator -> @manipulator-team, interfaces -> all three]
```

Each group owns its dedicated folder. Pure algorithms live in `_core` packages tested with synthetic data and standard pytest without ROS or hardware dependencies, while ROS nodes live in `_ros` packages. The `interfaces/tsa_interfaces` contract is shared and protected by `.github/CODEOWNERS`.

### Reused from Previous Repositories

- `kinematics.py` and `motors.py` (Fusion HAT+, mecanum): move to `fleet/` (onboard robot HAL).
- `multirobot_interfaces` and Perry's zone arbiter: form the basis of `interfaces/` and `manipulator/`.
- `peer_avoidance.py`, already written in arena coordinates: reference implementation for fleet traffic control.
- The "pure tested core + thin ROS node" pattern and the decision log (`docs/decisions.md`).

### Not Carried Over

- Onboard navigation perception: beacons, color strips, tilted-camera projection model.
- Reactive pilots `autopilot.py` and `behavior_fsm` (two duplicate state machines).
- Sensor nodes that were never mounted on hardware.
- The Webots simulation, which models a different robot.

## Application Flow

The system runs as a control loop at ~20 Hz:

1. **See** (`overhead`): capture frame, convert pixels to meters, correct parallax, and publish `/world/state`.
2. **Interpret** (`overhead`): compute virtual sensors and publish them on `/world/sensors`.
3. **Decide** (`fleet`, onboard each robot's Pi 5):
   - Mission selects or maintains a target cube for that robot.
   - Anti-collision uses overhead proximity alerts (< 0.34 m) and virtual navigation-light rules (port = red, starboard = green), validating clearance from red cubes (≥ 0.12 m) and arena borders before moving.
   - Control converts the goal into body velocities locally on the Pi.
4. **Act** (`fleet`, onboard HAL on the same Pi 5): receives local `(vx, vy, wz)` and drives the motors. Stops immediately if the black line is detected or if the internal command watchdog times out.

### Complete Delivery Sequence

```text
Overhead -> Phineas : /world/state and /world/sensors (20 Hz)
Phineas             : selects blue cube #3, publishes /phineas/plan, drives locally to A and pre-zone
Phineas  -> Perry   : /zone/request (phineas)
Perry    -> Phineas : granted (60 s lease)
Phineas             : pushes cube past the green line (local control)
Overhead -> Phineas : cube #3 inside delivery zone
Phineas             : backs up straight and exits zone (local control)
Overhead -> Phineas : virtual sensor: Phineas left the zone
Phineas  -> Perry   : /zone/release (1 blue)
Perry               : settling -> sorts cube
```

Each robot requests and releases the delivery-zone lease directly from its own Raspberry Pi 5, keeping Perry's stage-1 zone arbiter contract unchanged.

### Robot States (Onboard Each Robot)

```text
IDLE -> GO_TO_APPROACH -> PUSH -> REQUEST_ZONE -> DELIVER -> LEAVE -> IDLE
Interrupts: WAIT (traffic) · STOPPED (no camera data > 0.5 s) · RECOVER (cube slipped off the scoop)
```

### Go-to-Pose Control (Mecanum)

```text
e        = p_target - p_robot                   (world frame)
[vx, vy] = sat( K · R(-theta) · e )             (robot frame)
wz       = sat( K_theta · (theta_target - theta) )
```

### Guided Push

The robot aligns at an approach point `A` behind cube `C` along the line toward goal `G`, then pushes in a straight line:

```text
A = C + d · (C - G) / |C - G|
```

### Anti-Collision: Standard Navigation Lights (Red / Green)

- **Physical markers & overhead alert:** `overhead` tracks the existing physical cardstock roof triangles (D3) to obtain `(x, y, theta)` and publish collision alerts on `/world/sensors` (`robot_distance < 0.34 m` / `"near:robot"`). No physical red or green stickers or LEDs are mounted on the chassis so HSV detection of red obstacle cubes and the green delivery line is never contaminated.
- **Right-of-way rules (aligned with direction of motion):**
  1. *Restricted maneuverability:* `LEAVE`, `DELIVER`, or `PUSH` (carrying a cube) has priority over an empty robot.
  2. *Same condition:* seeing **red** (peer entering from your right) → yield; seeing **green** (peer entering from your left) → hold course; **head-on** → both sidestep to starboard (`-vy` mecanum; tie-break: `ferb` yields); **overtaking** → rear robot yields/bypasses.
- **Plan before moving (20 Hz closed-loop):** before commanding a starboard evasion (`~0.15–0.20 m`), the Pi 5 validates in `< 0.1 ms` against `/world/state` that the projected step stays inside the virtual border margin and `≥ 0.12 m` from every red cube (preventing knocking lightweight 2.5 cm red cubes or triggering `stopped_line`). If starboard is blocked, the robot stops safely (`WAIT`) and evaluates a safe straight reverse (`-vx`) if blocked `> 3 s`. The check repeats at ~20 Hz while moving.

## Language

Python with ROS 2 Lyrical on Ubuntu 26.04 is used across the project. Neither Go nor Rust is justified, for the following reasons:

- **Latency, not compute, is the bottleneck.** Wi-Fi ping is 560–870 ms (measured by the Phineas team), each camera frame takes 33–66 ms, and Phineas already oscillated with just 0.15 s of delay. None of those numbers change with the programming language.
- **OpenCV is already C++ under the hood.**
- **ROS 2 officially supports only Python (`rclpy`) and C++ (`rclcpp`).**
- **Inherited code is in Python**, with ~280 unit tests.

If profiling shows that a specific node is a CPU bottleneck, that individual node will be optimized in C++ first.
