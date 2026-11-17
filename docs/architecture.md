# Architecture

## Roles

| Group | Responsibility | Runs on |
| --- | --- | --- |
| Overhead | Sees: calibrates the camera, detects cubes and robots, publishes the world, computes virtual sensors, and displays the on-screen virtual projection (viewer). Only provides data, never gives orders | Camera PC or Pi |
| Fleet | Decides and moves: mission, traffic, and control of Phineas and Ferb (central brain), plus onboard robot code (HAL, watchdog, line sensor) | PC (brain) and each robot's Pi 5 |
| Manipulator | Improves Perry (gripper, speed, precision), maintains the delivery-zone arbiter and cube count | Perry's Pi |

In short: `overhead` sees everything but gives no commands, while `fleet` makes decisions and acts but relies on `overhead` for global vision.

## Paradigm

A **hybrid** paradigm is used. The central brain plans using the full view of the arena. Each robot retains two local reflexes:

- If it detects the black border line, it stops.
- If 300 ms pass without a new command, it stops (watchdog).

## Layers

| Layer | Contents | Owner |
| --- | --- | --- |
| C.A. (control application) | Mission (which cube each robot takes), traffic (who goes and who waits), sorting (Perry) | fleet, manipulator |
| A · S · A,S V · C | Actuators (each robot's velocity), sensors (overhead camera, line sensor), virtual sensors and actuators, communication (the contract) | fleet, overhead, all |
| HAL (hardware abstraction) | `drive(vx, vy, wz)`, `stop()`, `line()`. Phineas and Ferb differ only in their configuration file | fleet |
| RT (runtime) | Loops at ~20 Hz; command watchdog on each robot | all |
| HW | Camera (1 m height), PC, 2 × (Pi 5, Fusion HAT+, 4 TT motors, TCRT5000), Perry | — |

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
3. **Decide** (`fleet`):
   - Mission assigns or maintains a target cube per robot.
   - Traffic stops the lower-priority robot if their safety circles are about to intersect.
   - Control converts the goal into body velocities.
4. **Act** (`fleet` onboard robot): receives `(vx, vy, wz)` and drives the motors. Stops immediately if the black line is detected or if the command times out.

### Complete Delivery Sequence

```text
Overhead -> Fleet   : /world/state (20 Hz)
Fleet               : assigns blue cube #3 to Phineas
Fleet    -> Phineas : /phineas/cmd (vx, vy, wz)    ... drive to approach point A
Fleet    -> Perry   : /zone/request (phineas)
Perry    -> Fleet   : granted (60 s lease)
Fleet    -> Phineas : push cube past the green line
Overhead -> Fleet   : cube #3 inside delivery zone
Fleet    -> Phineas : back up and exit zone
Overhead -> Fleet   : virtual sensor: Phineas left the zone
Fleet    -> Perry   : /zone/release (1 blue)
Perry               : settling -> sorts cube
```

`fleet` requests the delivery-zone lease on behalf of each robot so Perry's stage-1 zone arbiter contract stays unchanged.

### Robot States (Inside Fleet)

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

## Language

Python with ROS 2 Lyrical on Ubuntu 26.04 is used across the project. Neither Go nor Rust is justified, for the following reasons:

- **Latency, not compute, is the bottleneck.** Wi-Fi ping is 560–870 ms (measured by the Phineas team), each camera frame takes 33–66 ms, and Phineas already oscillated with just 0.15 s of delay. None of those numbers change with the programming language.
- **OpenCV is already C++ under the hood.**
- **ROS 2 officially supports only Python (`rclpy`) and C++ (`rclcpp`).**
- **Inherited code is in Python**, with ~280 unit tests.

If profiling shows that a specific node is a CPU bottleneck, that individual node will be optimized in C++ first.
