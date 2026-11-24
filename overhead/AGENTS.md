# AGENTS.md — overhead group

You are working for the **overhead group** (the fixed camera above the arena). This file is
binding for any agent or person implementing overhead work. It adds to the root
[`AGENTS.md`](../AGENTS.md); it never relaxes it. Read the root file first.

---

## 1. Scope: overhead only

| You may change | You may not change |
|---|---|
| `overhead/**` | `fleet/`, `manipulator/`, `sim/` (other groups own them) |
| Overhead sections of `docs/` only when a task below says so | Root files: `AGENTS.md`, `CLAUDE.md`, `pyproject.toml`, `.github/`, `.githooks/` |
| | `interfaces/multirobot_interfaces/` (never) |
| | `interfaces/tsa_interfaces/` and `docs/contract.md`, except through a **contract proposal PR** (section 6) |

If a task cannot be finished without touching something on the right-hand side, **stop and
report** what is needed and why. Do not work around it.

### The role, in one line

The overhead camera **sees, reports and relays; it never gives orders.** It publishes where
things are, what is near each robot and how close a collision is, and it is the only link
between Perry (the manipulator) and the robots (section 5, phase 2b). Mission, traffic, paths
and speeds belong to the fleet. If a task asks the overhead to choose a cube, plan a path,
command a robot, decide who yields or grant the zone, it is out of scope.

---

## 2. Read first

1. [`../AGENTS.md`](../AGENTS.md): code conventions (English only, constants, multiline style,
   `_core`/`_ros` split, Conventional Commits).
2. [`../docs/contract.md`](../docs/contract.md): the messages we publish.
3. [`../docs/arena.md`](../docs/arena.md): arena size, frames, parallax, triangle marker.
4. [`../docs/architecture.md`](../docs/architecture.md): how the groups fit together.

---

## 3. What exists today

| Piece | File | State |
|---|---|---|
| Arena corner detection, homography, rectified view, world ↔ robot frame | `overhead_core/geometry.py` | Done, tested |
| Honest calibration quad validation & session lifecycle | `overhead_core/calibration.py`, `overhead_core/arena_calibration.py` | Done, tested |
| Frame sources (camera & still image) and camera session | `overhead_core/frame_source.py`, `overhead_core/camera_session.py` | Done, tested |
| Camera state machine (`STARTING`, `CALIBRATING`, `RUNNING`, `DEGRADED`, `NO_IMAGE`) | `overhead_core/state_machine.py` | Done, tested |
| Parallax correction | `overhead_core/parallax.py` | Done, tested |
| Cube detection by HSV | `overhead_core/cubes.py` | Done, tested on synthetic images and real arena photo |
| Triangle pose (centroid + tip heading) | `overhead_core/triangle.py` | Done, tested on synthetic images and two-robot photo (heading and centre offsets pending calibration) |
| ArUco robot marker detector | `overhead_core/aruco_robots.py` | Done, tested on synthetic images (not yet glued on a robot) |
| Cube tracker with stable ids and occlusion grace | `overhead_core/tracker.py` | Done, tested |
| Lost robot hold filter with confidence decay | `overhead_core/robot_hold.py` | Done, tested |
| Robot velocity estimation (exponential smoothing) | `overhead_core/velocity.py` | Done, tested |
| Virtual sensors: robot distance, zone enter/leave, nearest red | `overhead_core/virtual_sensors.py` | Done, tested |
| Obstacles, radar, collision level, corridor query, world safety & pipeline | `overhead_core/obstacles.py`, `overhead_core/radar.py`, `overhead_core/collision.py`, `overhead_core/corridor.py`, `overhead_core/world_safety.py`, `overhead_core/world_pipeline.py` | Done, tested |
| Manipulator relay routing rules | `overhead_core/relay.py` | Done, tested |
| Standalone web viewer with safety overlay | `overhead_core/web_viewer.py`, `overhead_core/viewer_pipeline.py`, `overhead_core/viewer_overlay.py`, `overhead_core/viewer_safety.py` | Done, tested |
| Overhead node: `/world/state`, `/world/sensors`, `/world/get`, `/world/corridor_free`, `/world/image`, `/overhead/recalibrate` | `overhead_ros/overhead_node.py`, `overhead_ros/node_parameters.py`, `overhead_ros/world_publishers.py`, `overhead_ros/safety_inputs.py`, `overhead_ros/conversions.py` | Verified on ROS 2 Lyrical in container (still photos); **never run with a real robot or on the Raspberry Pi** |
| Manipulator relay node: `/<robot>/zone/request`, `/<robot>/zone/release`, `/<robot>/zone/occupant` | `overhead_ros/manipulator_relay_node.py` | Verified on ROS 2 Lyrical in container (`FakePerry`); **never run with real Perry/robots or on the Raspberry Pi** |
| Viewer node: world, robots and fleet plans drawn over the image | `overhead_ros/viewer_node.py` | Written, never run on hardware display |
| Robot link check (pure core + one-shot ROS CLI) | `overhead_core/robot_link.py`, `overhead_ros/robot_link_check_node.py`, `overhead_ros/config/robots.yaml` | Done, core tested; ROS CLI pending field test |
| Multi-arch Docker image & test runner (ROS 2 Lyrical) | `overhead/docker/` (`Dockerfile`, `compose.yaml`, `run-tests.sh`) | Done, verified on PC |
| Windows/WSL2 mirrored DDS link setup | `overhead/wsl/` (`robots.env`, `setup-wsl.ps1`, `wsl-entrypoint.sh`, `README.md`) | Done, pending Windows + Wi-Fi field test |
| Messages | `interfaces/tsa_interfaces/` | Contract v0 + O7/O20 proposals (pending approval) |

Development machines may be Windows without ROS. Pure tests (`python -m pytest
overhead/overhead_core`) run anywhere; `overhead_ros` integration tests run in the ROS 2 Lyrical
container (`docker compose run --rm overhead run-tests` in `overhead/docker`). Both `overhead_node`
and `manipulator_relay_node` have been verified on ROS 2 Lyrical in a container, **never** with a
real robot, real Perry, or the Raspberry Pi.

---

## 4. Defects

### Resolved defects

- **Silent bad calibration:** removed the frame-bounds fallback in `overhead_core/arena_calibration.py`
  and `overhead_core/calibration.py`; the node stays in `CALIBRATING` and publishes nothing until
  validated arena corners are found.
- **No state machine:** added pure `overhead_core/state_machine.py` and wired it into
  `overhead_ros/overhead_node.py` so `/world/state` and `/world/sensors` publish only in `RUNNING`
  and `DEGRADED`.
- **Inline strings in the node:** moved service response messages, log texts, and parameter defaults
  into `overhead_core/constants.py` and `overhead_ros/ros_names.py`.
- **Detection only tested on synthetic images:** added real arena photo fixtures under
  `overhead_core/test/fixtures/` and tests covering corner calibration and cube counts/colors.

### Open defects

1. **Provisional triangle markers pending calibration:** both robots now detected by red and
   orange provisional triangles; heading and centre offsets pending calibration.
2. **ArUco marker not yet mounted:** the ArUco marker has not yet been glued onto a physical robot.
3. **Raspberry Pi performance unmeasured:** tracker stability and publish rates have not yet been
   measured on the Raspberry Pi.
4. **Contract changes pending approval:** O7 (`Obstacle`, `RobotObstacles`, `CorridorFree`,
   `VirtualSensors` additions) and O20 (per-robot relay names) still await approval from the fleet
   and manipulator groups.
5. **Per-robot carrying state not wired:** `overhead_node.py` passes an empty `carrying_robots` set
   to `SafetyInputs.build_config`, so `CARRY_EXTENSION_M` is not yet applied live.

---

## 5. Implementation plan

Work the tasks in order. Each task is one branch and one PR to `develop`
(branch `feat/overhead-<short-name>`, commit scope `overhead_core` or `overhead_ros`).
A task is done when its listed tests exist and pass, `ruff format --check .` and
`ruff check .` are clean, and the PR description states what was verified on real ROS and
what was not.

### Phase 1 — Make the current node trustworthy

**O1. Camera state machine (core).** New `overhead_core/state_machine.py`: `OverheadState`
(`STARTING`, `CALIBRATING`, `RUNNING`, `DEGRADED`, `NO_IMAGE`) and a pure transition function
driven by events (camera opened, corners found, corners not found, frame missing for
`NO_FRAME_TIMEOUT_S = 0.5`, frame back, robot lost, robot seen again, recalibrate requested).
Transitions:

| From | Event | To |
|---|---|---|
| `STARTING` | camera opened | `CALIBRATING` |
| `CALIBRATING` | corners found | `RUNNING` |
| `CALIBRATING` | corners not found | `CALIBRATING` (retry) |
| `RUNNING` | a robot is not seen | `DEGRADED` |
| `DEGRADED` | all robots seen again | `RUNNING` |
| `RUNNING`, `DEGRADED` | no frame for `NO_FRAME_TIMEOUT_S` | `NO_IMAGE` |
| `NO_IMAGE` | frame back | `RUNNING` |
| `RUNNING`, `DEGRADED` | recalibrate requested or camera moved | `CALIBRATING` |

Tests: every row of the table, and that `STARTING`, `CALIBRATING` and `NO_IMAGE` never allow
publishing.
**Status:** Done — implemented in `overhead_core/state_machine.py` with full transition and publishing tests.

**O2. Use the state machine in the node.** Publish `/world/state` and `/world/sensors` only in
`RUNNING` and `DEGRADED`. Log every transition once. In `NO_IMAGE`, retry opening the camera
every `CAMERA_RETRY_S = 1.0`.
**Status:** Done — wired into `overhead_ros/overhead_node.py` and `overhead_core/camera_session.py`; verified in container.

**O3. Honest calibration.** Remove the frame-bounds fallback. Validate the detected quad: the
ratio of its sides must match `ARENA_WIDTH_M / ARENA_HEIGHT_M` within a tolerance constant, and
its area must be a minimum fraction of the image. On failure stay in `CALIBRATING` and retry.
Tests with synthetic images: good border accepted, cropped or skewed border rejected.
**Status:** Done — implemented in `overhead_core/calibration.py` and `overhead_core/arena_calibration.py` with blank-photo ROS test.

**O4. Lost robot handling.** When a triangle is not seen, keep the last pose for
`ROBOT_HOLD_S = 0.5` with confidence falling linearly to 0, then drop it and enter `DEGRADED`.
Pure helper in core, tested with fixed timestamps.
**Status:** Done — implemented in `overhead_core/robot_hold.py` and integrated into `overhead_core/world_pipeline.py`.

**O5. Real-photo regression tests.** Add the arena photos (downscaled to 1280 × 720, JPEG,
under 300 KB each) to `overhead_core/test/fixtures/`. Tests: corners found within a pixel
tolerance of the hand-measured ones; every cube in the photo found with the right color.
Tune default HSV ranges in `constants.py` until they pass.
**Status:** Done — real arena fixtures added under `overhead_core/test/fixtures/` with core and ROS tests passing.

**O6. Constants cleanup.** Move the node's inline strings and numbers to `constants.py` /
`ros_names.py`. Parameter defaults in `_declare_parameters` must come from constants.
**Status:** Done — node strings, defaults, and ROS names centralized in `constants.py` and `ros_names.py`.

### Phase 2 — Collision warnings

**O7. Contract proposal (needs the other groups).** Draft, in one PR touching only
`interfaces/tsa_interfaces/` and `docs/contract.md`:

```text
Obstacle            kind (RED_CUBE | ROBOT | CUBE | WALL | ZONE), id, x, y, radius, vx, vy
RobotObstacles      name, sectors[8], level (CLEAR | SLOW | STOP), time_to_collision,
                    nearest_kind, nearest_dist, nearest_bearing
VirtualSensors      + Obstacle[] obstacles, + RobotObstacles[] per_robot_obstacles
CorridorFree.srv    request: ax, ay, gx, gy, width  ->  response: free, blocking_ids[]
```

Mark the PR as needing approval from fleet and manipulator. **Do not merge it.** Tasks O8–O11
are written against these types but their core logic does not depend on ROS, so they can go
ahead while the PR waits.
**Status:** Done (proposal pending approval) — messages and `CorridorFree.srv` defined in `interfaces/tsa_interfaces/`, awaiting fleet/manipulator sign-off.

**O8. Velocity estimation (core).** `overhead_core/velocity.py`: velocity of each robot from
consecutive poses, smoothed (exponential, factor as a constant), robust to a skipped frame and to
`dt` near 0. Tests: constant motion, stop, one missing frame.
**Status:** Done — implemented and tested in `overhead_core/velocity.py`.

**O9. Obstacles with safety circles (core).** `overhead_core/obstacles.py`: build the list from
tracked cubes, robots and the arena. Safety radii as constants: `RED_CUBE_SAFETY_M = 0.17`,
`ROBOT_SAFETY_M = 0.17`, `CUBE_SAFETY_M = 0.05`, `CARRY_EXTENSION_M = 0.025` (added in front
of a robot carrying a cube). Arena walls from the arena size. The zone becomes an obstacle for
every robot except the one named in `/zone/occupant`.
**Status:** Done — implemented and tested in `overhead_core/obstacles.py` and `overhead_core/world_safety.py`.

**O10. Robot radar (core).** `overhead_core/radar.py`: 8 sectors of 45°, sector 0 centred on the
scoop, each holding the free distance to the nearest safety circle or wall in that sector
(ray–circle and ray–segment distance in the robot frame). Tests: obstacle straight ahead lands
in sector 0, one on the left in sector 2, wall distances on an empty arena.
**Status:** Done — implemented and tested in `overhead_core/radar.py`.

**O11. Collision level and time to collision (core).** `overhead_core/collision.py`:
closest approach between two moving circles with constant velocities, and the level using only
the cone of ±60° around the scoop:

| Level | Free distance in the cone | or time to collision |
|---|---|---|
| CLEAR | more than 0.60 m | more than 2 s |
| SLOW | 0.40 to 0.60 m | 1 to 2 s |
| STOP | less than 0.40 m | less than 1 s |

All thresholds are constants. Tests: head-on robots → STOP, diverging robots → CLEAR,
obstacle behind the robot does not raise the level.
**Status:** Done — implemented and tested in `overhead_core/collision.py`.

**O12. Corridor query (core + ros).** `overhead_core/corridor.py`: is the segment A → G at
least `width / 2` away from every red safety circle; return the blocking ids. Service
`/world/corridor_free` in the node once O7 is approved.
**Status:** Done — implemented in `overhead_core/corridor.py` and `/world/corridor_free` in `overhead_ros/overhead_node.py`.

**O13. Wire obstacles into the node.** Subscribe to `/zone/occupant` (`std_msgs/String`),
compute O8–O11 each cycle, fill the new fields of `/world/sensors`. Only after O7 is merged.
**Status:** Done — wired via `overhead_ros/safety_inputs.py`, `world_publishers.py`, and `overhead_node.py` (awaiting O7 approval).

### Phase 2b — Manipulator relay

The overhead group owns the communication with Perry. Perry's arbiter and its contract
(`multirobot_interfaces`, `/zone/*`) stay unchanged; only the overhead talks to it. Every
message from Perry is either **for us** (we consume it) or **for a robot** (we forward it
unchanged). The relay forwards, it never decides: it does not grant, deny, delay or rewrite
anything.

| From Perry | For | What we do |
|---|---|---|
| `/zone/occupant` (heartbeat, 2 Hz) | us | Zone becomes an obstacle for every robot except the occupant (O9) |
| `/zone/occupant` (heartbeat, 2 Hz) | robots | Republish per robot as `/<robot>/zone/occupant` while it is fresh |
| `/perry/state` (`gate_closed`, count) | us | Zone obstacle for everyone while the gate is closed; count shown in the viewer |
| `/zone/request` response | the robot that asked | Return Perry's response as is |
| `/zone/release` response | the robot that released | Return Perry's response as is |

| From a robot | We forward to Perry |
|---|---|
| `/<robot>/zone/request` (`RequestZone`) | `/zone/request`, same request |
| `/<robot>/zone/release` (`ReleaseZone`) | `/zone/release`, same request |

**O17. Routing rules (core).** `overhead_core/relay.py`, no ROS: given a message kind and its
fields, return the destinations (`US`, a robot name, or both) and whether the data is still fresh
(`PERRY_HEARTBEAT_TIMEOUT_S = 2.0`, measured with our own clock from reception). Stale data is
never forwarded. Tests: every row of the two tables, stale heartbeat dropped, unknown robot
name rejected.
**Status:** Done — implemented and tested in `overhead_core/relay.py`.

**O18. Relay node (ros).** `overhead_ros/manipulator_relay_node.py`: subscribes to Perry's
topics, offers one `RequestZone` / `ReleaseZone` service per robot and calls Perry's services
asynchronously (never block a callback; time out after `PERRY_CALL_TIMEOUT_S` and return
`granted=false` / `ok=false` with the timeout logged). Topic and service names in
`ros_names.py`. Launched from `overhead.launch.py`.
**Status:** Done — implemented in `overhead_ros/manipulator_relay_node.py`, launched by `overhead.launch.py`, verified with `FakePerry`.

**O19. Heartbeat safety.** If Perry's heartbeat is older than `PERRY_HEARTBEAT_TIMEOUT_S`, stop
republishing `/<robot>/zone/occupant`, so each robot sees the silence and applies its own
2 s stop rule. Never publish a heartbeat that did not come from Perry.
**Status:** Done — enforced in `overhead_ros/manipulator_relay_node.py` and verified by ROS integration tests.

**O20. Contract proposal for the relay.** The per-robot names above and the change of who calls
Perry go in the contract (section 6). Decision D8 today says the fleet requests the zone on
behalf of each robot; this replaces that path and needs the fleet's and Perry's approval.
Implement O17–O19 against the proposed names behind parameters until it is approved.
**Status:** Done (proposal pending approval) — implemented behind parameters in `manipulator_relay_node.py`, awaiting fleet and Perry approval.

### Phase 3 — Real hardware

**O14. Calibration checklist on the real arena.** Measure and record in
`overhead_ros/config/overhead.yaml` (with the date and who measured in a comment):
marker height above the floor (today an estimate of 0.09 m from the chassis height), camera
centre (detect the red dot and compare with `[0.5825, 0.35]`), delivery zone rectangle,
triangle HSV ranges with the real cardstock. (Still open — needs physical arena.)

**O15. Measure the loop.** On the camera PC: publish rate of `/world/state` (target 20 Hz,
minimum 15 Hz) and capture-to-publish latency. Record the numbers in the PR. If the rate is
below 15 Hz, profile before changing anything. (Still open — needs physical arena and Raspberry Pi.)

**O16. Recordings for the other groups.** Record a few minutes of `/world/state`,
`/world/sensors` and `/world/image` with `ros2 bag record` so fleet and `sim/` can test without
the camera. Store them outside git; link them in the PR. (Still open — needs physical arena.)

---

## 6. Changing the contract

Messages are shared by three groups. To add or change anything in `interfaces/tsa_interfaces/`:

1. Open a PR that touches only `interfaces/tsa_interfaces/` and `docs/contract.md`.
2. Explain who publishes it, who consumes it, rate and QoS.
3. Wait for one approval from each group. Never merge it yourself and never change an existing
   field: add new fields or new messages.

---

## 7. Open questions (do not decide them alone)

- Red safety radius: 0.17 m, or 0.12 m plus a free corridor (decision D7, to be tested in `sim/`).
- Do the robots read their own `RobotObstacles` for a braking reflex, or only the fleet?
- Triangle colors for Phineas and Ferb (not blue, yellow, red or green).
- Is the camera exactly above the red dot?
- Should `WorldState` carry a capture-to-publish latency field?
- Relay: who calls `/<robot>/zone/request`, the robot's own Pi or the fleet brain on its behalf?
  And does any Perry message besides the zone ones need to reach the robots?

When one of these blocks a task, implement it behind a parameter with the current default and
note the question in the PR.

---

## 8. Working notes (ROS 2 Lyrical gotchas)

- `geometry_msgs/Pose2D` was removed in ROS 2 Lyrical; use `tsa_interfaces/Pose2D` instead.
- ROS 2 parameters cannot be nested arrays; HSV ranges are stored as flat 6-element integer
  lists `[h_lo, s_lo, v_lo, h_hi, s_hi, v_hi]`.
- `logger.warn` was removed in Lyrical; always use `logger.warning`.
- Never name a `Node` attribute `_services`, `_clients`, or `_publishers`: they clash with
  `rclpy.node.Node` internals and corrupt the executor wait set.
- Async service callbacks must `await` an `rclpy.task.Future` (and keep async services in a
  `MutuallyExclusiveCallbackGroup` while outgoing clients and timers stay in a
  `ReentrantCallbackGroup` under `MultiThreadedExecutor`).
- FastDDS discovery under Docker Desktop requires running multi-process checks inside the same
  container (with `network_mode: host` and an isolated `ROS_DOMAIN_ID` per test run).
- Live Wi-Fi robot runs from Windows 11 must run inside the `tsa-ros` WSL2 distro with
  `networkingMode=mirrored`, `ROS_STATIC_PEERS` (`overhead/wsl/robots.env`), and the
  Windows/Hyper-V firewall rules (`overhead/wsl/setup-wsl.ps1`).
- **Triangle heading rules (`TriangleHeadingRule` / `triangle_heading_rule`):**
  - `longest_side_normal` (default): finds the longest edge of the detected triangle (which lies
    along the robot rear edge on the provisional right-ish markers) and takes its unit normal
    pointing toward the third (opposite) vertex. The marker position remains the triangle
    centroid before per-robot calibration offsets.
  - `opposite_shortest_side`: points from the triangle centroid to the vertex opposite the
    shortest edge (for elongated isosceles markers as originally assumed in `docs/arena.md`).
