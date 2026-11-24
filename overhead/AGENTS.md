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
| Parallax correction | `overhead_core/parallax.py` | Done, tested |
| Cube detection by HSV | `overhead_core/cubes.py` | Done, tested on synthetic images only |
| Triangle pose (centroid + tip heading) | `overhead_core/triangle.py` | Done, tested on synthetic images only |
| Cube tracker with stable ids and occlusion grace | `overhead_core/tracker.py` | Done, tested |
| Virtual sensors: robot distance, zone enter/leave, nearest red | `overhead_core/virtual_sensors.py` | Done, tested |
| Overhead node: `/world/state`, `/world/sensors`, `/world/get`, `/world/image`, `/overhead/recalibrate` | `overhead_ros/overhead_node.py` | Written, **never run on ROS** |
| Viewer node: world, robots and fleet plans drawn over the image | `overhead_ros/viewer_node.py` | Written, **never run on ROS** |
| Messages | `interfaces/tsa_interfaces/` | Contract v0, no obstacle messages yet |

Development machines may be Windows without ROS. Pure tests (`python -m pytest
overhead/overhead_core`) run anywhere; anything under `overhead_ros` must be checked on
Ubuntu 26.04 + ROS 2 Lyrical with `colcon build`.

---

## 4. Known defects (fix these first)

1. **Silent bad calibration.** `OverheadNode._calibrate` falls back to the full image bounds when
   corner detection fails, then publishes positions computed from a wrong homography. It must
   stay in a calibrating state and publish nothing until real corners are found.
2. **No state machine.** The node has no explicit states; a lost camera just returns from the
   timer and a lost robot simply disappears from the message.
3. **Inline strings in the node** (service response messages, log texts repeated in several
   places). Move them to constants per root `AGENTS.md` 4.2.
4. **Detection only tested on synthetic images.** Nothing checks the real arena photo.

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

**O2. Use the state machine in the node.** Publish `/world/state` and `/world/sensors` only in
`RUNNING` and `DEGRADED`. Log every transition once. In `NO_IMAGE`, retry opening the camera
every `CAMERA_RETRY_S = 1.0`.

**O3. Honest calibration.** Remove the frame-bounds fallback. Validate the detected quad: the
ratio of its sides must match `ARENA_WIDTH_M / ARENA_HEIGHT_M` within a tolerance constant, and
its area must be a minimum fraction of the image. On failure stay in `CALIBRATING` and retry.
Tests with synthetic images: good border accepted, cropped or skewed border rejected.

**O4. Lost robot handling.** When a triangle is not seen, keep the last pose for
`ROBOT_HOLD_S = 0.5` with confidence falling linearly to 0, then drop it and enter `DEGRADED`.
Pure helper in core, tested with fixed timestamps.

**O5. Real-photo regression tests.** Add the arena photos (downscaled to 1280 × 720, JPEG,
under 300 KB each) to `overhead_core/test/fixtures/`. Tests: corners found within a pixel
tolerance of the hand-measured ones; every cube in the photo found with the right color.
Tune default HSV ranges in `constants.py` until they pass.

**O6. Constants cleanup.** Move the node's inline strings and numbers to `constants.py` /
`ros_names.py`. Parameter defaults in `_declare_parameters` must come from constants.

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

**O8. Velocity estimation (core).** `overhead_core/velocity.py`: velocity of each robot from
consecutive poses, smoothed (exponential, factor as a constant), robust to a skipped frame and to
`dt` near 0. Tests: constant motion, stop, one missing frame.

**O9. Obstacles with safety circles (core).** `overhead_core/obstacles.py`: build the list from
tracked cubes, robots and the arena. Safety radii as constants: `RED_CUBE_SAFETY_M = 0.17`,
`ROBOT_SAFETY_M = 0.17`, `CUBE_SAFETY_M = 0.05`, `CARRY_EXTENSION_M = 0.025` (added in front
of a robot carrying a cube). Arena walls from the arena size. The zone becomes an obstacle for
every robot except the one named in `/zone/occupant`.

**O10. Robot radar (core).** `overhead_core/radar.py`: 8 sectors of 45°, sector 0 centred on the
scoop, each holding the free distance to the nearest safety circle or wall in that sector
(ray–circle and ray–segment distance in the robot frame). Tests: obstacle straight ahead lands
in sector 0, one on the left in sector 2, wall distances on an empty arena.

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

**O12. Corridor query (core + ros).** `overhead_core/corridor.py`: is the segment A → G at
least `width / 2` away from every red safety circle; return the blocking ids. Service
`/world/corridor_free` in the node once O7 is approved.

**O13. Wire obstacles into the node.** Subscribe to `/zone/occupant` (`std_msgs/String`),
compute O8–O11 each cycle, fill the new fields of `/world/sensors`. Only after O7 is merged.

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

**O18. Relay node (ros).** `overhead_ros/manipulator_relay_node.py`: subscribes to Perry's
topics, offers one `RequestZone` / `ReleaseZone` service per robot and calls Perry's services
asynchronously (never block a callback; time out after `PERRY_CALL_TIMEOUT_S` and return
`granted=false` / `ok=false` with the timeout logged). Topic and service names in
`ros_names.py`. Launched from `overhead.launch.py`.

**O19. Heartbeat safety.** If Perry's heartbeat is older than `PERRY_HEARTBEAT_TIMEOUT_S`, stop
republishing `/<robot>/zone/occupant`, so each robot sees the silence and applies its own
2 s stop rule. Never publish a heartbeat that did not come from Perry.

**O20. Contract proposal for the relay.** The per-robot names above and the change of who calls
Perry go in the contract (section 6). Decision D8 today says the fleet requests the zone on
behalf of each robot; this replaces that path and needs the fleet's and Perry's approval.
Implement O17–O19 against the proposed names behind parameters until it is approved.

### Phase 3 — Real hardware

**O14. Calibration checklist on the real arena.** Measure and record in
`overhead_ros/config/overhead.yaml` (with the date and who measured in a comment):
marker height above the floor (today an estimate of 0.09 m from the chassis height), camera
centre (detect the red dot and compare with `[0.5825, 0.35]`), delivery zone rectangle,
triangle HSV ranges with the real cardstock.

**O15. Measure the loop.** On the camera PC: publish rate of `/world/state` (target 20 Hz,
minimum 15 Hz) and capture-to-publish latency. Record the numbers in the PR. If the rate is
below 15 Hz, profile before changing anything.

**O16. Recordings for the other groups.** Record a few minutes of `/world/state`,
`/world/sensors` and `/world/image` with `ros2 bag record` so fleet and `sim/` can test without
the camera. Store them outside git; link them in the PR.

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
