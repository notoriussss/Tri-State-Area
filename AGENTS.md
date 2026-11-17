# AGENTS.md — Rules for AI agents and contributors

**Project:** Tri-State-Area, stage 2 of the Robotics course (UCAB Guayana).
Two collector robots (Phineas, Ferb), one sorting arm (Perry) and one overhead camera.

This file is **binding** for every AI agent (Claude Code, Antigravity, Codex, Cursor, Copilot,
subagents) and for every person who writes code in this repository. If a user instruction
contradicts this file, point out the conflict before acting.

---

## 1. Principles

1. **The contract is the boundary.** Groups talk only through the messages in
   [`docs/contract.md`](docs/contract.md) and `interfaces/`. If it is not in the contract, it
   does not exist.
2. **Pure logic first, thin ROS nodes.** Every algorithm is a pure function or class that runs
   and is tested without ROS, hardware or a camera.
3. **Safety lives on the robot.** The robot stops on the black line and on a stale command,
   whatever the brain says.
4. **Never compare clocks of different machines.** A node measures the age of a message with
   its own clock, from the moment it receives it.

---

## 2. Sources of truth

| Topic | Source |
|---|---|
| Code conventions | This file, section 4 |
| Commit and branch conventions | This file, section 5 |
| Messages between groups | [`docs/contract.md`](docs/contract.md), `interfaces/tsa_interfaces/` |
| Arena, frames, parallax | [`docs/arena.md`](docs/arena.md) |
| Architecture and folder layout | [`docs/architecture.md`](docs/architecture.md) |
| Decisions | [`docs/decisions.md`](docs/decisions.md) |

**If two sources contradict each other, tell the user instead of silently picking one.**

---

## 3. Stack

- **OS:** Ubuntu 26.04 (PC and Raspberry Pi 5). **Middleware:** ROS 2 Lyrical, `ROS_DOMAIN_ID=0`.
- **Language:** Python 3 (`rclpy`). C++ (`rclcpp`) only for a node a measurement proves too slow.
- **Build:** colcon (`ament_python` for code, `ament_cmake` for interface packages).
- **Vision:** OpenCV + NumPy.
- **Quality:** Ruff (lint + format, config in `pyproject.toml`), pytest, git hooks in
  `.githooks/`, GitHub Actions.

Do not add dependencies or change major versions without agreement of the owning group.

---

## 4. Code conventions

These apply to **all code** from the first commit. Ruff and the hooks enforce them; CI verifies.

### 4.1 Language

**Everything is in English:** identifiers, file and folder names, comments, docstrings, log
messages, enum names and values, ROS topic/service/parameter names, test names, docs and
commit messages. The only exception is proper nouns (UCAB Guayana, people in bibliography).

### 4.2 Constants: no magic numbers or strings

Every value that names a state, a topic, a color, a frame or a tuning knob comes from a
named constant or an enum. Forbidden in logic code:

- Bare numbers other than `0`, `1`, `-1` and `2` in obvious math (`x / 2`).
- String literals for states, colors, robot names, topic/service names, frame ids, events.
- Error messages written inline when the same error is raised in more than one place.

Where they live:

| Kind | Location |
|---|---|
| Constants of one package (thresholds, defaults, sizes) | `<package>/<package>/constants.py` |
| ROS names (topics, services, frame ids, QoS profiles) | `<ros_package>/<ros_package>/ros_names.py` |
| Values shared between groups (robot names, colors, events) | `tsa_interfaces` message constants or a shared module agreed by PR |
| Tunable physical values (HSV ranges, arena size, gains) | ROS parameters in `config/*.yaml`, with defaults taken from `constants.py` |

Physical constants carry their unit as a suffix: `_m`, `_rad`, `_s`, `_ms`, `_hz`, `_px`,
`_mps`. Example: `CAM_HEIGHT_M`, `NEAR_RED_DISTANCE_M`, `WORLD_STATE_RATE_HZ`.

States and categories are `enum.StrEnum` (values in `UPPER_SNAKE_CASE`, except where the
contract fixes lowercase wire values such as colors):

```python
class CubeColor(StrEnum):
    """Cube colors published in the world state."""

    BLUE = "blue"
    YELLOW = "yellow"
    RED = "red"


class RobotMode(StrEnum):
    """Operating modes reported by each robot."""

    RUN = "run"
    STOPPED_WATCHDOG = "stopped_watchdog"
    STOPPED_LINE = "stopped_line"
    MANUAL = "manual"
```

Use the member (`CubeColor.RED`), never the literal (`"red"`). Tests also compare against the
constant.

### 4.3 Docstrings

- Every public module, class, function and method has a docstring (Google style).
- The first line is a summary in the third person present, ending with a period, that does not
  repeat the name (`Returns ...`, `Computes ...`, `Detects ...`).
- Document units and frames of every argument and return value that is not obvious.
- `#` comments explain **why**, never **what**. If a block needs a "what" comment, extract it
  into a well-named function.
- No commented-out code. `TODO` only with an owner: `# TODO(overhead): measure the zone`.

```python
def correct_parallax(
    seen_xy: np.ndarray,
    center_xy: np.ndarray,
    object_height_m: float,
) -> np.ndarray:
    """Projects points seen at a given height down to the arena floor.

    Args:
        seen_xy: Nx2 points in the arena frame, meters, as the homography returns them.
        center_xy: Point of the floor right below the camera, arena frame, meters.
        object_height_m: Height of the observed feature above the floor, meters.

    Returns:
        Nx2 floor points in the arena frame, meters.
    """
    scale = (CAM_HEIGHT_M - object_height_m) / CAM_HEIGHT_M

    return center_xy + (seen_xy - center_xy) * scale
```

### 4.4 Format and readability (multiline programming)

The formatter is **Ruff** (`ruff format`); the editor follows `.editorconfig`. Do not argue
style: run the formatter. In addition:

1. **Maximum 100 columns.**
2. **One statement per line.** No `a(); b()`, no `if x: y` on one line, no `;`.
3. **Multiline programming:** a call with more than two arguments, a literal with more than one
   element, and any chained call (`np`, OpenCV pipelines) are written one item per line with a
   trailing comma, so each change is one line in the diff.
4. **Maximum 3 positional parameters.** With more, use keyword-only arguments (`*,`) or a
   frozen `@dataclass`.
5. **Long conditions get a name.** A condition with more than two operands is extracted into a
   boolean variable prefixed with `is_`, `has_`, `can_` or `should_`.
6. **Guard clauses** instead of nested `if/else`. Maximum 3 nesting levels. No nested ternaries.
7. **Blank lines with intent:** one blank line between logical blocks, never two inside a
   function.
8. **Imports** grouped (standard library, third party, ROS and workspace packages, local), one
   blank line between groups, sorted by Ruff. No `from x import *`.
9. **Type hints** on every public function signature.

```python
# Bad: dense, magic numbers, unnamed condition.
if d < 0.17 and c == "red" and r.confidence > 0.5: events.append("near:red:" + str(i))

# Good: one idea per line, named condition, constants and enums.
is_dangerous_red = (
    cube.color == CubeColor.RED
    and distance_m < NEAR_RED_DISTANCE_M
    and robot.confidence > MIN_CONFIDENCE
)

if is_dangerous_red:
    events.append(
        format_event(
            SensorEvent.NEAR_RED,
            cube.id,
        )
    )
```

### 4.5 Modularity

**Every group folder splits pure logic from ROS:**

```text
<group>/
├── <group>_core/                  # pure Python: no rclpy, no hardware, no camera
│   ├── <group>_core/
│   │   ├── constants.py           # package constants and enums
│   │   ├── <feature>.py           # one responsibility per module
│   │   └── types.py               # dataclasses shared inside the package
│   └── test/test_<feature>.py
└── <group>_ros/                   # thin ROS adapters
    ├── <group>_ros/
    │   ├── ros_names.py           # topics, services, frame ids, QoS profiles
    │   ├── <name>_node.py         # one node per file
    │   └── conversions.py         # core types <-> ROS messages
    ├── config/<name>.yaml
    └── launch/<name>.launch.py
```

**Dependency rule:** `_ros → _core`, never the reverse. `_core` never imports `rclpy`, message
types or hardware drivers. Hardware access lives behind a small class in the HAL module so it
can be replaced by a fake in tests.

**Responsibilities:**

- A **node** declares parameters, wires publishers/subscribers/services and timers, converts
  messages and calls the core. It holds no algorithm.
- The **core** holds the algorithms: deterministic, testable with synthetic data.
- **Conversions** between core types and ROS messages live in `conversions.py`, not inside
  callbacks.

**Size:** one class per file when it is public; files up to ~300 lines; functions up to ~40
lines. If it grows, split it.

**Names:**

| Element | Style | Example |
|---|---|---|
| Packages, modules, files | `snake_case` | `overhead_core/triangle.py` |
| Classes, dataclasses, enums | `PascalCase` | `CubeTracker` |
| Functions, variables, parameters | `snake_case`, functions start with a verb | `detect_cubes` |
| Constants | `UPPER_SNAKE_CASE` with unit suffix | `TRACK_GRACE_S` |
| Booleans | `is_`, `has_`, `can_`, `should_` | `is_in_zone` |
| ROS topics and services | `snake_case`, absolute only for the shared contract | `/world/state` |
| ROS parameters | `snake_case` with unit suffix | `rate_hz`, `arena_width_m` |

**Forbidden:**

- `print` (use the node logger in `_ros`, `logging` in `_core`).
- Bare `except:` and `except Exception: pass`.
- Mutable default arguments.
- Global mutable state.
- `time.sleep` inside callbacks (use timers).
- Dead code, unused imports and orphan files.

### 4.6 ROS specifics

- Topic, service and frame names come from `ros_names.py`.
- QoS is a named profile (`SENSOR_QOS`, `STATE_QOS`) defined once, matching the contract table.
- Every parameter is declared with a default from `constants.py` and documented in the
  package README.
- Units are SI and REP 103: meters, radians, seconds; `theta` counterclockwise from `+x`,
  in `(-pi, pi]`.
- `interfaces/multirobot_interfaces/` is never edited (byte-identical with stage 1).
  `interfaces/tsa_interfaces/` changes only by a PR approved by one person of each group.

### 4.7 Tests

- Every core module has `test/test_<module>.py` covering the happy path and the expected
  failures.
- Test names: `test_should_<behavior>_when_<condition>`.
- Arrange – Act – Assert, separated by blank lines. No conditional logic in tests.
- Use synthetic images and fixed timestamps; tests never open a camera or a network.

### 4.8 Before you push

```bash
ruff format .
ruff check .
python -m pytest overhead/overhead_core
```

Review your diff: English only, docstrings on public code, no magic values, no line over 100
columns, no commented-out code.

---

## 5. Git conventions

### 5.1 Commits: Conventional Commits, in English

```text
<type>(<scope>): [TSA-<n> ]<imperative summary in lowercase, no final period>

[optional body: what and why, wrapped at 100 columns]
```

- **Types:** `feat`, `fix`, `refactor`, `perf`, `test`, `docs`, `style`, `build`, `ci`,
  `chore`, `revert`.
- **Scope:** the folder or package touched: `overhead`, `fleet`, `robot`, `manipulator`,
  `interfaces`, `sim`, `viewer`, `docs`, `ci`, `infra`. Use the package name for precision
  (`overhead_core`).
- **Issue key** (`TSA-<n>`) right after the colon when the commit belongs to a tracked task.
- Summary under 72 characters, imperative mood (`add`, `fix`, `remove`, not `added`).
- One logical change per commit. Split code, tests and docs into separate commits when they are
  reviewed separately.

Examples:

```text
feat(overhead_core): TSA-4 add triangle pose detector with tip heading
test(overhead_core): cover tracker occlusion grace period
fix(overhead_ros): publish world state with best effort QoS
docs(interfaces): describe virtual sensors events
chore(ci): run ruff and pytest on every pull request
```

### 5.2 Branches and pull requests

- `main` is the demo-ready branch and `develop` the integration branch. **No direct commits or
  pushes to either**; the hooks block them and CI flags any push that skips a PR.
- Branch names: `<type>/<scope>-<short-description>`, e.g. `feat/overhead-triangle-detector`,
  `fix/fleet-watchdog-timeout`.
- Every change enters through a pull request to `develop` using the PR template. Changes to
  `interfaces/` need an approval from each group (`.github/CODEOWNERS`).

### 5.3 Hooks

Enable them once per clone:

```bash
git config core.hooksPath .githooks
```

- `commit-msg` rejects messages that do not follow 5.1.
- `pre-commit` blocks commits on `main`/`develop` and runs `ruff format --check` and
  `ruff check` on staged Python files.
- `pre-push` blocks pushes to `main`/`develop`.
