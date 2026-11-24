# Tri-State-Area

Tri-State-Area is stage 2 of the UCAB Guayana Robotics course: a multi-robot autonomous collection and sorting system operating in a shared arena.

The system comprises two mobile collector robots (Phineas and Ferb), one sorting robotic arm (Perry), and one overhead camera 1 m above the arena.

## System Groups and Ownership

| Group | Folder | Responsibility |
| --- | --- | --- |
| **Overhead** | `overhead/` | Fixed camera 1 m above the arena. Performs camera calibration, cube and robot detection, world model tracking, virtual sensors computation, and on-screen virtual projection (`viewer_node`). Never issues commands. |
| **Fleet** | `fleet/` | Autonomous onboard software running on each robot's Raspberry Pi 5 (mission, peer-to-peer traffic negotiation, path control, HAL, internal watchdog, ground line sensor reflexes). |
| **Manipulator** | `manipulator/` | Perry sorting arm and delivery-zone arbiter. |

## Repository Layout

```text
Tri-State-Area/
├── README.md
├── AGENTS.md
├── CLAUDE.md
├── docs/
│   ├── architecture.md
│   ├── contract.md
│   ├── arena.md
│   ├── decisions.md
│   └── open-questions.md
├── interfaces/
│   ├── multirobot_interfaces/    [shared with stage 1, untouched]
│   └── tsa_interfaces/           [shared ROS 2 message and service definitions]
├── overhead/
│   ├── overhead_core/            [pure Python, no rclpy: vision, tracking, virtual sensors]
│   └── overhead_ros/             [thin ROS 2 nodes: overhead_node and viewer_node]
├── fleet/
│   └── README.md                 [owned by fleet group]
├── manipulator/
│   └── README.md                 [Perry arm and zone arbiter]
├── sim/
│   └── README.md                 [mock world simulator publishing overhead contract]
└── .github/
    ├── CODEOWNERS
    ├── pull_request_template.md
    └── workflows/
```

## Documentation

- [Architecture](docs/architecture.md): System layers, hybrid paradigm, control flow, and data pipelines.
- [Inter-Group Contract](docs/contract.md): ROS 2 topic, service, and QoS specifications.
- [Arena Specification](docs/arena.md): Physical dimensions, coordinates, homography, parallax correction, and markers.
- [Decisions Log](docs/decisions.md): Architecture and implementation decision history (D1–D12).
- [Open Questions](docs/open-questions.md): Pending questions and measurement tasks.

## Build and Run

### Environment

- **OS:** Ubuntu 26.04 (PC and Raspberry Pi 5)
- **ROS 2:** Lyrical
- **Domain ID:** `export ROS_DOMAIN_ID=0`

### Building the Workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

### Running Overhead Vision and Viewer

```bash
ros2 launch overhead_ros overhead.launch.py
```

To run without the OpenCV GUI window:

```bash
ros2 launch overhead_ros overhead.launch.py use_viewer:=false
```

### Running Unit Tests

Pure unit tests run without ROS, cameras, or hardware:

```bash
python -m pytest overhead/overhead_core
```

## Contributing

All contributors and AI agents must follow the code conventions and git workflow documented in [AGENTS.md](AGENTS.md).

Enable git hooks once after cloning:

```bash
git config core.hooksPath .githooks
```
