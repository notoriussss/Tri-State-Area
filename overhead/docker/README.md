# Overhead ROS 2 container

Image based on `ros:lyrical-ros-base` (Ubuntu 26.04). It is multi-arch, so the same files build
on the PC (amd64) and on the Raspberry Pi 5 (arm64).

## Build and run (from `overhead/docker`)

```bash
docker compose build
docker compose run --rm overhead colcon-build
docker compose run --rm overhead ros2 launch overhead_ros overhead.launch.py use_viewer:=false
```

The repository is bind-mounted at `/ws/src/tri-state-area`; build, install and log folders live in
named volumes, so rebuilds are incremental.

## Test without a camera

Run the node on a still photo instead of a camera:

```bash
docker compose run --rm overhead ros2 run overhead_ros overhead_node \
  --ros-args -p video_source:=/ws/src/tri-state-area/overhead/overhead_core/test/fixtures/arena_real_2026-11-24.jpg
```

## On the Raspberry Pi

1. Install Docker, clone the repository and run `docker compose build` in `overhead/docker`.
2. Uncomment the `devices` block of `compose.yaml` to expose `/dev/video0`.
3. Run the launch command above; the container uses the host network so other groups on the same
   Wi-Fi see the topics (`ROS_DOMAIN_ID=0`).

## Known quirks

- Docker Desktop on Windows has no camera passthrough, use `video_source` there.
- `rclpy` nodes must not name an attribute `_services`, `_clients` or `_publishers`: they clash
  with the base `Node` internals and break the executor wait set.
