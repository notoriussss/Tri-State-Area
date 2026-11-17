# `tsa_interfaces`

Shared ROS 2 message and service definitions for **Tri-State-Area** (contract v0, see [`docs/contract.md`](../../docs/contract.md)).

> **Rule:** Any change to this package requires a Pull Request approved by at least one member of each group (`overhead`, `fleet`, `manipulator`).

## Contract Summary

All spatial coordinates use meters and radians in frame `"arena"`, with heading `theta` measured counterclockwise from `+x` in `(-pi, pi]`.

| Topic / Service | Type | Publisher / Server | Rate | QoS |
| --- | --- | --- | --- | --- |
| `/world/state` | `tsa_interfaces/msg/WorldState` | `overhead` | 20 Hz (min. 15 Hz) | best effort, keep last 1 |
| `/world/sensors` | `tsa_interfaces/msg/VirtualSensors` | `overhead` | 20 Hz (min. 15 Hz) | best effort, keep last 1 |
| `/world/get` | `tsa_interfaces/srv/GetWorld` | `overhead` | On demand | default |
| `/world/image` | `sensor_msgs/msg/CompressedImage` | `overhead` | 5 Hz | best effort, keep last 1 |
| `/phineas/cmd`, `/ferb/cmd` | `geometry_msgs/msg/Twist` | `fleet` | ~20 Hz | best effort, keep last 1 |
| `/fleet/plan` | `tsa_interfaces/msg/FleetPlan` | `fleet` | ~5 Hz | reliable, keep last 1 |
| `/phineas/status`, `/ferb/status` | `tsa_interfaces/msg/RobotStatus` | `robot` (`fleet` onboard) | 5 Hz | reliable, keep last 1 |
| `/zone/request` | `multirobot_interfaces/srv/RequestZone` | `manipulator` (Perry) | On demand | default |
| `/zone/release` | `multirobot_interfaces/srv/ReleaseZone` | `manipulator` (Perry) | On demand | default |
| `/zone/occupant` | `std_msgs/msg/String` | `manipulator` (Perry) | 2 Hz | default |
| `/perry/state` | `multirobot_interfaces/msg/RobotState` | `manipulator` (Perry) | — | default |
