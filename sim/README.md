# `sim`

Lightweight 2D arena simulator implementing the Tri-State-Area ROS 2 contract without Webots or a physical camera.

## Packages

- `sim_core/`: Pure Python 2D holonomic physics, push-contact model, border tape check, virtual sensors, and `MockPerryArbiter`. Tested closed-loop with two autonomous `fleet_core` agents (Phineas and Ferb).
- `sim_ros/`: Thin ROS 2 node (`sim_node`) publishing `/world/state`, `/world/sensors`, `/zone/occupant`, `/perry/state` and serving `/world/get`, `/zone/request`, and `/zone/release`.

## Running closed-loop tests

```bash
python -m pytest sim/sim_core -v
```
