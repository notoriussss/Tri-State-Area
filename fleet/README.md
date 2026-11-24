# `fleet`

Owned by the **fleet** group (Phineas and Ferb autonomous collector robots).

## Packages

- `fleet_core/`: Pure Python core algorithms and hardware abstractions (mecanum kinematics, Fusion HAT+ / GPIO motor drivers, TCRT5000 line sensor, local safety reflexes, world model, holonomic controller, navigation, mission, P2P traffic, and zone choreography). No `rclpy` dependency; tested with `pytest`.
- `fleet_ros/`: Thin ROS 2 Lyrical adapters, per-robot YAML configs (`phineas.yaml`, `ferb.yaml`), and launch files running onboard each Raspberry Pi 5.

## Running unit tests

```bash
python -m pytest fleet/fleet_core
```
