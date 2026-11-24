# overhead_ros

Thin ROS 2 adapters for the overhead camera vision node (`overhead_node`) and rectified viewer (`viewer_node`).

## Nodes

### `overhead_node`

Captures frames from a live camera or a still image (`video_source`), calibrates the arena homography against the black-tape border, tracks cubes and robot markers via `overhead_core`, and publishes `/world/state` and `/world/sensors` only when the state machine is in `RUNNING` or `DEGRADED`.

#### Modules

- `overhead_node.py`: Thin ROS 2 adapter wiring publishers, subscribers, services, timers, and state-machine transitions.
- `node_parameters.py`: Declares all ROS parameters with defaults from `overhead_core.constants` and loads them into `OverheadNodeParameters`.
- `world_publishers.py`: Builds and publishes `WorldState`, `VirtualSensors`, and rectified `CompressedImage` messages.
- `safety_inputs.py`: Tracks `/zone/occupant` heartbeat freshness and `/perry/state` gate status on the local monotonic clock.
- `conversions.py`: Pure conversions between `overhead_core` types and `tsa_interfaces` messages.

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `camera_index` | `int` | `0` | OpenCV camera device index used when `video_source` is empty. |
| `video_source` | `string` | `""` | Path to a still image file for offline testing; empty string opens the live camera. |
| `width` | `int` | `1280` | Requested camera capture width in pixels. |
| `height` | `int` | `720` | Requested camera capture height in pixels. |
| `rate_hz` | `double` | `20.0` | Main vision processing and state-machine loop rate in Hz. |
| `calibration_retry_s` | `double` | `1.0` | Interval in seconds between arena corner calibration retries while `CALIBRATING`. |
| `arena_width_m` | `double` | `1.165` | Physical arena width along world `+x` in meters. |
| `arena_height_m` | `double` | `0.70` | Physical arena height along world `+y` in meters. |
| `cam_height_m` | `double` | `1.0` | Camera height above the arena floor in meters for parallax correction. |
| `camera_center_world` | `double[2]` | `[0.5825, 0.35]` | Floor coordinates directly below the camera in meters. |
| `zone_rect` | `double[4]` | `[1.165, 0.27, 1.30, 0.44]` | Delivery zone bounds `[xmin, ymin, xmax, ymax]` in meters. |
| `robots` | `string[]` | `["phineas", "ferb"]` | Expected robot names tracked by the marker detector and hold filter. |
| `image_rate_hz` | `double` | `5.0` | Publish rate in Hz for the rectified JPEG stream on `/world/image`. |
| `min_area_px` | `int` | `150` | Minimum contour area in pixels for cube detection. |
| `max_area_px` | `int` | `1500` | Maximum contour area in pixels for cube detection; larger blobs are rejected. |
| `near_red_m` | `double` | `0.17` | Distance threshold in meters for `near:red:<id>` virtual sensor events. |
| `robot_detector` | `string` | `"triangle"` | Robot marker detector backend (`"triangle"` or `"aruco"`). |
| `aruco_dictionary` | `string` | `"DICT_4X4_50"` | OpenCV ArUco dictionary name used when `robot_detector` is `"aruco"`. |
| `aruco_ids` | `int[]` | `[0, 1]` | ArUco marker IDs mapped positionally to the `robots` parameter. |
| `aruco_heading_offset_rad` | `double` | `0.0` | Angle offset in radians from the ArUco marker up direction to robot forward. |
| `hsv_blue` | `int[6]` | `[110, 80, 25, 135, 255, 255]` | Flat `[H_lo, S_lo, V_lo, H_hi, S_hi, V_hi]` HSV bounds for blue cubes. |
| `hsv_yellow` | `int[6]` | `[20, 100, 55, 35, 255, 255]` | Flat HSV bounds for yellow cubes. |
| `hsv_red_1` | `int[6]` | `[0, 90, 55, 10, 255, 255]` | Lower red cube HSV hue band. |
| `hsv_red_2` | `int[6]` | `[170, 90, 55, 180, 255, 255]` | Upper red cube HSV hue band. |
| `hsv_robot_phineas_1` | `int[6]` | `[0, 100, 100, 4, 255, 255]` | Lower red HSV band for Phineas's provisional triangle marker. |
| `hsv_robot_phineas_2` | `int[6]` | `[170, 100, 100, 180, 255, 255]` | Upper red HSV band for Phineas's provisional triangle marker. |
| `hsv_robot_ferb` | `int[6]` | `[6, 150, 100, 14, 255, 255]` | Orange HSV bounds for Ferb's provisional triangle marker. |
| `robot_heading_offset_rad_phineas` | `double` | `0.0` | Heading offset in radians added to Phineas's triangle heading and wrapped to `(-pi, pi]`. |
| `robot_heading_offset_rad_ferb` | `double` | `0.0` | Heading offset in radians added to Ferb's triangle heading and wrapped to `(-pi, pi]`. |
| `robot_position_offset_m_phineas` | `double[2]` | `[0.0, 0.0]` | Phineas robot-frame position offset `[forward_m, left_m]` applied after heading offset. |
| `robot_position_offset_m_ferb` | `double[2]` | `[0.0, 0.0]` | Ferb robot-frame position offset `[forward_m, left_m]` applied after heading offset. |

#### Topics and Services

- **Publishes:** `/world/state` (`tsa_interfaces/WorldState`), `/world/sensors` (`tsa_interfaces/VirtualSensors`), `/world/image` (`sensor_msgs/CompressedImage`)
- **Services:** `/world/get` (`tsa_interfaces/GetWorld`), `/overhead/recalibrate` (`std_srvs/Trigger`)

### `manipulator_relay_node`

Relays delivery-zone requests, releases, and `/zone/occupant` heartbeats between the collector robots and Perry using the pure routing rules in `overhead_core.relay`. Stops republishing `/<robot>/zone/occupant` when Perry's heartbeat ages past `heartbeat_timeout_s` on the local monotonic clock. Launched alongside `overhead_node` when `use_relay:=true`.

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `robots` | `string[]` | `["phineas", "ferb"]` | Configured robot namespaces served and republished by the relay. |
| `perry_call_timeout_s` | `double` | `5.0` | Timeout in seconds when forwarding zone service calls to Perry. |
| `heartbeat_timeout_s` | `double` | `2.0` | Maximum local age in seconds before Perry's `/zone/occupant` heartbeat is treated as stale. |
| `republish_rate_hz` | `double` | `2.0` | Timer rate in Hz for republishing fresh `/<robot>/zone/occupant` heartbeats. |

#### Topics and Services

- **Subscribes:** `/zone/occupant` (`std_msgs/String`)
- **Publishes:** `/<robot>/zone/occupant` (`std_msgs/String`)
- **Services:** `/<robot>/zone/request` (`multirobot_interfaces/RequestZone`), `/<robot>/zone/release` (`multirobot_interfaces/ReleaseZone`)
- **Clients:** `/zone/request` (`multirobot_interfaces/RequestZone`), `/zone/release` (`multirobot_interfaces/ReleaseZone`)

## Tests

The ROS 2 integration tests in `overhead/overhead_ros/test/` run in-process against still-photo fixtures and a `FakePerry` node using a PID-isolated `ROS_DOMAIN_ID`, so no physical camera or Perry hardware is required.

From `overhead/docker`, build and run all `overhead_core` and `overhead_ros` tests in the ROS 2 Lyrical container:

```bash
docker compose run --rm overhead run-tests
```

Inside an already-sourced ROS 2 Lyrical workspace (`/ws/install/setup.bash`), run only the ROS integration tests with:

```bash
python3 -m pytest overhead/overhead_ros/test -q -p no:cacheprovider
```
