# The Arena

Each measurement lists its source and confidence level.

## Measurements

| Item | Value | Source | Confidence |
| --- | --- | --- | --- |
| Arena (inside the black tape border) | 116.5 × 70 cm | Phineas and Ferb reports | current, nominal |
| Arena center | painted red dot | confirmed by Ignacio, 17-11 | high |
| Camera height above the arena | 100 cm exact | confirmed by Ignacio, 17-11 | high |
| Capture resolution | 1280 × 720 px | overhead capture | pending confirmation as working resolution |
| Image scale | ~9.73 px/cm (~1.03 mm/px) | visually estimated from capture | ±1 cm |
| Cube | 2.5 cm (~24 px in image) | Ferb report and capture | high |
| Robot | 18.6 × 16.1 × 9.1 cm (chassis) | Phineas and Ferb reports | current, nominal |
| Wheels | 65 mm mecanum, 4 × 1:120 TT motors, no encoders | reports | high |

## Layout (from Capture)

- The **delivery zone** is the blue notch on the right border, where the black tape is interrupted. A robot delivers a cube by **pushing it past the green line**.
- The **yellow box** is at the top right and the **green box** is at the bottom right, outside the arena.
- **Perry** sits outside the camera frame to the right. The camera does cover the delivery zone.
- Along the delivery zone opening there is no black tape, so the onboard line sensor does not protect the robot there. That opening is monitored by the virtual delivery-zone region.

## Origin and Axes

- The origin is at the **inner bottom-left** corner of the overhead image.
- `x` points toward Perry (to the right in the image) and `y` points upward in the image.
- `theta` is measured counterclockwise from `+x`.
- In the sample capture, the origin is at ~(30, 695) px and the opposite corner is at ~(1165, 15) px.

## Pixel to World Transform

Because the arena floor is flat, a 3 × 3 homography `H` computed from the four inner corners of the black tape maps any pixel to meters on the floor:

```text
[x, y, 1]^T  ∝  H · [u, v, 1]^T
```

If the camera moves, recalibrating only requires re-detecting the four corners.

## Parallax (Important with the Camera at 1 m)

The floor homography is exact only for points at floor height (`z = 0`). The robot marker sits on top of the robot roof at ~9 cm height, so the camera sees it radially displaced outward from the nadir point `c` directly beneath the lens:

```text
p_real = c + (p_seen - c) · (H - h) / H         H = 1.00 m, h ≈ 0.09 m
error  ≈ r · h / (H - h) ≈ 0.1 · r              (r = distance from c)
```

| Distance from `c` | Uncorrected Error |
| --- | --- |
| 20 cm | ~2 cm |
| 68 cm (corner) | ~6.7 cm |

`c` is the painted red center dot if the camera is mounted directly above it. Cubes (2.5 cm tall) also experience slight parallax (~0.9 cm in the corners) and are corrected using `h = 0.0125 m` (half their height).

## Robot Marker

A cardstock triangle is mounted on the roof of each robot, following the method of Castillejo (2020):

- **Position:** the centroid of the triangle gives `(x, y)`.
- **Orientation:** the tip—the vertex opposite the shortest side—gives `theta`. An elongated isosceles triangle is therefore preferred over an equilateral one.
- **Colors:** each robot needs a distinct color that does not clash with blue, yellow, or red (the cubes) or green (the delivery line).
- **Mounting:** the triangle's centroid must sit over the geometric center of the robot, with the tip pointing toward the front scoop.
