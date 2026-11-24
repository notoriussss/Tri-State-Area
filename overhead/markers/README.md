# Robot markers

Printable ArUco markers (`DICT_4X4_50`) for the robot roofs.

| File | Robot | Id |
| --- | --- | --- |
| `aruco_4x4_50_id0_phineas.png` | phineas | 0 |
| `aruco_4x4_50_id1_ferb.png` | ferb | 1 |

Print so the black square is about 8 cm wide (keep the white border), glue it flat on the roof
with the center over the robot's geometric center and the edge marked `FRONT` toward the scoop.
Then run the overhead node with `robot_detector:=aruco` (defaults to ids 0 and 1). If the marker
is rotated on the roof, set `aruco_heading_offset_rad` to the angle between its up edge and the
scoop direction.
