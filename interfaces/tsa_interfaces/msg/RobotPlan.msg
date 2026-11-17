# Fleet plan for a single robot in frame "arena" (units: meters, radians, theta CCW from +x in (-pi, pi])
string name                        # "phineas" | "ferb"
string state                       # IDLE | GO_TO_APPROACH | PUSH | REQUEST_ZONE | DELIVER | LEAVE | WAIT | STOPPED | RECOVER
uint32 target_cube_id              # 0 if no target cube
geometry_msgs/Pose2D target_pose   # meters and radians in frame "arena"
geometry_msgs/Point[] path         # waypoints in meters in frame "arena"
