# Obstacle radar and collision warning for a single robot (units: meters, radians, seconds)
string name               # "phineas" | "ferb"
float64[8] sectors        # meters, free distance per 45-degree sector CCW from scoop (sector 0 centered on scoop)
string level              # "CLEAR" | "SLOW" | "STOP"
float64 time_to_collision # seconds (-1.0 if none)
string nearest_kind       # kind of nearest obstacle ("" if none)
float64 nearest_dist      # meters, distance to nearest obstacle (-1.0 if none)
float64 nearest_bearing   # radians, bearing to nearest obstacle relative to the scoop
