# Virtual sensor readings for a single robot in frame "arena" (units: meters)
string name              # "phineas" | "ferb"
uint32 nearest_red_id    # 0 if no red cube is visible
float64 nearest_red_dist # meters (-1.0 if no red cube is visible)
string[] events          # e.g. "entered:zone", "left:zone", "near:red:7"
