# Obstacle with safety circle in frame "arena" (units: meters, meters per second)
string kind    # "RED_CUBE" | "ROBOT" | "CUBE" | "WALL" | "ZONE"
uint32 id      # obstacle identifier (cube id for cubes)
float64 x      # meters, obstacle center in frame "arena"
float64 y      # meters, obstacle center in frame "arena"
float64 radius # meters, safety circle radius
float64 vx     # m/s, velocity along +x in frame "arena"
float64 vy     # m/s, velocity along +y in frame "arena"
