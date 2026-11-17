# Robot observation in frame "arena" (units: meters, radians; parallax-corrected floor projection)
string name        # "phineas" | "ferb"
float64 x          # meters, robot center on the floor in frame "arena"
float64 y          # meters, robot center on the floor in frame "arena"
float64 theta      # radians, scoop heading, CCW from +x in (-pi, pi]
float32 confidence # detection confidence in [0.0, 1.0]
