# Cube observation in frame "arena" (units: meters, floor projection)
uint32 id          # stable while the cube remains visible
string color       # "blue" | "yellow" | "red"
float64 x          # meters, cube center on the floor in frame "arena"
float64 y          # meters, cube center on the floor in frame "arena"
float32 confidence # detection confidence in [0.0, 1.0]
