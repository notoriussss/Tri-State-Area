# Virtual sensors computed by overhead in frame "arena" (units: meters)
std_msgs/Header header      # frame_id = "arena"
float64 robot_distance      # meters between robot centers (-1.0 if < 2 robots)
string[] robots_in_zone     # robot names inside the delivery zone
RobotSensors[] per_robot
Obstacle[] obstacles
RobotObstacles[] per_robot_obstacles
