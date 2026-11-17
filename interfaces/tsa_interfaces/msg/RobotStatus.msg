# Onboard robot status published by each robot
bool line_detected
float32 battery_v        # volts
uint32 cmd_age_ms        # milliseconds since the last received velocity command
string mode              # "run" | "stopped_watchdog" | "stopped_line" | "manual"
