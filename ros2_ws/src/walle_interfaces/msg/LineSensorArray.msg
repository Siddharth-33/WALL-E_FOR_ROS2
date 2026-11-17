# LineSensorArray.msg
# Carries all data from the 5-channel IR Line Sensor Array (LSA).
# Published by ESP32 at 50 Hz on /line_sensors topic.

# Timestamp and coordinate frame of this reading
std_msgs/Header header

# Raw ADC readings from all 5 IR sensors (left=A0 to right=A4)
# Range: 0 to 4095 (12-bit ADC on ESP32)
# High value ~ 4095 means sensor sees BLACK (line detected)
# Low  value ~ 0    means sensor sees WHITE (no line)
float32[5] raw_adc

# Normalized values scaled to 0.0 to 1.0
# 0.0 = white surface, 1.0 = black surface (line)
# Uses per-sensor calibration: (raw - cal_min) / (cal_max - cal_min)
float32[5] normalized

# Weighted-average line position relative to robot center
# -1.0 = line is far LEFT  (only sensor 0 sees it)
#  0.0 = line is CENTERED  (sensor 2 sees it)
# +1.0 = line is far RIGHT (sensor 4 sees it)
# Used as the error signal for the line-following PID controller
float32 line_position

# True if a line is detected under the sensors
# False if all sensors see white (line lost)
# When False, the line PID should hold last steering or stop
bool line_detected
