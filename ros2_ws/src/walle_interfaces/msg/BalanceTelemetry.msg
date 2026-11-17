# BalanceTelemetry.msg
# Complete internal state of the ESP32 balance controller.
# Published at 50 Hz on /balance_telemetry topic.
# Used for: live dashboard, PlotJuggler PID tuning, rosbag recording.

# Timestamp and coordinate frame
std_msgs/Header header

# Current tilt angle in degrees (from complementary filter)
# Positive = leaning FORWARD, Negative = leaning BACKWARD
# Safety cutoff triggers at abs(pitch_angle) > 40.0 degrees
float32 pitch_angle

# How fast the pitch is changing in degrees per second (from gyroscope)
# Used by the Derivative term of the PID to predict and dampen overshoot
float32 pitch_rate

# The angle the robot is trying to achieve (degrees)
# IDLE mode:          0.0 (motors off)
# BALANCE_ONLY mode:  0.0 (stand upright) or small lean from cmd_vel
# BOTH mode:          small forward lean to move + line steering
float32 target_pitch

# Total PID output driving the motors (-100.0 to +100.0 percent duty)
# Positive = wheels forward (catching a forward lean)
# Negative = wheels backward (catching a backward lean)
float32 pid_output

# Proportional term: Kp times error
# Plot this to see how aggressively P is reacting to tilt
float32 pid_p_term

# Integral term: Ki times accumulated error over time
# Grows slowly; corrects steady-state drift in one direction
float32 pid_i_term

# Derivative term: Kd times rate of error change
# Damps fast oscillations; too high causes vibration from sensor noise
float32 pid_d_term

# Actual duty cycle sent to left motor (-100.0 to +100.0 percent)
# Formula: left = pid_output - steering_offset
float32 motor_left_duty

# Actual duty cycle sent to right motor (-100.0 to +100.0 percent)
# Formula: right = pid_output + steering_offset
float32 motor_right_duty

# Forward velocity commanded via cmd_vel topic (maps to a pitch lean)
float32 forward_velocity

# Differential steering correction from line PID or cmd_vel angular
# Positive = turn right, Negative = turn left
float32 steering_offset

# Current operating mode
# 0 = IDLE, 1 = BALANCE_ONLY, 2 = LINE_FOLLOW, 3 = BOTH
uint8 mode

# True if ESP32 is connected to Wi-Fi
bool wifi_connected

# True if micro-ROS agent is reachable and data is flowing
bool agent_connected
