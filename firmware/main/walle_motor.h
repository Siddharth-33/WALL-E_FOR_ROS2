/*
 * motor_driver.h
 * ==============
 * Clean interface for the TB6612FNG dual motor driver on SRA board.
 *
 * HARDWARE: TB6612FNG Motor Driver IC
 * ------------------------------------
 * The TB6612FNG is a dual H-bridge motor driver chip.
 * An H-bridge lets you control:
 *   - Speed: via PWM (Pulse Width Modulation) duty cycle 0-100%
 *   - Direction: forward or backward
 *
 * WHAT IS PWM?
 *   PWM = rapidly switching the voltage ON and OFF.
 *   A 50% duty cycle means the motor gets power half the time → runs at half speed.
 *   A 100% duty cycle means full power → full speed.
 *   The switching frequency is 20kHz (inaudible to humans).
 *
 * HOW H-BRIDGE CONTROLS DIRECTION:
 *   Two control pins (IN_1, IN_2) per motor:
 *   IN_1=HIGH, IN_2=LOW  → Motor spins FORWARD
 *   IN_1=LOW,  IN_2=HIGH → Motor spins BACKWARD
 *   IN_1=LOW,  IN_2=LOW  → Motor STOPS (coast)
 *   IN_1=HIGH, IN_2=HIGH → Motor BRAKES (short circuit, immediate stop)
 *
 * SRA BOARD PIN ASSIGNMENTS (from pin_defs.h):
 *   Motor A (LEFT):
 *     IN_1 = GPIO13 (MDA_NORMAL_IN_1) → MCPWM Unit 1, Channel 0A
 *     IN_2 = GPIO12 (MDA_NORMAL_IN_2) → MCPWM Unit 1, Channel 0B
 *   Motor B (RIGHT):
 *     IN_3 = GPIO26 (MDA_NORMAL_IN_3) → MCPWM Unit 1, Channel 1A
 *     IN_4 = GPIO25 (MDA_NORMAL_IN_4) → MCPWM Unit 1, Channel 1B
 *
 * NOTE ON MOTOR DIRECTION:
 *   "Forward" in firmware doesn't always mean physical forward for the robot.
 *   The left and right motors may be mounted in opposite orientations.
 *   You'll discover this during testing and fix it by swapping IN_1/IN_2 OR
 *   by negating the duty cycle for one motor. We handle this with MOTOR_LEFT_INVERT.
 */

#pragma once

#include "esp_err.h"

/*
 * motor_init()
 * -------------
 * Initializes both motor driver channels using the SRA board component.
 * Sets up MCPWM (Motor Control PWM) hardware at 20kHz frequency.
 *
 * MCPWM = Motor Control PWM — a dedicated ESP32 peripheral for motor control.
 * Unlike regular GPIO, MCPWM generates PWM signals in hardware,
 * so the CPU doesn't need to bit-bang the signal. This is critical
 * at our control loop speed (200 Hz updates × many PWM pulses each cycle).
 */
esp_err_t motor_init(void);

/*
 * set_motor_duties()
 * ------------------
 * Set the duty cycle for both motors simultaneously.
 *
 * Parameters:
 *   left_duty:  -100.0 to +100.0 (negative = backward, positive = forward)
 *   right_duty: -100.0 to +100.0
 *
 * This function:
 *   1. Applies deadband compensation (motors don't move below ~20% duty)
 *   2. Clamps values to [-100, +100]
 *   3. Sets direction and speed via SRA board motor driver
 *
 * WHY SIGNED DUTY (-100 to +100)?
 *   The PID output can be positive or negative.
 *   Positive means both motors forward (catching forward lean).
 *   Negative means both motors backward (catching backward lean).
 *   Having a signed duty simplifies the balance controller math.
 */
void set_motor_duties(float left_duty, float right_duty);

/*
 * stop_motors()
 * -------------
 * Immediately stops both motors. Used when:
 *   - Robot enters IDLE mode
 *   - Pitch angle exceeds safety threshold (robot has fallen)
 *   - watchdog_task detects a control loop freeze
 */
void stop_motors(void);
