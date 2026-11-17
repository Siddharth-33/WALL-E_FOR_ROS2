/**
 * @file balance_controller.h
 * @brief PID-based balance and line-following controller for Wall-E robot.
 *
 * This header declares the data structures and functions used to implement
 * two independent PID (Proportional-Integral-Derivative) controllers:
 *   1. Balance PID  – keeps the robot upright by commanding motor torque
 *      proportional to the pitch angle error from vertical.
 *   2. Line PID     – steers the robot to follow a dark line on the floor
 *      by computing an error signal from the line-sensor array.
 *
 * WHAT IS A PID CONTROLLER?
 * --------------------------
 * A PID controller is a feedback control algorithm.  It continuously
 * calculates an "error" (the difference between a desired setpoint and the
 * current measured value) and uses three terms to correct it:
 *
 *   P (Proportional) – Output is directly proportional to the error.
 *       A large error → large correction. Fast response, but can overshoot.
 *
 *   I (Integral)     – Output is proportional to the accumulated (integrated)
 *       error over time. This removes steady-state offset (the robot leaning
 *       slightly even when P says it should be upright), but can cause
 *       "integral windup" if the error accumulates too fast, so we clamp it.
 *
 *   D (Derivative)   – Output is proportional to the *rate of change* of
 *       the error (its derivative w.r.t. time). This dampens oscillations
 *       and makes the system react ahead of the trend. Sensitive to noise.
 *
 * Combined output: u(t) = Kp*e(t) + Ki*∫e dt + Kd*(de/dt)
 *
 * HOW IT APPLIES HERE:
 * --------------------
 * Balance PID:
 *   error   = setpoint_pitch − measured_pitch   (ideally setpoint is 0°)
 *   output  = duty cycle sent to both motors (positive → forward, negative → back)
 *
 * Line PID:
 *   error   = weighted_centroid of the line sensor readings − center_of_sensors
 *   output  = differential correction added/subtracted to left/right motor duty
 */

#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <stdint.h>

/* ─────────────────────────────────────────────────────────────────────────────
 * PID CONTROLLER STRUCTURE
 * ─────────────────────────────────────────────────────────────────────────────
 * Each PID instance is self-contained: it holds the three tuning gains
 * (kp, ki, kd) AND the running state (integral accumulator, previous error).
 * This allows us to have two independent PIDs without global state conflicts.
 */
typedef struct {
    float kp;           /**< Proportional gain: scales immediate error response  */
    float ki;           /**< Integral gain: scales accumulated error correction   */
    float kd;           /**< Derivative gain: scales rate-of-change damping       */

    float integral;     /**< Running sum of error × dt (the ∫e dt term)           */
    float prev_error;   /**< Error from the previous iteration (needed for de/dt) */

    float integral_max; /**< Anti-windup clamp: |integral| is capped at this value */
    float setpoint;     /**< Desired target value (e.g., 0.0° for balance)         */
} pid_controller_t;

/* ─────────────────────────────────────────────────────────────────────────────
 * MOTOR OUTPUT STRUCTURE
 * ─────────────────────────────────────────────────────────────────────────────
 * Both PIDs together produce the final left/right motor duty cycles.
 * Duty cycle range is 0–100 (percent), direction is separate.
 */
typedef struct {
    float left_duty;    /**< Left  motor duty cycle (0.0 – 100.0)  */
    float right_duty;   /**< Right motor duty cycle (0.0 – 100.0)  */
    int   left_dir;     /**< +1 = forward, -1 = backward, 0 = stop */
    int   right_dir;    /**< +1 = forward, -1 = backward, 0 = stop */
} motor_output_t;

/* ─────────────────────────────────────────────────────────────────────────────
 * STARTING PID GAINS
 * ─────────────────────────────────────────────────────────────────────────────
 * These are empirically chosen starting values.  They will almost certainly
 * need tuning on the real robot.  The micro-ROS /set_pid_gains service lets
 * you update them at runtime without reflashing firmware.
 *
 * Balance gains (tuned for SRA board TT motors, 6×8×10 cm chassis):
 *
 *   HOW TO TUNE (Ziegler-Nichols approach):
 *   1. Start with Ki=0, Kd=0.  Raise Kp until robot oscillates steadily.
 *      That Kp is the "ultimate gain" Ku.  Note the oscillation period Tu.
 *   2. Set Kp = 0.6×Ku, Ki=0, Kd=0.125×Ku×Tu  as a starting point.
 *   3. Fine-tune on the robot.
 *
 *   These defaults are conservative starting values matched to the SRA
 *   reference implementation (see Wall-E_reference/6_self_balancing):
 *   Kp = 25.0 – moderate proportional response (less likely to saturate)
 *   Ki =  0.2 – small integral to correct slow offset drift
 *   Kd =  1.5 – light derivative for damping
 *
 *   IMPORTANT: You MUST also tune BALANCE_PITCH_OFFSET below to match
 *   your robot's physical centre of gravity.  Run with Ki=0, Kd=0 first,
 *   find the offset where the robot stays still, then add Ki and Kd.
 *
 * Line-following gains:
 *   Kp = 30.0 – strong correction toward line center
 *   Ki =  0.0 – no integral (line error rarely has steady-state offset)
 *   Kd =  5.0 – heavy derivative to prevent overshooting the line
 */
#define BALANCE_KP_DEFAULT    40.0f
#define BALANCE_KI_DEFAULT     0.0f
#define BALANCE_KD_DEFAULT     1.5f

#define LINE_KP_DEFAULT        1.6f
#define LINE_KI_DEFAULT        0.0f
#define LINE_KD_DEFAULT        4.5f

/*
 * LINE FOLLOWING SPEED & DUTY CYCLE LIMITS (Mode 2):
 * ──────────────────────────────────────────────────
 * LINE_OPTIMUM_DUTY_CYCLE : Base cruising speed when centered on the line (default: 57%)
 * LINE_LOWER_DUTY_CYCLE   : Minimum motor speed limit during sharp turns (default: 45%)
 * LINE_HIGHER_DUTY_CYCLE  : Maximum motor speed limit during sharp turns (default: 65%)
 */
#define LINE_OPTIMUM_DUTY_CYCLE        43.0f
#define LINE_LOWER_DUTY_CYCLE          33.0f
#define LINE_HIGHER_DUTY_CYCLE         52.0f

/*
 * PITCH OFFSET (setpoint trim):
 * ─────────────────────────────
 * Default physical upright angle of the chassis.
 * SRA board reference uses setpoint ~9.0° to 10.0°.
 * Telemetry confirms resting pitch is around ~10° to 13°.
 */
#define BALANCE_PITCH_OFFSET_DEFAULT   -31.0f

/* Safety threshold: if |pitch| exceeds this many degrees from vertical, stop immediately */
#define PITCH_SAFETY_LIMIT    50.0f

/*
 * MOTOR DUTY LIMITS:
 * ──────────────────
 * MOTOR_DEADBAND: Minimum duty to overcome gearbox friction (18%)
 * BALANCE_MAX_DUTY: Maximum duty clamp (50%) to prevent dangerous runaway speeds.
 */
#define MOTOR_DEADBAND        18.0f
#define BALANCE_MAX_DUTY      60.0f

/* Deadzone: if |pitch_error| is below this, robot is balanced — coast */
#define BALANCE_DEADZONE_DEG   0.8f

/* Maximum allowed integral accumulation (anti-windup) */
#define INTEGRAL_MAX_BALANCE 100.0f
#define INTEGRAL_MAX_LINE     50.0f

/* ─────────────────────────────────────────────────────────────────────────────
 * FUNCTION PROTOTYPES
 * ─────────────────────────────────────────────────────────────────────────────
 */

void balance_controller_init(void);

void balance_controller_update(float pitch_deg,
                               float line_error,
                               int   do_line,
                               float dt_s,
                               motor_output_t *out);

void balance_controller_line_update(float line_error,
                                    float dt_s,
                                    motor_output_t *out);

void set_balance_pid_gains(float kp, float ki, float kd);
void set_line_pid_gains(float kp, float ki, float kd);

void get_balance_pid_gains(float *kp, float *ki, float *kd);
void get_line_pid_gains(float *kp, float *ki, float *kd);

void set_balance_pitch_offset(float offset);
float get_balance_pitch_offset(void);

#endif /* BALANCE_CONTROLLER_H */
