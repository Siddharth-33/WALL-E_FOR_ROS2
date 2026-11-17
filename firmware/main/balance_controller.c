/**
 * @file balance_controller.c
 * @brief Implementation of dual PID control for balancing and line-following.
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * PID MATHEMATICS – EXHAUSTIVE EXPLANATION
 * ═══════════════════════════════════════════════════════════════════════════════
 *
 * The continuous-time PID formula is:
 *
 *   u(t) = Kp·e(t)  +  Ki·∫₀ᵗ e(τ)dτ  +  Kd·(de/dt)
 *
 * where:
 *   u(t) = controller output (motor duty cycle in our case)
 *   e(t) = error = setpoint − measurement
 *   Kp   = proportional gain (tuning parameter)
 *   Ki   = integral gain     (tuning parameter)
 *   Kd   = derivative gain   (tuning parameter)
 *
 * Because we run on a digital MCU at discrete time steps (dt = 5 ms), we
 * approximate the continuous formulas:
 *
 *   INTEGRAL:     ∫e dt  ≈  integral_accumulator += e[n] * dt
 *                 (Euler forward method – simple rectangle approximation)
 *
 *   DERIVATIVE:   de/dt  ≈  (e[n] - e[n-1]) / dt
 *                 (backward difference approximation)
 *
 * So each call to pid_update() computes:
 *
 *   p_term = Kp * error
 *   i_term = Ki * (integral += error * dt)     [clamped to ±integral_max]
 *   d_term = Kd * (error - prev_error) / dt
 *   output = p_term + i_term + d_term
 *   prev_error = error                         [save for next call]
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * ANTI-WINDUP (why we clamp the integral):
 * ─────────────────────────────────────────────────────────────────────────────
 * If the robot falls over and the motor can't correct fast enough, the error
 * grows large for many cycles. Without clamping, the integral term would grow
 * without bound ("wind up"), making the motor jump to full speed the instant
 * the robot stands upright again, causing an overshoot/crash.
 *
 * We clamp:  integral = clamp(integral, -integral_max, +integral_max)
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * BALANCE CONTROL ARCHITECTURE:
 * ─────────────────────────────────────────────────────────────────────────────
 * The balance PID produces a single scalar "base_duty":
 *   base_duty > 0  → robot leans forward → drive FORWARD to catch it
 *   base_duty < 0  → robot leans backward → drive BACKWARD to catch it
 *
 * This duty is applied EQUALLY to both motors, keeping the robot going
 * straight while balancing.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * LINE-FOLLOW CONTROL ARCHITECTURE:
 * ─────────────────────────────────────────────────────────────────────────────
 * The line sensor array has 5 sensors (indices 0–4, left to right).
 * We compute a weighted centroid:
 *
 *   raw_error = Σ(weight[i] * sensor[i]) / Σ(sensor[i])
 *
 * where weight[] = {-2, -1, 0, +1, +2}
 * A negative error means the line is left of center → steer left.
 * A positive error means the line is right of center → steer right.
 *
 * The line PID produces a "correction" value:
 *   left_duty  = base_duty + correction   (speed up left to steer right)
 *   right_duty = base_duty - correction   (slow down right to steer right)
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * DEADBAND (why we enforce a minimum duty):
 * ─────────────────────────────────────────────────────────────────────────────
 * DC motors have static friction ("stiction"). If the commanded duty is very
 * small (< ~15–20%), the motor doesn't spin at all. This creates a dead zone
 * in the control response. To avoid wasted PID effort, if the computed duty
 * is non-zero but below MOTOR_DEADBAND, we snap it up to MOTOR_DEADBAND.
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 */

#include "balance_controller.h"

#include <math.h>    /* fabsf() – absolute value for floats */
#include <string.h>  /* memset()                            */

#include "esp_log.h" /* ESP_LOGI, ESP_LOGW – ESP-IDF logging macros */

/* Tag used by ESP_LOG macros; appears in serial monitor output */
static const char *TAG = "BALANCE_CTRL";

/* ─────────────────────────────────────────────────────────────────────────────
 * MODULE-PRIVATE STATE
 * ─────────────────────────────────────────────────────────────────────────────
 * Two PID instances – static so they are not accessible outside this file.
 * Keeping them file-scoped (rather than global) prevents accidental
 * modification from other modules and makes the data flow explicit through
 * the provided getter/setter functions.
 */
static pid_controller_t s_balance_pid; /* Balance (pitch) PID controller  */
static pid_controller_t s_line_pid;    /* Line-following PID controller    */
static float s_balance_pitch_offset = BALANCE_PITCH_OFFSET_DEFAULT; /* Balance point trim */

/* ─────────────────────────────────────────────────────────────────────────────
 * PRIVATE HELPER: pid_update()
 * ─────────────────────────────────────────────────────────────────────────────
 * Computes one iteration of the PID algorithm and returns the output.
 * This is an internal function (not exposed in the header) because callers
 * should use balance_controller_update() which handles safety + deadband.
 *
 * @param pid     Pointer to the PID instance to update.
 * @param error   Current error = setpoint − measurement.
 * @param dt_s    Time step in seconds since the last call.
 * @return        PID output (can be positive or negative).
 */
static float pid_update(pid_controller_t *pid, float error, float dt_s)
{
    /* ── PROPORTIONAL TERM ────────────────────────────────────────────────── */
    /* Immediate reaction to current error.                                    */
    /* Large Kp → aggressive correction but oscillation risk.                 */
    float p_term = pid->kp * error;

    /* ── INTEGRAL TERM ───────────────────────────────────────────────────── */
    /* Accumulate error × time.  Uses Euler forward integration:              */
    /*   integral_new = integral_old + error * dt                             */
    pid->integral += error * dt_s;

    /* Anti-windup: clamp the accumulator to ±integral_max.                  */
    /* Without this, integral could grow to millions after a fall.            */
    if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;

    float i_term = pid->ki * pid->integral;

    /* ── DERIVATIVE TERM ─────────────────────────────────────────────────── */
    /* Backward difference: (e[n] - e[n-1]) / dt                             */
    /* Dividing by dt normalises for varying loop rates.                      */
    /* Note: this is "derivative on error", which means a sudden setpoint     */
    /* change causes a derivative spike ("derivative kick").  For our use     */
    /* (setpoint is always 0°) this is not a problem.                        */
    float d_term = 0.0f;
    if (dt_s > 0.0f) {
        d_term = pid->kd * (error - pid->prev_error) / dt_s;
    }

    /* Save error for next iteration's derivative calculation */
    pid->prev_error = error;

    /* ── COMBINED OUTPUT ─────────────────────────────────────────────────── */
    float output = p_term + i_term + d_term;

    return output;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * PRIVATE HELPER: clamp()
 * ─────────────────────────────────────────────────────────────────────────────
 * Clamp a float value to [min_val, max_val].
 * Motor duty cycle must stay within 0–100%.
 */
static float clamp(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * PUBLIC: balance_controller_init()
 * ─────────────────────────────────────────────────────────────────────────────
 */
void balance_controller_init(void)
{
    /* Zero-initialise both PID structs (clears integral, prev_error etc.) */
    memset(&s_balance_pid, 0, sizeof(pid_controller_t));
    memset(&s_line_pid,    0, sizeof(pid_controller_t));

    /* ── Balance PID configuration ──────────────────────────────────────── */
    s_balance_pid.kp           = BALANCE_KP_DEFAULT;
    s_balance_pid.ki           = BALANCE_KI_DEFAULT;
    s_balance_pid.kd           = BALANCE_KD_DEFAULT;
    s_balance_pid.integral_max = INTEGRAL_MAX_BALANCE;
    s_balance_pid.setpoint     = 0.0f; /* Trim via BALANCE_PITCH_OFFSET in .h */

    /* ── Line PID configuration ─────────────────────────────────────────── */
    s_line_pid.kp           = LINE_KP_DEFAULT;
    s_line_pid.ki           = LINE_KI_DEFAULT;
    s_line_pid.kd           = LINE_KD_DEFAULT;
    s_line_pid.integral_max = INTEGRAL_MAX_LINE;
    s_line_pid.setpoint     = 0.0f; /* Line centered = 0 error */

    ESP_LOGI(TAG, "Balance PID init: Kp=%.1f Ki=%.2f Kd=%.2f",
             s_balance_pid.kp, s_balance_pid.ki, s_balance_pid.kd);
    ESP_LOGI(TAG, "Line PID init:    Kp=%.1f Ki=%.2f Kd=%.2f",
             s_line_pid.kp, s_line_pid.ki, s_line_pid.kd);
}

/* ─────────────────────────────────────────────────────────────────────────────
 * PUBLIC: balance_controller_update()
 * ─────────────────────────────────────────────────────────────────────────────
 */
void balance_controller_update(float pitch_deg,
                               float line_error,
                               int   do_line,
                               float dt_s,
                               motor_output_t *out)
{
    /* ── SAFETY CHECK ────────────────────────────────────────────────────── */
    /* Check deviation from balance point. If |tilt| > 45°, stop immediately */
    float tilt_from_balance = pitch_deg - s_balance_pitch_offset;
    if (fabsf(tilt_from_balance) > PITCH_SAFETY_LIMIT) {
        /* Reset integrators so we don't start with accumulated error when righted */
        s_balance_pid.integral  = 0.0f;
        s_balance_pid.prev_error = 0.0f;
        s_line_pid.integral     = 0.0f;
        s_line_pid.prev_error    = 0.0f;

        out->left_duty  = 0.0f;
        out->right_duty = 0.0f;
        out->left_dir   = 0;
        out->right_dir  = 0;
        return;
    }

    /* ── BALANCE PID ─────────────────────────────────────────────────────── */
    /*
     * Inverted pendulum error:
     *   balance_error = pitch_deg - (setpoint + offset)
     *   Forward lean (pitch > setpoint)  --> error > 0 --> drive FORWARD
     *   Backward lean (pitch < setpoint) --> error < 0 --> drive BACKWARD
     */
    float effective_setpoint = s_balance_pid.setpoint + s_balance_pitch_offset;
    float balance_error      = pitch_deg - effective_setpoint;

    /* ── DEADZONE CHECK ──────────────────────────────────────────────────── */
    if (fabsf(balance_error) < BALANCE_DEADZONE_DEG) {
        s_balance_pid.integral  = 0.0f;
        s_balance_pid.prev_error = 0.0f;
        s_line_pid.integral     = 0.0f;
        s_line_pid.prev_error    = 0.0f;
        out->left_duty  = 0.0f;
        out->right_duty = 0.0f;
        out->left_dir   = 0;
        out->right_dir  = 0;
        return;
    }

    float base_pid_out = pid_update(&s_balance_pid, balance_error, dt_s);

    /* ── GENTLE DEADBAND & SAFE SPEED CLAMP ──────────────────────────────── */
    /*
     * 1. Start gently from MOTOR_DEADBAND (18%) to overcome gearbox stiction.
     * 2. Limit maximum speed to BALANCE_MAX_DUTY (50%) to prevent dangerous runaway.
     */
    float base_duty = base_pid_out;
    if (fabsf(base_duty) > 0.0f) {
        if (fabsf(base_duty) < MOTOR_DEADBAND) {
            base_duty = (base_duty > 0.0f) ? MOTOR_DEADBAND : -MOTOR_DEADBAND;
        }
        base_duty = clamp(base_duty, -BALANCE_MAX_DUTY, BALANCE_MAX_DUTY);
    }

    /* ── LINE PID ────────────────────────────────────────────────────────── */
    float correction = 0.0f;
    if (do_line) {
        correction = pid_update(&s_line_pid, line_error, dt_s);
    }

    /* ── COMBINE: DIFFERENTIAL STEERING ─────────────────────────────────── */
    float left_cmd  = base_duty + correction;
    float right_cmd = base_duty - correction;

    /* ── EXTRACT DIRECTION AND CLAMP MAGNITUDE ───────────────────────────── */
    out->left_dir   = (left_cmd  > 0.0f) ?  1 : (left_cmd  < 0.0f) ? -1 : 0;
    out->right_dir  = (right_cmd > 0.0f) ?  1 : (right_cmd < 0.0f) ? -1 : 0;

    out->left_duty  = clamp(fabsf(left_cmd),  0.0f, BALANCE_MAX_DUTY);
    out->right_duty = clamp(fabsf(right_cmd), 0.0f, BALANCE_MAX_DUTY);
}

/* ─────────────────────────────────────────────────────────────────────────────
 * PUBLIC: PID gain setters / getters
 * ─────────────────────────────────────────────────────────────────────────────
 */
void set_balance_pid_gains(float kp, float ki, float kd)
{
    s_balance_pid.kp        = kp;
    s_balance_pid.ki        = ki;
    s_balance_pid.kd        = kd;
    s_balance_pid.integral  = 0.0f;  /* Reset on gain change */
    s_balance_pid.prev_error = 0.0f;
    ESP_LOGI(TAG, "Balance PID updated: Kp=%.2f Ki=%.3f Kd=%.3f", kp, ki, kd);
}

void set_line_pid_gains(float kp, float ki, float kd)
{
    s_line_pid.kp        = kp;
    s_line_pid.ki        = ki;
    s_line_pid.kd        = kd;
    s_line_pid.integral  = 0.0f;
    s_line_pid.prev_error = 0.0f;
    ESP_LOGI(TAG, "Line PID updated: Kp=%.2f Ki=%.3f Kd=%.3f", kp, ki, kd);
}

void get_balance_pid_gains(float *kp, float *ki, float *kd)
{
    *kp = s_balance_pid.kp;
    *ki = s_balance_pid.ki;
    *kd = s_balance_pid.kd;
}

void get_line_pid_gains(float *kp, float *ki, float *kd)
{
    *kp = s_line_pid.kp;
    *ki = s_line_pid.ki;
    *kd = s_line_pid.kd;
}

void set_balance_pitch_offset(float offset)
{
    s_balance_pitch_offset = offset;
    s_balance_pid.integral  = 0.0f;
    s_balance_pid.prev_error = 0.0f;
    ESP_LOGI(TAG, "Balance pitch offset set to %.2f°", offset);
}

float get_balance_pitch_offset(void)
{
    return s_balance_pitch_offset;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * PUBLIC: balance_controller_line_update()
 * ─────────────────────────────────────────────────────────────────────────────
 * For Mode 2: Pure line-following with caster wheel attached (differential drive).
 * Exact SRA reference implementation:
 *   optimum_duty_cycle = 57%
 *   lower_duty_cycle   = 45%
 *   higher_duty_cycle  = 65%
 *   error              = line_error * 10
 *   correction         = kp*error + ki*cum_error + kd*difference
 *   left_duty          = bound(57 + correction, 45, 65)
 *   right_duty         = bound(57 - correction, 45, 65)
 */
void balance_controller_line_update(float line_error,
                                    float dt_s,
                                    motor_output_t *out)
{
    static float prev_line_error = 0.0f;
    static float cumulative_error = 0.0f;

    float error = line_error * 10.0f;
    float difference = error - prev_line_error;
    cumulative_error += error;

    if (cumulative_error < -30.0f) cumulative_error = -30.0f;
    if (cumulative_error >  30.0f) cumulative_error =  30.0f;

    float correction = s_line_pid.kp * error + s_line_pid.ki * cumulative_error + s_line_pid.kd * difference;
    prev_line_error = error;

    float left_duty  = LINE_OPTIMUM_DUTY_CYCLE + correction;
    float right_duty = LINE_OPTIMUM_DUTY_CYCLE - correction;

    if (left_duty < LINE_LOWER_DUTY_CYCLE) left_duty = LINE_LOWER_DUTY_CYCLE;
    if (left_duty > LINE_HIGHER_DUTY_CYCLE) left_duty = LINE_HIGHER_DUTY_CYCLE;
    if (right_duty < LINE_LOWER_DUTY_CYCLE) right_duty = LINE_LOWER_DUTY_CYCLE;
    if (right_duty > LINE_HIGHER_DUTY_CYCLE) right_duty = LINE_HIGHER_DUTY_CYCLE;

    out->left_dir   = 1;
    out->right_dir  = 1;
    out->left_duty  = left_duty;
    out->right_duty = right_duty;
}
