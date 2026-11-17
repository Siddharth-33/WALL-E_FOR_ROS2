/*
 * walle_motor.c
 * =============
 * Motor driver implementation using the SRA board component.
 *
 * EXACT PINS (from SRA board pin_defs.h):
 *   Motor A (LEFT  wheel): IN1=GPIO13, IN2=GPIO12
 *   Motor B (RIGHT wheel): IN3=GPIO26, IN4=GPIO25
 *
 * DEADBAND EXPLANATION:
 *   Real DC motors have a "deadband" — a range of duty cycles near zero
 *   where the motor receives power but doesn't actually move.
 *   This is because the motor needs to overcome static friction first.
 *   On our TT motors with the SRA board, this deadband is approximately 20-25%.
 *
 *   If we send duty=5%, the motor is barely energized and just heats up.
 *   Deadband compensation: any |duty| > 0 but < DEADBAND is boosted to DEADBAND.
 *   Any |duty| that IS 0 stays at 0 (motor off).
 *
 *   Without deadband compensation: robot hunts (oscillates in small range near balance)
 *   With deadband compensation: smoother, more responsive balance
 */

#include "walle_motor.h"
#include "motor_driver.h"   // SRA board motor driver API
#include "pin_defs.h"       // GPIO pin definitions from SRA board
#include "esp_log.h"
#include <math.h>           // fabsf()

static const char *TAG = "MOTOR";

// Deadband threshold: any duty below this is boosted to this value
// (if the requested duty is non-zero). Tune this for your motors.
// SRA board TT motors typically need ~20-25% to overcome static friction.
#define MOTOR_DEADBAND 20.0f

// If your left or right motor spins in the wrong direction,
// change these to -1. Test this in IDLE mode by sending small positive duties.
#define MOTOR_LEFT_POLARITY  (+1.0f)
#define MOTOR_RIGHT_POLARITY (+1.0f)

// SRA board motor handles (opaque pointers to motor objects)
static motor_handle_t motor_left  = NULL;
static motor_handle_t motor_right = NULL;

esp_err_t motor_init(void)
{
    ESP_LOGI(TAG, "Initializing motor drivers...");

    /*
     * enable_motor_driver() is from the SRA board component.
     * It sets up the MCPWM peripheral for one motor channel.
     *
     * MOTOR_A_0 → uses GPIO13 and GPIO12 (Motor A, pair 0)
     * MOTOR_A_1 → uses GPIO26 and GPIO25 (Motor A, pair 1)
     *
     * Internally it calls:
     *   mcpwm_gpio_init() → routes GPIO to MCPWM peripheral
     *   mcpwm_init()      → configures PWM at 20kHz (MCPWM_FREQ)
     */
    esp_err_t ret;

    ret = enable_motor_driver(&motor_left, MOTOR_A_0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Left motor init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = enable_motor_driver(&motor_right, MOTOR_A_1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Right motor init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Motors initialized (Left: GPIO13/12, Right: GPIO26/25)");
    return ESP_OK;
}

/*
 * apply_deadband()
 * ----------------
 * If |duty| is between 0 and DEADBAND (exclusive), boost it to DEADBAND.
 * If duty is 0, keep it at 0 (true stop).
 */
static float apply_deadband(float duty)
{
    if (fabsf(duty) < 0.001f) {
        return 0.0f;  // True zero → stop motor completely
    }
    if (fabsf(duty) < MOTOR_DEADBAND) {
        // Below deadband but non-zero → boost to minimum effective duty
        return (duty > 0) ? MOTOR_DEADBAND : -MOTOR_DEADBAND;
    }
    return duty;
}

void set_motor_duties(float left_duty, float right_duty)
{
    // Apply direction polarity (in case motors are mounted backwards)
    left_duty  *= MOTOR_LEFT_POLARITY;
    right_duty *= MOTOR_RIGHT_POLARITY;

    // Apply deadband compensation
    left_duty  = apply_deadband(left_duty);
    right_duty = apply_deadband(right_duty);

    // Clamp to valid range
    if (left_duty  >  100.0f) left_duty  =  100.0f;
    if (left_duty  < -100.0f) left_duty  = -100.0f;
    if (right_duty >  100.0f) right_duty =  100.0f;
    if (right_duty < -100.0f) right_duty = -100.0f;

    /*
     * set_motor_speed() is from the SRA board component.
     * Parameters:
     *   motor:      the motor handle
     *   direction:  MOTOR_FORWARD or MOTOR_BACKWARD
     *   duty_cycle: 0.0 to 100.0 (absolute value — direction is separate)
     *
     * We translate our signed duty into direction + magnitude:
     */
    if (motor_left != NULL) {
        int dir_l = (left_duty >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
        set_motor_speed(motor_left,  dir_l, fabsf(left_duty));
    }

    if (motor_right != NULL) {
        int dir_r = (right_duty >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
        set_motor_speed(motor_right, dir_r, fabsf(right_duty));
    }
}

void stop_motors(void)
{
    // Set duty to 0 → set_motor_speed with duty=0 → motor coasts to stop
    if (motor_left  != NULL) set_motor_speed(motor_left,  MOTOR_STOP, 0.0f);
    if (motor_right != NULL) set_motor_speed(motor_right, MOTOR_STOP, 0.0f);
}
