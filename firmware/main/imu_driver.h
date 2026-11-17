/*
 * imu_driver.h
 * ============
 * Header file for the IMU (Inertial Measurement Unit) driver.
 * Wraps the SRA board's MPU6050 functions into a clean interface
 * for our balance controller.
 *
 * The MPU6050 chip on the SRA board contains TWO sensors in one package:
 *   1. Accelerometer: measures linear acceleration (gravity direction)
 *   2. Gyroscope:     measures angular velocity (rotation rate)
 *
 * We combine them using a complementary filter to get the pitch angle.
 *
 * HOW HEADER FILES WORK in C:
 *   A header (.h) file declares WHAT functions exist (their signatures).
 *   The source (.c) file defines HOW they work (their implementation).
 *   Other files that want to USE these functions #include the .h file.
 *
 *   This separation allows:
 *     - Faster compilation (compile .c once, include .h many times)
 *     - Clean interfaces (users see what's available, not how it works)
 */

#pragma once  // Include guard: prevents this header from being included twice
              // Alternative to the old #ifndef HEADER_H / #define HEADER_H pattern

#include <stdbool.h>
#include "esp_err.h"  // ESP-IDF error type (esp_err_t, ESP_OK, ESP_FAIL)

/*
 * imu_data_t
 * ----------
 * A struct that holds a complete snapshot of IMU state.
 * Our balance controller reads from this struct every 5ms (200 Hz).
 *
 * WHY A STRUCT?
 *   Instead of returning 4 separate values from read_imu(), we bundle
 *   them into one struct. This is cleaner, easier to pass around,
 *   and can be extended later without changing function signatures.
 */
typedef struct {
    float pitch_angle;   // Tilt angle in degrees (+ = forward, - = backward)
    float roll_angle;    // Side tilt in degrees (not used for balance, but available)
    float pitch_rate;    // How fast pitch is changing (degrees/second, from gyro)
    float roll_rate;     // How fast roll is changing (degrees/second, from gyro)
    bool  valid;         // Is this data fresh and valid? False if I2C read failed.
} imu_data_t;

/*
 * imu_init()
 * ----------
 * Initialize the MPU6050 sensor:
 *   1. Set up I2C bus on GPIO21 (SDA) and GPIO22 (SCL) at 400kHz
 *   2. Wake up the MPU6050 from sleep mode
 *   3. Configure accelerometer range: ±2g (most sensitive)
 *   4. Configure gyroscope range: ±250°/s (most sensitive for slow balancing)
 *   5. Run calibration: robot must be STILL and UPRIGHT during this!
 *
 * Returns: ESP_OK if successful, error code if I2C or sensor init failed.
 *
 * WHY esp_err_t RETURN TYPE?
 *   ESP-IDF convention. All hardware init functions return esp_err_t.
 *   Check with: if (imu_init() != ESP_OK) { handle error }
 */
esp_err_t imu_init(void);

/*
 * read_imu()
 * ----------
 * Read the latest pitch angle and rate from the MPU6050.
 * Uses the complementary filter (built into SRA board component).
 *
 * This is called at 200 Hz by the balance_task in main.c.
 * The complementary filter uses:
 *   pitch = 0.96 × (pitch + gyro_rate × dt) + 0.04 × accel_pitch
 *
 * Parameter: pointer to imu_data_t struct to fill with fresh data.
 */
void read_imu(imu_data_t *data);
