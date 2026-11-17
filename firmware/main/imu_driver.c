/*
 * imu_driver.c
 * ============
 * Implementation of the IMU driver. Uses the SRA board component's
 * MPU6050 functions (mpu6050.h) which handle the low-level I2C
 * communication and complementary filter math.
 *
 * PIN ASSIGNMENTS (from SRA board pin_defs.h):
 *   SDA: GPIO21
 *   SCL: GPIO22
 *   INT: GPIO23 (interrupt pin — not used, we poll instead)
 *
 * WHY POLLING INSTEAD OF INTERRUPT?
 *   The MPU6050 has an interrupt pin that fires when new data is ready.
 *   Using it (interrupt-driven) would be more efficient, but polling
 *   at 200 Hz with a FreeRTOS task is simpler and reliable enough
 *   for our balance controller.
 */

#include "imu_driver.h"
#include "mpu6050.h"    // SRA board MPU6050 driver (handles I2C, registers)
#include "esp_log.h"    // ESP_LOGI, ESP_LOGE macros for logging to serial

static const char *TAG = "IMU_DRIVER";

// mpu_offset stores the "zero angle" of the robot.
// During calibration, the robot must be held upright.
// The offset is subtracted from future readings so 0° = balanced.
static float mpu_offset[2] = {0.0f, 0.0f};  // [0]=roll offset, [1]=pitch offset

esp_err_t imu_init(void)
{
    ESP_LOGI(TAG, "Initializing MPU6050...");

    /*
     * enable_mpu6050() is from the SRA board component (mpu6050.h).
     * It performs:
     *   1. i2cdev_init() → sets up ESP-IDF I2C driver on the bus
     *   2. Writes to MPU6050 register 0x6B (PWR_MGMT_1) to wake it from sleep
     *   3. Writes to 0x1B (GYRO_CONFIG) to set ±250°/s range
     *   4. Writes to 0x1C (ACCEL_CONFIG) to set ±2g range
     *
     * WHY ±250°/s for gyro?
     *   A balancing robot tilts slowly (rarely more than 30°/s).
     *   Lower range = higher sensitivity = more precise readings.
     *   If we used ±2000°/s, the small tilt angles would be lost in noise.
     *
     * WHY ±2g for accelerometer?
     *   Our robot only experiences ~1g of gravitational acceleration when upright.
     *   ±2g is the most sensitive range that still captures this.
     */
    esp_err_t ret = enable_mpu6050();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /*
     * Attempt calibration.
     * Note: calibrate_mpu6050() tries to converge to error <= 5 LSBs.
     * If the bot is slightly tilted, vibrating, or on a desk that isn't perfectly
     * level, it can exceed the attempt limit and return ESP_FAIL.
     * We don't abort initialization on calibration failure: enable_mpu6050() already
     * succeeded and the complementary filter will still operate with factory/uncalibrated offsets.
     */
    ESP_LOGI(TAG, "Calibrating MPU6050... Keep robot STILL and UPRIGHT!");
    ret = calibrate_mpu6050();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "MPU6050 calibration convergence limit reached: continuing with default offsets.");
    } else {
        ESP_LOGI(TAG, "MPU6050 calibrated successfully");
    }

    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

void read_imu(imu_data_t *data)
{
    /*
     * euler_angle is a 2-element array: [roll, pitch]
     * The SRA board's read_mpu6050() fills this using:
     *
     *   Step 1: Read raw 16-bit values from MPU6050 registers via I2C
     *   Step 2: Apply calibration offsets
     *   Step 3: Run complementary filter:
     *
     *     accel_pitch = atan2(ay, az) × (180/π)   ← from gravity direction
     *     gyro_rate   = gx / 131.0                 ← raw / sensitivity (LSB per °/s)
     *     pitch       = 0.96 × (pitch + gyro_rate × dt) + 0.04 × accel_pitch
     *
     *   The 0.96/0.04 split means:
     *     96% trust the gyroscope for fast, accurate short-term changes
     *     4%  trust the accelerometer to correct long-term gyro drift
     */
    float euler_angle[2];  // [0] = roll, [1] = pitch

    esp_err_t ret = read_mpu6050(euler_angle, mpu_offset);

    static float s_last_pitch = 0.0f;
    static float s_last_roll  = 0.0f;
    static float prev_pitch   = 0.0f;
    static bool first_call    = true;

    if (ret == ESP_OK) {
        s_last_pitch = euler_angle[1];
        s_last_roll  = euler_angle[0];
        data->pitch_angle = s_last_pitch;
        data->roll_angle  = s_last_roll;
        data->valid = true;

        if (first_call) {
            prev_pitch = data->pitch_angle;
            first_call = false;
        }

        // dt = 10ms = 0.010s (called at 100 Hz from balance_task)
        data->pitch_rate = (data->pitch_angle - prev_pitch) / 0.010f;
        prev_pitch = data->pitch_angle;

    } else {
        // I2C glitch — maintain last valid readings so controller doesn't jump
        data->pitch_angle = s_last_pitch;
        data->roll_angle  = s_last_roll;
        data->pitch_rate  = 0.0f;
        data->valid       = false;
        ESP_LOGW(TAG, "MPU6050 I2C glitch: %s (holding last valid pitch %.1f°)", esp_err_to_name(ret), s_last_pitch);
    }
}
