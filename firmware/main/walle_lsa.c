/*
 * walle_lsa.c
 * ===========
 * Line sensor array implementation.
 *
 * EXACT PINS (from SRA board pin_defs.h):
 *   LSA_A0 = GPIO32 (leftmost sensor)
 *   LSA_A1 = GPIO35
 *   LSA_A2 = GPIO34 (center sensor)
 *   LSA_A3 = GPIO39
 *   LSA_A4 = GPIO36 (rightmost sensor)
 *
 * All on ADC1 (ADC2 cannot be used when Wi-Fi is active on ESP32).
 * This is a hardware limitation: ADC2 shares resources with Wi-Fi RF.
 *
 * WEIGHTED AVERAGE EXPLAINED:
 *   weights = [-2, -1, 0, +1, +2] for sensors [A0, A1, A2, A3, A4]
 *   These weights represent the physical position of each sensor.
 *   Sensor A0 is leftmost (weight -2), A4 is rightmost (weight +2).
 *
 *   line_position = sum(weight[i] * normalized[i]) / (sum(normalized[i]) * 2.0)
 *
 *   When only A2 sees black: position = (0 * 1.0) / (1.0 * 2.0) = 0.0 ✓ (center)
 *   When only A0 sees black: position = (-2 * 1.0) / (1.0 * 2.0) = -1.0 ✓ (left)
 *   When A1 and A2 both see black: position = (-1*0.8 + 0*0.9) / ((0.8+0.9)*2.0) ≈ -0.24
 */

#include "walle_lsa.h"
#include "lsa.h"            // SRA board LSA driver
#include "pin_defs.h"       // GPIO pin definitions
#include "esp_log.h"
#include <math.h>

static const char *TAG = "LSA";

// SRA reference weights for 5 line sensors: {-5, -3, 1, 3, 5}
// Index 0 = leftmost (GPIO32=LSA_A0), index 4 = rightmost (GPIO36=LSA_A4)

// Calibration values — default margins for TCRT5000 IR sensors
// White line reflecting IR: raw ADC ~ 300 to 900
// Dark/black arena floor: raw ADC ~ 3200 to 4095
static int cal_min[LSA_SENSOR_COUNT] = {400, 400, 400, 400, 400};
static int cal_max[LSA_SENSOR_COUNT] = {3800, 3800, 3800, 3800, 3800};

// SRA board ADC handle
static adc_handle_t adc_handle;

esp_err_t lsa_init(void)
{
    ESP_LOGI(TAG, "Initializing LSA on GPIO32,35,34,39,36...");

    /*
     * enable_line_sensor() from SRA board component configures:
     *   - ADC1 channel for each of the 5 LSA pins
     *   - 12-bit resolution (0-4095)
     *   - 11dB attenuation (reads 0-3.9V)
     *   - Multisampling (CONFIG_NUMBER_OF_SAMPLES=5 samples averaged per read)
     *
     * ADC attenuation explained:
     *   The ESP32's ADC measures 0-1.1V by default.
     *   With 11dB attenuation, it measures 0-3.9V (needed for 3.3V sensors).
     */
    esp_err_t ret = enable_line_sensor(&adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LSA init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "LSA initialized. Run lsa_calibrate() before use.");
    return ESP_OK;
}

void lsa_calibrate(void)
{
    /*
     * Calibration process:
     *   Take 500 readings over 5 seconds while robot is moved over the line.
     *   Record the minimum and maximum ADC value seen per sensor.
     *   These become our "white reference" (min) and "black reference" (max).
     *
     * WHY CALIBRATE?
     *   ADC readings vary with:
     *     - Ambient lighting (sunlight adds IR noise)
     *     - Surface reflectivity (matte vs glossy white differs by ~500 counts)
     *     - Distance from ground (if robot tilts, sensors are farther away)
     *   Calibration normalizes all of this away.
     */
    ESP_LOGI(TAG, "Calibrating LSA... Move robot slowly over the line for 5 seconds");

    for (int i = 0; i < 500; i++) {
        line_sensor_array raw = read_line_sensor(adc_handle);

        for (int s = 0; s < LSA_SENSOR_COUNT; s++) {
            int val = raw.adc_reading[s];
            if (val < cal_min[s]) cal_min[s] = val;
            if (val > cal_max[s]) cal_max[s] = val;
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms delay → 500 × 10ms = 5 seconds
    }

    ESP_LOGI(TAG, "Calibration complete:");
    for (int s = 0; s < LSA_SENSOR_COUNT; s++) {
        ESP_LOGI(TAG, "  Sensor %d: min=%d, max=%d", s, cal_min[s], cal_max[s]);
    }
}

#define BLACK_MARGIN 4095
#define WHITE_MARGIN 0
#define bound_LSA_LOW 0
#define bound_LSA_HIGH 1000
#define BLACK_BOUNDARY  820    // Boundary value to distinguish between black and white readings

void read_lsa(lsa_data_t *data)
{
    // Read raw multisampled ADC values from all 5 sensors
    line_sensor_array raw = read_line_sensor(adc_handle);

    static float prev_error = 0.0f;
    const int weights[5] = {-5, -3, 1, 3, 5};
    int all_black_flag = 1;
    float weighted_sum = 0.0f;
    float sum = 0.0f;
    float pos = 0.0f;

    for (int i = 0; i < 5; i++) {
        data->raw_adc[i] = raw.adc_reading[i];

        int bounded = raw.adc_reading[i];
        if (bounded < WHITE_MARGIN) bounded = WHITE_MARGIN;
        if (bounded > BLACK_MARGIN) bounded = BLACK_MARGIN;

        // Map from 0..4095 to 0..1000 and invert (SRA reference mapping)
        int mapped = (bounded - WHITE_MARGIN) * (bound_LSA_HIGH - bound_LSA_LOW) / (BLACK_MARGIN - WHITE_MARGIN) + bound_LSA_LOW;
        int inverted = 1000 - mapped;

        // Normalized for ROS visualization: 0.0 = dark floor, 1.0 = bright white line
        float norm = (float)inverted / 1000.0f;
        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;
        data->normalized[i] = norm;

        int k = 0;
        if (inverted > BLACK_BOUNDARY) {
            all_black_flag = 0;
            k = 1;
        }

        weighted_sum += (float)weights[i] * k;
        sum += (float)k;
    }

    if (sum != 0.0f) {
        pos = (weighted_sum - 1.0f) / sum;
    }

    if (all_black_flag == 1) {
        data->line_detected = false;
        if (prev_error > 0.0f) {
            data->line_position = 2.5f;
        } else {
            data->line_position = -2.5f;
        }
    } else {
        data->line_detected = true;
        data->line_position = pos;
        prev_error = pos;
    }
}
