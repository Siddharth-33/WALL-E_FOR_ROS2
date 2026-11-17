/*
 * line_sensor.h
 * =============
 * Interface for the 5-channel IR Line Sensor Array (LSA).
 *
 * HARDWARE: LSA (Line Sensor Array) on SRA Board
 * -----------------------------------------------
 * 5 IR (infrared) sensors in a row, connected to ESP32's ADC:
 *   LSA_A0 = GPIO32
 *   LSA_A1 = GPIO35
 *   LSA_A2 = GPIO34
 *   LSA_A3 = GPIO39
 *   LSA_A4 = GPIO36
 *
 * HOW IR LINE SENSORS WORK:
 *   Each sensor has an IR LED (emitter) and an IR photodiode (receiver).
 *   The emitter shines IR light downward at the surface.
 *   The receiver measures how much bounces back:
 *     - WHITE surface: reflects lots of IR → high voltage → high ADC value
 *     - BLACK surface: absorbs IR → little reflected → low voltage → low ADC value
 *
 *   Wait — isn't that backwards from what we said in the .msg file?
 *   YES! The raw ADC is HIGH for white and LOW for black.
 *   But we INVERT during normalization: normalized = 1.0 - raw_normalized
 *   So in normalized space: 0.0 = white, 1.0 = black.
 *   Check with your actual sensor — some boards may differ.
 *
 * CALIBRATION:
 *   Min/max ADC values per sensor are stored and used to normalize readings.
 *   Calibration must be done before the robot runs.
 *   Process: sweep the robot slowly over the black line to record min/max.
 */

#pragma once
#include <stdbool.h>

#include "esp_err.h"

// Number of sensors in the LSA
#define LSA_SENSOR_COUNT 5

/*
 * lsa_data_t
 * ----------
 * A complete snapshot of line sensor state.
 * Read at 50 Hz by line_sensor_task in main.c.
 */
typedef struct {
    int   raw_adc[LSA_SENSOR_COUNT];        // Raw ADC values (0-4095)
    float normalized[LSA_SENSOR_COUNT];     // Normalized 0.0=white, 1.0=black
    float line_position;                    // -1.0=left, 0.0=center, +1.0=right
    bool  line_detected;                    // Is any part of the line visible?
} lsa_data_t;

/*
 * lsa_init()
 * ----------
 * Initialize the ADC channels for all 5 LSA sensors.
 * Configures ADC1 with 12-bit resolution and 11dB attenuation
 * (allows reading up to 3.9V on 3.3V ESP32).
 */
esp_err_t lsa_init(void);

/*
 * lsa_calibrate()
 * ---------------
 * Record min/max ADC values for each sensor by sweeping over the line.
 * Call this once before the main loop when robot is in IDLE mode.
 * Takes samples for 5 seconds while the robot is manually moved over the line.
 */
void lsa_calibrate(void);

/*
 * read_lsa()
 * ----------
 * Read all 5 sensors and compute normalized values and line position.
 * Called at 50 Hz from line_sensor_task.
 *
 * Fills the provided lsa_data_t struct with fresh readings.
 */
void read_lsa(lsa_data_t *data);
