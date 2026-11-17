#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "imu_driver.h"
#include "walle_motor.h"
#include "walle_lsa.h"
#include "balance_controller.h"
#include "state_machine.h"
#include "wifi_manager.h"
#include "microros_transport.h"

static const char *TAG = "MAIN";

SemaphoreHandle_t g_data_mutex  = NULL;

float    g_pitch_deg     = 0.0f;
float    g_roll_deg      = 0.0f;
uint16_t g_lsa[5]        = {0};
float    g_lsa_norm[5]   = {0.0f};
float    g_line_error    = 0.0f;
bool     g_line_detected = false;
float    g_left_duty     = 0.0f;
float    g_right_duty    = 0.0f;
int      g_left_dir      = 0;
int      g_right_dir     = 0;

static void balance_task(void *arg) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(10); // 100 Hz (same as SRA reference, eases I2C load)

    while (1) {
        imu_data_t imu_data = {0};
        read_imu(&imu_data);
        float pitch = imu_data.pitch_angle;
        float roll  = imu_data.roll_angle;

        robot_mode_t mode = get_robot_mode();

        float line_err = 0.0f;
        if (mode == ROBOT_LINE_FOLLOW || mode == ROBOT_BOTH) {
            if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                line_err = g_line_error;
                xSemaphoreGive(g_data_mutex);
            }
        }

        motor_output_t motor_out = {0};
        bool do_line = (mode == ROBOT_LINE_FOLLOW || mode == ROBOT_BOTH);

        if (mode == ROBOT_IDLE) {
            stop_motors();
            motor_out.left_duty = 0;
            motor_out.right_duty = 0;
            motor_out.left_dir = 0;
            motor_out.right_dir = 0;
        } else if (mode == ROBOT_LINE_FOLLOW) {
            // Mode 2: Pure line-following (with front caster wheel attached)
            // SRA reference optimum_duty_cycle is 57%
            balance_controller_line_update(line_err, 0.010f, &motor_out);
            float out_left = (motor_out.left_dir > 0) ? motor_out.left_duty : ((motor_out.left_dir < 0) ? -motor_out.left_duty : 0);
            float out_right = (motor_out.right_dir > 0) ? motor_out.right_duty : ((motor_out.right_dir < 0) ? -motor_out.right_duty : 0);
            set_motor_duties(out_left, out_right);
        } else if (imu_data.valid) {
            // Mode 1 (Balance only) or Mode 3 (Balancing line-follower)
            balance_controller_update(pitch, line_err, do_line, 0.010f, &motor_out);
            float out_left = (motor_out.left_dir > 0) ? motor_out.left_duty : ((motor_out.left_dir < 0) ? -motor_out.left_duty : 0);
            float out_right = (motor_out.right_dir > 0) ? motor_out.right_duty : ((motor_out.right_dir < 0) ? -motor_out.right_duty : 0);
            set_motor_duties(out_left, out_right);
        }

        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            g_pitch_deg = pitch;
            g_roll_deg = roll;
            g_left_duty = motor_out.left_duty;
            g_right_duty = motor_out.right_duty;
            g_left_dir = motor_out.left_dir;
            g_right_dir = motor_out.right_dir;
            xSemaphoreGive(g_data_mutex);
        }

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

static void line_sensor_task(void *arg) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(20); // 50 Hz

    while (1) {
        lsa_data_t lsa;
        read_lsa(&lsa);

        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            for (int i = 0; i < 5; i++) {
                g_lsa[i] = lsa.raw_adc[i];
                g_lsa_norm[i] = lsa.normalized[i];
            }
            g_line_error = lsa.line_position;
            g_line_detected = lsa.line_detected;
            xSemaphoreGive(g_data_mutex);
        }

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}


void app_main(void) {
    g_data_mutex = xSemaphoreCreateMutex();
    if (g_data_mutex == NULL) {
        esp_restart();
    }

    imu_init();
    motor_init();
    lsa_init();
    balance_controller_init();
    state_machine_init();

    if (wifi_manager_init()) {
        ESP_LOGI(TAG, "Wi-Fi connected! micro-ROS will start.");
    } else {
        ESP_LOGW(TAG, "Wi-Fi failed to connect. Robot runs offline (no micro-ROS).");
    }

    xTaskCreatePinnedToCore(balance_task, "balance", 4096, NULL, 24, NULL, 1);
    xTaskCreatePinnedToCore(line_sensor_task, "line_sensor", 2048, NULL, 20, NULL, 1);
    xTaskCreatePinnedToCore(state_machine_task, "state_machine", 2048, NULL, 18, NULL, 1);
    
    if (is_wifi_connected()) {
        xTaskCreatePinnedToCore(microros_spin_task, "microros", 16384, NULL, 15, NULL, 0);
    } else {
        ESP_LOGW(TAG, "micro-ROS task NOT started (no Wi-Fi). Robot operates standalone.");
    }
}
