#include "microros_transport.h"
#include "esp_log.h"
#include <string.h>          // memcpy(), strcmp()
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/init_options.h>
#include "esp_timer.h"
#include "wifi_manager.h"
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <walle_interfaces/msg/line_sensor_array.h>
#include <walle_interfaces/msg/balance_telemetry.h>
#include <walle_interfaces/srv/set_pid_gains.h>
#include <walle_interfaces/srv/set_mode.h>
#include "balance_controller.h"
#include "state_machine.h"

static const char *TAG = "MICROROS";

static rcl_publisher_t pub_imu;
static rcl_publisher_t pub_lsa;
static rcl_publisher_t pub_telemetry;
static rcl_publisher_t pub_joint;

static rcl_service_t srv_pid;
static rcl_service_t srv_mode;
static walle_interfaces__srv__SetPidGains_Request pid_req;
static walle_interfaces__srv__SetPidGains_Response pid_resp;
static walle_interfaces__srv__SetMode_Request mode_req;
static walle_interfaces__srv__SetMode_Response mode_resp;

extern SemaphoreHandle_t g_data_mutex;
extern float g_pitch_deg;
extern float g_roll_deg;
extern uint16_t g_lsa[5];
extern float g_lsa_norm[5];
extern float g_line_error;
extern bool g_line_detected;
extern float g_left_duty;
extern float g_right_duty;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}

static void set_pid_gains_callback(const void *request_msg, void *response_msg) {
    const walle_interfaces__srv__SetPidGains_Request  *req = (const walle_interfaces__srv__SetPidGains_Request *)request_msg;
    walle_interfaces__srv__SetPidGains_Response *resp = (walle_interfaces__srv__SetPidGains_Response *)response_msg;

    if (req->controller.data != NULL && strcmp(req->controller.data, "balance") == 0) {
        set_balance_pid_gains(req->kp, req->ki, req->kd);
        resp->success = true;
    } else if (req->controller.data != NULL && strcmp(req->controller.data, "line") == 0) {
        set_line_pid_gains(req->kp, req->ki, req->kd);
        resp->success = true;
    } else if (req->controller.data != NULL && (strcmp(req->controller.data, "offset") == 0 || strcmp(req->controller.data, "setpoint") == 0)) {
        set_balance_pitch_offset(req->kp);
        resp->success = true;
    } else {
        resp->success = false;
    }
}

static void set_mode_callback(const void *request_msg, void *response_msg) {
    const walle_interfaces__srv__SetMode_Request  *req = (const walle_interfaces__srv__SetMode_Request *)request_msg;
    walle_interfaces__srv__SetMode_Response *resp = (walle_interfaces__srv__SetMode_Response *)response_msg;

    set_robot_mode((robot_mode_t)req->mode);
    resp->success = true;
}

void microros_spin_task(void *arg) {
    ESP_LOGI(TAG, "micro-ROS task started");

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    RCCHECK(rclc_node_init_default(&node, "walle", "", &support));

    RCCHECK(rclc_publisher_init_default(&pub_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data"));
    RCCHECK(rclc_publisher_init_default(&pub_lsa, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(walle_interfaces, msg, LineSensorArray), "/line_sensors"));
    RCCHECK(rclc_publisher_init_default(&pub_telemetry, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(walle_interfaces, msg, BalanceTelemetry), "/balance_telemetry"));
    RCCHECK(rclc_publisher_init_default(&pub_joint, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/joint_states"));

    RCCHECK(rclc_service_init_default(&srv_pid, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(walle_interfaces, srv, SetPidGains), "/set_pid_gains"));
    RCCHECK(rclc_service_init_default(&srv_mode, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(walle_interfaces, srv, SetMode), "/set_mode"));

    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_service(&executor, &srv_pid, &pid_req, &pid_resp, set_pid_gains_callback));
    RCCHECK(rclc_executor_add_service(&executor, &srv_mode, &mode_req, &mode_resp, set_mode_callback));

    sensor_msgs__msg__Imu imu_msg;
    walle_interfaces__msg__LineSensorArray lsa_msg;
    walle_interfaces__msg__BalanceTelemetry tel_msg;
    sensor_msgs__msg__JointState joint_msg;

    sensor_msgs__msg__Imu__init(&imu_msg);
    walle_interfaces__msg__LineSensorArray__init(&lsa_msg);
    walle_interfaces__msg__BalanceTelemetry__init(&tel_msg);
    sensor_msgs__msg__JointState__init(&joint_msg);

    imu_msg.header.frame_id.data = "imu_link";
    imu_msg.header.frame_id.size = 8;
    imu_msg.header.frame_id.capacity = 9;

    lsa_msg.header.frame_id.data = "lsa_link";
    lsa_msg.header.frame_id.size = 8;
    lsa_msg.header.frame_id.capacity = 9;

    uint32_t loop_count = 0;
    while (1) {
        float pitch = 0, roll = 0, left_duty = 0, right_duty = 0, line_pos = 0;
        bool line_det = false;
        uint16_t lsa[5] = {0};
        float lsa_norm[5] = {0.0f};

        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            pitch = g_pitch_deg; roll = g_roll_deg;
            left_duty = g_left_duty; right_duty = g_right_duty;
            line_pos = g_line_error; line_det = g_line_detected;
            memcpy(lsa, g_lsa, sizeof(lsa));
            memcpy(lsa_norm, g_lsa_norm, sizeof(lsa_norm));
            xSemaphoreGive(g_data_mutex);
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        int64_t now_us = esp_timer_get_time();
        int32_t sec = (int32_t)(now_us / 1000000LL);
        int32_t nsec = (int32_t)((now_us % 1000000LL) * 1000LL);

        imu_msg.header.stamp.sec = sec;
        imu_msg.header.stamp.nanosec = (uint32_t)nsec;
        imu_msg.orientation.w = 1.0;
        imu_msg.orientation.x = roll * 0.00872664625;
        imu_msg.orientation.y = pitch * 0.00872664625;
        imu_msg.orientation.z = 0.0;
        imu_msg.orientation_covariance[0] = -1.0;
        RCCHECK(rcl_publish(&pub_imu, &imu_msg, NULL));

        if (loop_count % 2 == 0) {
            lsa_msg.header.stamp.sec = sec;
            lsa_msg.header.stamp.nanosec = (uint32_t)nsec;
            for (int i = 0; i < 5; i++) {
                lsa_msg.raw_adc[i] = (float)lsa[i];
                lsa_msg.normalized[i] = lsa_norm[i];
            }
            lsa_msg.line_position = line_pos;
            lsa_msg.line_detected = line_det;
            RCCHECK(rcl_publish(&pub_lsa, &lsa_msg, NULL));

            tel_msg.header.stamp.sec = sec;
            tel_msg.header.stamp.nanosec = (uint32_t)nsec;
            tel_msg.pitch_angle = pitch;
            tel_msg.motor_left_duty = left_duty;
            tel_msg.motor_right_duty = right_duty;
            tel_msg.mode = (uint8_t)get_robot_mode();
            tel_msg.wifi_connected = is_wifi_connected();
            tel_msg.agent_connected = true;
            float dummy_float = 0.0f;
            get_balance_pid_gains(&tel_msg.pid_p_term, &tel_msg.pid_i_term, &tel_msg.pid_d_term);
            get_line_pid_gains(&tel_msg.steering_offset, &tel_msg.forward_velocity, &dummy_float);
            tel_msg.target_pitch = get_balance_pitch_offset();
            RCCHECK(rcl_publish(&pub_telemetry, &tel_msg, NULL));

            RCCHECK(rcl_publish(&pub_joint, &joint_msg, NULL));
        }

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
        loop_count++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
