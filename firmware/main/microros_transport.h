/**
 * @file microros_transport.h
 * @brief micro-ROS setup, publishers, services, and spin task for Wall-E.
 *
 * WHAT IS micro-ROS?
 * ------------------
 * micro-ROS is a port of ROS 2 (Robot Operating System 2) to microcontrollers.
 * It uses a subset of the full rcl/rclc API and communicates with a full ROS 2
 * network via a "micro-ROS agent" process running on a Linux PC.
 *
 * THE COMMUNICATION STACK (bottom to top):
 *
 *   Physical layer:  Wi-Fi (802.11 b/g/n)
 *   Transport:       UDP/IP (unreliable but low-latency; suitable for sensors)
 *   Middleware:      DDS (Data Distribution Service) – micro-XRCE-DDS
 *   micro-ROS API:   rcl (ROS Client Library) + rclc (C convenience layer)
 *   Application:     Our publishers, subscribers, services
 *
 * KEY CONCEPTS:
 * -------------
 *
 * Allocator:
 *   micro-ROS needs to allocate memory for messages, queues, etc.
 *   On bare-metal MCUs we use rcl_get_default_allocator() which calls
 *   malloc/free.  This is fine on ESP32 which has a heap.
 *
 * Support / Init Options:
 *   rclc_support_t bundles the allocator, init options, and DDS context.
 *   rclc_support_init() connects to the agent and creates the DDS participant.
 *
 * Node:
 *   A "node" is the fundamental unit of participation in a ROS 2 graph.
 *   Every publisher and service belongs to a node.  Our node is named "walle".
 *
 * Publisher:
 *   A publisher sends messages on a named "topic".  Any ROS 2 subscriber
 *   on the network with a matching topic name and message type will receive them.
 *   Publishers are non-blocking: publish() returns immediately.
 *
 * Service Server:
 *   A service provides request/response (like an RPC call).
 *   A client sends a request; the server sends a response.
 *   We use services for /set_pid_gains and /set_mode so the operator can
 *   change gains without reflashing firmware.
 *
 * Executor:
 *   The rclc_executor processes incoming service requests and subscriber
 *   callbacks.  rclc_executor_spin_some() must be called periodically to
 *   drain the receive queue.  We call it from our FreeRTOS task loop.
 *
 * CUSTOM TRANSPORT (UDP):
 *   micro-ROS defaults to serial UART.  For UDP we call:
 *     rmw_uros_set_custom_transport(false,          // not stream-based
 *                                   &transport_arg, // IP + port config
 *                                   open_fn, close_fn, write_fn, read_fn)
 *   The four callback functions implement raw UDP socket send/receive using
 *   LWIP (the TCP/IP stack built into ESP-IDF).
 *
 * PUBLISHERS AND THEIR FREQUENCIES:
 *   /imu/data            sensor_msgs/Imu            100 Hz  (every 10 ms)
 *   /line_sensors        walle_interfaces/LineSensorArray  50 Hz (every 20 ms)
 *   /balance_telemetry   walle_interfaces/BalanceTelemetry 50 Hz
 *   /joint_states        sensor_msgs/JointState     50 Hz
 *
 * SERVICES:
 *   /set_pid_gains   walle_interfaces/SetPidGains  (read-write PID gains)
 *   /set_mode        walle_interfaces/SetMode      (change robot FSM mode)
 */

#ifndef MICROROS_TRANSPORT_H
#define MICROROS_TRANSPORT_H

/* ─────────────────────────────────────────────────────────────────────────────
 * FUNCTION PROTOTYPES
 * ─────────────────────────────────────────────────────────────────────────────
 */

/**
 * @brief FreeRTOS task: run the micro-ROS node (publish + service spin).
 *
 * This task:
 *   1. Creates the rclc_support, node, publishers, service servers, executor.
 *   2. Enters an infinite loop that:
 *        a. Reads shared IMU/LSA/controller data (under mutex).
 *        b. Publishes to all four topics at their respective rates.
 *        c. Calls rclc_executor_spin_some() to handle incoming service requests.
 *        d. vTaskDelay(10 ms) – caps the loop at ~100 Hz max.
 *
 * Pinned to Core 0 so it does not interfere with the real-time balance_task
 * on Core 1.
 *
 * @param arg Unused (required by FreeRTOS task signature).
 */
void microros_spin_task(void *arg);

#endif /* MICROROS_TRANSPORT_H */
