/**
 * @file wifi_manager.h
 * @brief ESP-IDF event-driven Wi-Fi connection manager for Wall-E.
 *
 * WHY WI-FI IS NEEDED:
 * ---------------------
 * micro-ROS communicates with the ROS 2 agent running on a host PC over UDP/IP.
 * The ESP32 must first connect to the same Wi-Fi network as the agent so that
 * IP packets can travel between them.
 *
 * HOW ESP-IDF WI-FI WORKS (EVENT-DRIVEN MODEL):
 * -----------------------------------------------
 * ESP-IDF does NOT use a blocking "connect and wait" API.  Instead it uses an
 * event-loop + event-group pattern:
 *
 *   1. esp_netif_init()         – initialises the TCP/IP network interface
 *   2. esp_event_loop_create_default() – creates a background task that
 *                                  dispatches events to registered handlers
 *   3. esp_wifi_init()          – allocates driver structures
 *   4. esp_event_handler_register() – we subscribe to WIFI_EVENT and IP_EVENT
 *   5. esp_wifi_start()         – starts the Wi-Fi radio; fires WIFI_EVENT_STA_START
 *   6. wifi_event_handler()     – our callback is invoked for every subscribed event:
 *        WIFI_EVENT_STA_START   → call esp_wifi_connect()
 *        WIFI_EVENT_STA_DISCONNECTED → set reconnect timer, clear connected flag
 *        IP_EVENT_STA_GOT_IP    → log IP, set wifi_connected = true
 *
 * EVENT GROUPS:
 * -------------
 * A FreeRTOS EventGroup is a set of bit flags protected by the RTOS scheduler.
 * We create one EventGroup with two bits:
 *   WIFI_CONNECTED_BIT (bit 0) – set when we have an IP address
 *   WIFI_FAIL_BIT      (bit 1) – set after MAX_RETRY failed attempts
 *
 * wifi_manager_init() blocks with xEventGroupWaitBits() until one of these
 * bits is set (or a timeout), then returns so app_main() can continue.
 * This is cleaner than a busy-wait loop because the FreeRTOS scheduler can
 * run other tasks while we wait.
 *
 * AUTO-RECONNECT:
 * ---------------
 * In the DISCONNECTED handler we call esp_wifi_connect() again (with a
 * retry counter). Once connected, the retry counter resets.  If we exhaust
 * retries, the WIFI_FAIL_BIT is set and wifi_manager_init() returns false,
 * letting app_main() decide what to do (e.g., keep retrying).
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>

/* Maximum number of reconnection attempts before giving up */
#define WIFI_MAXIMUM_RETRY  5

/* ─────────────────────────────────────────────────────────────────────────────
 * FUNCTION PROTOTYPES
 * ─────────────────────────────────────────────────────────────────────────────
 */

/**
 * @brief Initialise and start the Wi-Fi station.
 *
 * Reads SSID and password from Kconfig (CONFIG_ESP_WIFI_SSID and
 * CONFIG_ESP_WIFI_PASSWORD).  Blocks until connected or max retries exceeded.
 *
 * Sets the global `wifi_connected` flag to true on success.
 *
 * @return true  if connected and IP obtained
 *         false if connection failed after WIFI_MAXIMUM_RETRY attempts
 */
bool wifi_manager_init(void);

/**
 * @brief Thread-safe check of current Wi-Fi connectivity.
 *
 * Safe to call from any task.  Reads a volatile bool updated by the
 * ESP-IDF event handler.
 *
 * @return true if Wi-Fi is connected and we have a valid IP.
 */
bool is_wifi_connected(void);

#endif /* WIFI_MANAGER_H */
