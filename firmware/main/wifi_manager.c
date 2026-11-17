/**
 * @file wifi_manager.c
 * @brief ESP-IDF event-driven Wi-Fi station connection with auto-reconnect.
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * ESP-IDF WI-FI ARCHITECTURE – HOW IT WORKS INTERNALLY
 * ═══════════════════════════════════════════════════════════════════════════════
 *
 * The ESP-IDF Wi-Fi driver is entirely event-driven.  There is NO blocking
 * "connect and block until done" function.  Instead:
 *
 *   ┌──────────────────────────────────────────────────────────────────────┐
 *   │  Your code                                                           │
 *   │    ├─ esp_wifi_start()       → fires WIFI_EVENT_STA_START           │
 *   │    └─ esp_wifi_connect()     → fires WIFI_EVENT_STA_CONNECTED or    │
 *   │                                       WIFI_EVENT_STA_DISCONNECTED   │
 *   │                                                                      │
 *   │  esp_event_loop (background task)                                    │
 *   │    └─ calls wifi_event_handler() for each event                     │
 *   │                                                                      │
 *   │  Your event handler:                                                 │
 *   │    ├─ WIFI_EVENT_STA_START       → esp_wifi_connect()               │
 *   │    ├─ WIFI_EVENT_STA_DISCONNECTED → retry or set FAIL bit           │
 *   │    └─ IP_EVENT_STA_GOT_IP        → set CONNECTED bit                │
 *   └──────────────────────────────────────────────────────────────────────┘
 *
 * EVENT GROUPS (FreeRTOS primitive):
 * -----------------------------------
 * An EventGroup is a 32-bit integer where each bit is a flag.
 * xEventGroupSetBits()   – sets one or more bits (from any task/ISR)
 * xEventGroupWaitBits()  – blocks until specified bits are set (OR/AND logic)
 *
 * wifi_manager_init() blocks on xEventGroupWaitBits() waiting for either:
 *   WIFI_CONNECTED_BIT → success
 *   WIFI_FAIL_BIT      → gave up after retries
 *
 * This is the canonical ESP-IDF pattern shown in their Wi-Fi station example.
 *
 * NETWORK INTERFACE (esp_netif):
 * --------------------------------
 * esp_netif is the ESP-IDF abstraction layer over the LWIP TCP/IP stack.
 * esp_netif_create_default_wifi_sta() creates a "station" netif, sets up
 * DHCP client, etc.  After connecting, LWIP runs a DHCP request and fires
 * IP_EVENT_STA_GOT_IP when an IP address is assigned.
 *
 * NVS (Non-Volatile Storage):
 * ---------------------------
 * The Wi-Fi driver internally stores calibration data in NVS flash.
 * nvs_flash_init() must be called before esp_wifi_init(); otherwise the
 * driver will panic.
 */

#include "wifi_manager.h"

#include <string.h>              /* memset()                                  */

#include "esp_wifi.h"            /* esp_wifi_init, esp_wifi_connect, etc.     */
#include "esp_event.h"           /* esp_event_handler_register, event IDs     */
#include "esp_log.h"             /* ESP_LOGI, ESP_LOGE                        */
#include "esp_netif.h"           /* esp_netif_init, esp_netif_create_default… */
#include "nvs_flash.h"           /* nvs_flash_init                            */

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h" /* EventGroupHandle_t, xEventGroupWaitBits */

static const char *TAG = "WIFI_MGR";

/* ─────────────────────────────────────────────────────────────────────────────
 * EVENT GROUP BIT DEFINITIONS
 * ─────────────────────────────────────────────────────────────────────────────
 * Bit 0 (value 1) = connected and IP obtained
 * Bit 1 (value 2) = failed after WIFI_MAXIMUM_RETRY attempts
 */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/* Module-private state */
static EventGroupHandle_t s_wifi_event_group; /* FreeRTOS event group handle   */
static volatile bool      s_wifi_connected = false; /* True once IP is obtained */
static int                s_retry_count    = 0;     /* Number of reconnect tries */

/* ─────────────────────────────────────────────────────────────────────────────
 * PRIVATE: wifi_event_handler()
 * ─────────────────────────────────────────────────────────────────────────────
 * This function is registered as a callback for both WIFI_EVENT and IP_EVENT.
 * The ESP-IDF event loop calls it from a background system task whenever a
 * Wi-Fi or IP event occurs.
 *
 * Parameters (required by esp_event_handler_t signature):
 *   handler_arg – our user data pointer (NULL in this case)
 *   event_base  – WIFI_EVENT or IP_EVENT
 *   event_id    – specific event within that base
 *   event_data  – event-specific payload (e.g., ip_event_got_ip_t)
 */
static void wifi_event_handler(void        *handler_arg,
                               esp_event_base_t event_base,
                               int32_t      event_id,
                               void        *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        /* Wi-Fi driver started → attempt connection to the access point.   */
        /* This triggers an association request (802.11 auth + assoc frames) */
        esp_wifi_connect();
        ESP_LOGI(TAG, "STA started, attempting connection…");

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        /* Connection dropped (wrong password, AP out of range, or timeout). */
        s_wifi_connected = false;

        if (s_retry_count < WIFI_MAXIMUM_RETRY) {
            /* Auto-reconnect: try again immediately.                        */
            esp_wifi_connect();
            s_retry_count++;
            ESP_LOGW(TAG, "Disconnected. Retry %d/%d…", s_retry_count, WIFI_MAXIMUM_RETRY);
        } else {
            /* Give up after max retries. Signal failure so wifi_manager_init
             * can return false and app_main can handle the error.           */
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Failed to connect after %d attempts", WIFI_MAXIMUM_RETRY);
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        /* LWIP DHCP client successfully obtained an IP address.            */
        /* event_data points to ip_event_got_ip_t which contains the IP.   */
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

        /* IPSTR and IP2STR are ESP-IDF macros to format esp_ip4_addr_t     */
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));

        /* Reset retry counter so a future disconnect starts fresh          */
        s_retry_count   = 0;
        s_wifi_connected = true;

        /* Unblock wifi_manager_init() which is waiting on this bit         */
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ─────────────────────────────────────────────────────────────────────────────
 * PUBLIC: wifi_manager_init()
 * ─────────────────────────────────────────────────────────────────────────────
 */
bool wifi_manager_init(void)
{
    /* ── Step 1: Initialise NVS flash ───────────────────────────────────── */
    /* The Wi-Fi driver uses NVS to persist calibration data (PHY calibration,
     * last connected AP info, etc.).  Without NVS init, esp_wifi_init() will
     * crash with "nvs: NVS partition not found".                            */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition may be corrupted (e.g., after a firmware flash that
         * changed partition layout).  Erase and reinitialise.              */
        ESP_LOGW(TAG, "NVS partition needs erase/reinit");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ── Step 2: Create FreeRTOS Event Group ────────────────────────────── */
    /* We wait on this group in Step 8.  Must be created before registering
     * the event handler (which may set bits during init on very fast APs). */
    s_wifi_event_group = xEventGroupCreate();

    /* ── Step 3: Initialise the TCP/IP network interface ────────────────── */
    /* esp_netif abstracts LWIP. Creates the default station (STA) interface */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default()); /* Background event dispatcher */
    esp_netif_create_default_wifi_sta();              /* Creates netif for Wi-Fi STA  */

    /* ── Step 4: Initialise the Wi-Fi driver with default config ─────────── */
    /* wifi_init_config_t sets buffer sizes, task priorities, etc.           */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* ── Step 5: Register event handlers ─────────────────────────────────── */
    /* We subscribe to ALL events in the WIFI_EVENT base and ALL in IP_EVENT. */
    /* ESP_EVENT_ANY_ID means "call me for every event ID in this base".      */
    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    /* ── Step 6: Configure Wi-Fi STA credentials from Kconfig ───────────── */
    /* CONFIG_ESP_WIFI_SSID and CONFIG_ESP_WIFI_PASSWORD are defined in
     * Kconfig.projbuild and read from sdkconfig at compile time.            */
    wifi_config_t wifi_config = {
        .sta = {
            .ssid     = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            /* threshold.authmode: only connect to APs with WPA2 or better  */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            /* pmf_cfg: Protected Management Frames – security feature       */
            .pmf_cfg = {
                .capable  = true,
                .required = false,
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));   /* Station (client) mode */
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    /* ── Step 7: Start the Wi-Fi driver ─────────────────────────────────── */
    /* This fires WIFI_EVENT_STA_START → our handler calls esp_wifi_connect() */
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi started. Waiting for connection to '%s'…",
             CONFIG_ESP_WIFI_SSID);

    /* ── Step 8: Block until connected or failed ─────────────────────────── */
    /* xEventGroupWaitBits() suspends this task and yields the CPU until:    */
    /*   a) WIFI_CONNECTED_BIT is set (success), OR                         */
    /*   b) WIFI_FAIL_BIT is set (gave up after retries)                    */
    /*   c) portMAX_DELAY means wait forever (no timeout)                   */
    /* The pdFALSE for xClearOnExit means bits remain set after this call    */
    /* so other code can also check them.                                    */
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,        /* Don't clear bits on exit */
        pdFALSE,        /* Wait for EITHER bit (OR semantics), not both (AND) */
        portMAX_DELAY); /* Block indefinitely */

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP '%s' successfully", CONFIG_ESP_WIFI_SSID);
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to connect to AP '%s'", CONFIG_ESP_WIFI_SSID);
        return false;
    }
}

/* ─────────────────────────────────────────────────────────────────────────────
 * PUBLIC: is_wifi_connected()
 * ─────────────────────────────────────────────────────────────────────────────
 * Returns the `volatile bool` maintained by the event handler.
 * Safe to call from any task; volatile prevents stale cached reads.
 */
bool is_wifi_connected(void)
{
    return s_wifi_connected;
}
