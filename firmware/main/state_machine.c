/**
 * @file state_machine.c
 * @brief Robot FSM implementation: DIP switch reading + atomic mode variable.
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * WHY THIS FILE EXISTS
 * ═══════════════════════════════════════════════════════════════════════════════
 * The robot has two independent inputs for changing operating mode:
 *   1. Physical DIP switches (SW1 = GPIO4, SW2 = GPIO2) – set by the operator.
 *   2. A micro-ROS /set_mode service – sent from the ROS 2 host PC.
 *
 * Both sources can write the mode variable from different FreeRTOS tasks
 * running on different CPU cores.  This module centralises all mode-change
 * logic so the rest of the firmware only calls get_robot_mode().
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * FREERTOS CONCURRENCY AND WHY volatile + portMUX MATTERS
 * ═══════════════════════════════════════════════════════════════════════════════
 *
 * The ESP32 has TWO Xtensa LX6 cores (PRO_CPU = Core 0, APP_CPU = Core 1).
 * FreeRTOS runs tasks on both cores simultaneously (true parallelism, not just
 * cooperative multitasking).
 *
 * Problem scenario WITHOUT any protection:
 *   - Core 0 (microros_spin_task) reads mode into register R1
 *   - Core 1 (state_machine_task) also reads mode from memory into R2
 *   - Core 1 writes R2+1 back to mode memory (e.g., 0 → 1)
 *   - Core 0 writes R1+2 back to mode memory (e.g., 0 → 2, overwriting Core 1's write)
 *   → Race condition: mode = 2 but we wanted 1 from Core 1's write.
 *
 * For a single uint8_t on LX6, a store is ONE instruction and is therefore
 * hardware-atomic (it can't be half-written).  However, the compiler may
 * optimise away re-reads because it "thinks" no other code changes the variable.
 *
 * FIX:
 *   `volatile`         – prevents compiler from caching value in registers.
 *                        Every read/write goes to/from RAM.
 *   `portMUX_TYPE`     – a lightweight spinlock (no context switch needed;
 *                        FreeRTOS enters a critical section disabling interrupts
 *                        on the current core and using a hardware mutex on
 *                        the shared bus for cross-core protection).
 *
 *   portENTER_CRITICAL(&mux)  – disables interrupts + acquires spinlock
 *   portEXIT_CRITICAL(&mux)   – releases spinlock + re-enables interrupts
 *
 * We use the critical section ONLY around the write (set_robot_mode).
 * Reads (get_robot_mode) are safe without locking on LX6 because reading
 * a single byte is atomic and volatile prevents stale caching.
 *
 * Alternative: FreeRTOS mutex (xSemaphoreTake/Give) would also work but
 * is ~10× slower and cannot be used inside ISRs or critical sections.
 * For a simple enum value, portMUX is the right tool.
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * GPIO CONFIGURATION FOR DIP SWITCHES
 * ═══════════════════════════════════════════════════════════════════════════════
 * SW1 = GPIO4,  SW2 = GPIO2
 * The switches connect GPIO to 3.3 V when ON.  We configure INPUT_PULLDOWN
 * so the GPIO reads 0 when the switch is OFF and 1 when ON.
 * Reading is done with gpio_get_level() which returns 0 or 1.
 *
 * Mode encoding: mode = (SW2 << 1) | SW1
 *   SW1=0, SW2=0 → 0b00 = 0 = ROBOT_IDLE
 *   SW1=1, SW2=0 → 0b01 = 1 = ROBOT_BALANCE_ONLY
 *   SW1=0, SW2=1 → 0b10 = 2 = ROBOT_LINE_FOLLOW
 *   SW1=1, SW2=1 → 0b11 = 3 = ROBOT_BOTH
 */

#include "state_machine.h"

#include "driver/gpio.h"         /* gpio_config_t, gpio_get_level() */
#include "freertos/FreeRTOS.h"   /* portMUX_TYPE, portENTER/EXIT_CRITICAL */
#include "freertos/task.h"       /* vTaskDelay(), pdMS_TO_TICKS() */
#include "esp_log.h"             /* ESP_LOGI */

static const char *TAG = "STATE_MACHINE";

/* GPIO numbers for DIP switches (from SRA board pin_defs.h) */
#define SW1_GPIO   4    /* GPIO4 → bit 0 of mode */
#define SW2_GPIO   2    /* GPIO2 → bit 1 of mode */

/* ─────────────────────────────────────────────────────────────────────────────
 * GLOBAL VOLATILE MODE VARIABLE + SPINLOCK
 * ─────────────────────────────────────────────────────────────────────────────
 * `volatile` tells the C compiler: "do NOT put this in a register; always
 * read/write from actual RAM".  Without volatile, the compiler might load
 * s_robot_mode into a register once at the start of balance_task and never
 * re-read memory, so mode changes from the other core would be invisible.
 *
 * `portMUX_INITIALIZER_UNLOCKED` initialises the spinlock to "not held".
 */
static volatile robot_mode_t s_robot_mode = ROBOT_IDLE;
static portMUX_TYPE s_mode_mux = portMUX_INITIALIZER_UNLOCKED;

/* ─────────────────────────────────────────────────────────────────────────────
 * state_machine_init()
 * ─────────────────────────────────────────────────────────────────────────────
 */
void state_machine_init(void)
{
    /* Configure SW1 (GPIO4) as input with internal pull-down resistor.
     * gpio_config_t is a structure that bundles all GPIO settings.
     * We use it instead of individual gpio_set_direction() / gpio_set_pull_mode()
     * calls so the configuration is applied atomically. */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SW1_GPIO) | (1ULL << SW2_GPIO),
        /* Input mode: we read the voltage level but don't drive it */
        .mode         = GPIO_MODE_INPUT,
        /* Pull-down: GPIO floats LOW (0) when the switch is open */
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        /* No interrupt: we poll the switch in the task loop */
        .intr_type    = GPIO_INTR_DISABLE,
    };
    /* Apply the configuration – this writes to the GPIO matrix registers */
    gpio_config(&io_conf);

    /* Read initial switch state and set starting mode */
    int sw1 = gpio_get_level(SW1_GPIO); /* 0 or 1 */
    int sw2 = gpio_get_level(SW2_GPIO); /* 0 or 1 */

    /* Encode into 2-bit mode: bit0=SW1, bit1=SW2 */
    robot_mode_t initial_mode = (robot_mode_t)((sw2 << 1) | sw1);
    set_robot_mode(initial_mode);

    ESP_LOGI(TAG, "State machine init: SW1=%d SW2=%d → mode=%d", sw1, sw2, initial_mode);
}

/* ─────────────────────────────────────────────────────────────────────────────
 * get_robot_mode()  –  LOCK-FREE READ
 * ─────────────────────────────────────────────────────────────────────────────
 * Called from balance_task at 200 Hz (every 5 ms).  We MUST NOT take a mutex
 * here because:
 *   a) balance_task has very high priority (24) – blocking on a mutex would
 *      cause priority inversion if microros_spin_task (priority 15) holds it.
 *   b) The read of a uint8_t-sized enum is hardware-atomic on LX6.
 *   c) The `volatile` keyword ensures the compiler always re-reads from memory.
 */
robot_mode_t get_robot_mode(void)
{
    return s_robot_mode; /* Volatile read – always fetches fresh value from RAM */
}

/* ─────────────────────────────────────────────────────────────────────────────
 * set_robot_mode()  –  SPINLOCK-PROTECTED WRITE
 * ─────────────────────────────────────────────────────────────────────────────
 * Can be called from state_machine_task (Core 1) OR microros_spin_task (Core 0).
 * The portENTER_CRITICAL/portEXIT_CRITICAL pair:
 *   1. Disables interrupts on the calling core (prevents ISR mid-write).
 *   2. Acquires a hardware bus mutex (prevents the other core from writing
 *      simultaneously using the DPORT hardware arbitration mechanism on ESP32).
 */
void set_robot_mode(robot_mode_t mode)
{
    /* Validate: clamp to known modes to prevent garbage values */
    if (mode > ROBOT_BOTH) {
        ESP_LOGW(TAG, "Invalid mode %d requested; ignoring", mode);
        return;
    }

    portENTER_CRITICAL(&s_mode_mux);  /* Lock: disable interrupts + acquire spinlock */
    s_robot_mode = mode;
    portEXIT_CRITICAL(&s_mode_mux);   /* Unlock: re-enable interrupts + release spinlock */

    ESP_LOGI(TAG, "Mode set → %d", mode);
}

/* ─────────────────────────────────────────────────────────────────────────────
 * state_machine_task()  –  FREERTOS TASK (10 Hz polling loop)
 * ─────────────────────────────────────────────────────────────────────────────
 * This task runs every 100 ms (10 Hz).  That is more than fast enough for
 * a DIP switch – humans can't toggle a switch faster than ~2 Hz, and
 * mechanical bounce settles within a few milliseconds.
 *
 * WHY A TASK INSTEAD OF AN ISR?
 * DIP switches bounce (rapidly toggle) for 1–20 ms after being flipped.
 * An interrupt would fire dozens of times during bounce.  Polling at 10 Hz
 * samples the switch well after bounce has settled, giving a clean reading.
 * For real-time critical inputs (encoder pulses) we'd use interrupts, but
 * mode selection is a "slow" human input.
 */
void state_machine_task(void *arg)
{
    ESP_LOGI(TAG, "State machine task started on core %d", xPortGetCoreID());

    /*
     * ── WHY WE TRACK PREVIOUS SWITCH STATE ───────────────────────────────────
     * BUG FIX: Previously, this task polled DIP switches every 100 ms and
     * always called set_robot_mode() with the switch-derived mode.  If both
     * switches are OFF (SW1=0, SW2=0 → mode=IDLE), that overwrote any mode
     * set via the ROS 2 /set_mode service, causing the robot to stop within
     * 100 ms of every ROS command — even when the user wanted BALANCE mode.
     *
     * FIX: We remember the previous switch readings.  set_robot_mode() is
     * only called when a DIP switch physically changes.  If the switches don't
     * move, the ROS 2 service-set mode is preserved indefinitely.
     *
     * Initialise prev_sw to current hardware state so an initial "change"
     * event fires at startup (via state_machine_init() which already called
     * set_robot_mode() with the boot-time switch state).
     */
    int prev_sw1 = gpio_get_level(SW1_GPIO);
    int prev_sw2 = gpio_get_level(SW2_GPIO);

    while (1) {
        /* Read current physical switch state */
        int sw1 = gpio_get_level(SW1_GPIO);
        int sw2 = gpio_get_level(SW2_GPIO);

        /*
         * Only update mode when a DIP switch physically CHANGES.
         * This allows /set_mode service calls to persist between switch flips.
         * Example: switches OFF, user sends "m 1" via ROS → robot balances.
         *          State machine leaves mode alone until a switch is toggled.
         */
        if (sw1 != prev_sw1 || sw2 != prev_sw2) {
            robot_mode_t new_mode = (robot_mode_t)((sw2 << 1) | sw1);
            set_robot_mode(new_mode);
            ESP_LOGI(TAG, "DIP switch changed: SW1=%d SW2=%d → mode=%d", sw1, sw2, new_mode);
            prev_sw1 = sw1;
            prev_sw2 = sw2;
        }

        /* vTaskDelay suspends this task for 100 ms, yielding the CPU.       */
        /* pdMS_TO_TICKS converts milliseconds to FreeRTOS ticks (usually    */
        /* 1 tick = 1 ms with configTICK_RATE_HZ = 1000, but always use      */
        /* pdMS_TO_TICKS for portability to different tick rates).            */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    /* FreeRTOS tasks must never return.  If they do, the scheduler will     */
    /* enter an undefined state.  We use an infinite loop to prevent this.   */
}
