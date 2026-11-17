/**
 * @file state_machine.h
 * @brief Robot operating-mode FSM for Wall-E (Balance-only / Line-follow / Both / Idle).
 *
 * WHAT IS A FINITE STATE MACHINE (FSM)?
 * --------------------------------------
 * An FSM is a model of computation consisting of:
 *   - A finite set of STATES  (the modes our robot can be in)
 *   - TRANSITIONS between states triggered by inputs (DIP switches, ROS service)
 *   - ACTIONS performed when in a state or on a transition (start/stop tasks)
 *
 * Our FSM has exactly 4 states mapped to 2-bit combinations of SW1+SW2:
 *
 *   SW1=0, SW2=0  → IDLE         (motors off, sensors idle)
 *   SW1=1, SW2=0  → BALANCE_ONLY (balance PID running, no line follow)
 *   SW1=0, SW2=1  → LINE_FOLLOW  (line PID running, assume flat surface)
 *   SW1=1, SW2=1  → BOTH         (balance + line follow simultaneously)
 *
 * WHY A GLOBAL ATOMIC MODE VARIABLE?
 * -----------------------------------
 * The robot mode can be changed from TWO places concurrently:
 *   1. The state_machine_task (running on Core 1) reads DIP switches.
 *   2. The microros_spin_task (running on Core 0) handles a ROS service call.
 *
 * Because these tasks run on different FreeRTOS cores (ESP32 is dual-core),
 * a plain `uint8_t` write from one core may be only partially visible to
 * the other core due to CPU caching and out-of-order execution.
 *
 * Solutions:
 *   - `volatile`: tells the compiler never to cache the variable in a register;
 *     always re-read it from RAM. This prevents compiler-level data races but
 *     does NOT prevent CPU-level races on multi-core.
 *
 *   - Mutex: a FreeRTOS mutex (binary semaphore with ownership) guarantees
 *     that only one task can read/write the variable at a time. However,
 *     taking a mutex from an ISR or very high-priority task can cause
 *     priority inversion or delay.
 *
 * For a single-byte variable (uint8_t) on Xtensa LX6 (ESP32), a single
 * store/load is naturally atomic at the hardware level, so we use:
 *   - `volatile` to prevent compiler optimizations
 *   - A portMUX_TYPE spinlock for the rare simultaneous-write case
 *
 * This gives us lock-free reads (safe and fast in balance_task at 200 Hz)
 * while writes are protected by a brief critical section.
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>

/* ─────────────────────────────────────────────────────────────────────────────
 * ROBOT MODE ENUMERATION
 * ─────────────────────────────────────────────────────────────────────────────
 * Using named constants instead of magic numbers makes code self-documenting
 * and prevents accidental assignment of invalid mode values.
 */
typedef enum {
    ROBOT_IDLE         = 0, /**< Both switches OFF: do nothing, save power   */
    ROBOT_BALANCE_ONLY = 1, /**< SW1 ON, SW2 OFF: balance PID only           */
    ROBOT_LINE_FOLLOW  = 2, /**< SW1 OFF, SW2 ON: line-follow on flat surface */
    ROBOT_BOTH         = 3, /**< Both switches ON: balance + line-follow      */
} robot_mode_t;

/* ─────────────────────────────────────────────────────────────────────────────
 * FUNCTION PROTOTYPES
 * ─────────────────────────────────────────────────────────────────────────────
 */

/**
 * @brief Initialize the state machine.
 *
 * - Configures SW1 (GPIO4) and SW2 (GPIO2) as inputs with internal pull-down.
 * - Reads initial DIP switch state to set starting mode.
 * - Initializes the spinlock protecting the mode variable.
 *
 * Must be called once from app_main() before creating tasks.
 */
void state_machine_init(void);

/**
 * @brief Return the current robot operating mode.
 *
 * This is a lock-free read (safe to call from high-priority balance_task
 * at 200 Hz).  The `volatile` qualifier on the backing variable ensures
 * the compiler always reads from memory rather than a cached register.
 *
 * @return One of: ROBOT_IDLE, ROBOT_BALANCE_ONLY, ROBOT_LINE_FOLLOW, ROBOT_BOTH
 */
robot_mode_t get_robot_mode(void);

/**
 * @brief Set the robot operating mode (protected by spinlock).
 *
 * Can be called from:
 *   - state_machine_task (DIP switch polling)
 *   - microros_spin_task (ROS /set_mode service callback)
 *
 * Uses a portMUX_TYPE critical section to atomically update the global mode
 * variable without risking a torn write visible to the other core.
 *
 * @param mode New robot_mode_t value.
 */
void set_robot_mode(robot_mode_t mode);

/**
 * @brief FreeRTOS task: poll DIP switches and update mode every 100 ms.
 *
 * Reads SW1 and SW2, computes (SW1 | SW2<<1), calls set_robot_mode() if
 * the computed mode differs from the current one.
 *
 * This is intentionally slow (10 Hz) because DIP switches are mechanical
 * and can bounce; we don't need microsecond reaction time for mode changes.
 *
 * @param arg Unused (required by FreeRTOS task signature).
 */
void state_machine_task(void *arg);

#endif /* STATE_MACHINE_H */
