## This is the COMPONENT-level CMakeLists.txt for the `main` component.
##
## ═══════════════════════════════════════════════════════════════════════════════
## HOW ESP-IDF CMakeLists FILES DIFFER FROM TOP-LEVEL
## ═══════════════════════════════════════════════════════════════════════════════
##
## ESP-IDF has a two-level CMake structure:
##
##   TOP-LEVEL (~/esp/walle_ros2/CMakeLists.txt)
##   ─────────────────────────────────────────────
##   - Calls cmake_minimum_required() and project()
##   - Calls include($ENV{IDF_PATH}/tools/cmake/project.cmake) which sets up
##     the entire ESP-IDF build system
##   - Uses add_subdirectory() or relies on IDF_EXTRA_COMPONENTS_DIRS to
##     discover component directories
##
##   COMPONENT-LEVEL (this file: main/CMakeLists.txt)
##   ─────────────────────────────────────────────────
##   - Does NOT call cmake_minimum_required() or project()
##   - Calls idf_component_register() – an ESP-IDF CMake function (not standard
##     CMake) that registers this directory as a "component"
##   - Specifies SOURCE files, INCLUDE directories, and REQUIRES (dependencies)
##
## WHAT IS AN ESP-IDF "COMPONENT"?
## ---------------------------------
## An ESP-IDF component is the fundamental build unit (similar to a CMake
## library target).  Every piece of code lives in a component directory with:
##   - CMakeLists.txt (this file)
##   - One or more .c and .h files
##   - Optionally: Kconfig.projbuild (adds menuconfig options)
##
## The `main` component is special: it's always built and must contain app_main().
##
## REQUIRES vs PRIV_REQUIRES:
## ---------------------------
##   REQUIRES      – public dependency: headers exposed to OTHER components that
##                   REQUIRE this component will also see these dependencies.
##   PRIV_REQUIRES – private dependency: only THIS component needs it; other
##                   components depending on `main` won't automatically get it.
##
##   In practice, `main` is the top-level component (nothing depends on it),
##   so REQUIRES and PRIV_REQUIRES are equivalent here. We use REQUIRES for
##   clarity.
##
## WHY EACH REQUIRED COMPONENT:
## ------------------------------
##   sra-board-component       – SRA board BSP: MPU6050, motor driver, LSA APIs
##   micro_ros_espidf_component – micro-ROS pre-built library for ESP-IDF
##   driver                    – ESP-IDF peripheral drivers (GPIO, LEDC, ADC)
##   esp_wifi                  – Wi-Fi driver
##   freertos                  – FreeRTOS (always required for tasks, semaphores)
##   nvs_flash                 – Non-volatile storage (required by Wi-Fi driver)
##   esp_event                 – Event loop (required by Wi-Fi events)
##   esp_timer                 – High-resolution timer (for ROS timestamps)
##   spi_flash                 – Flash access layer (indirect dependency of NVS)

cmake_minimum_required(VERSION 3.16)

idf_component_register(
    SRCS
        ## ── Source files for this component ──────────────────────────────
        ## List every .c file; CMake will compile each into a .o object file
        ## and link them into the final firmware .elf image.
        "main.c"
        "imu_driver.c"
        "walle_motor.c"
        "walle_lsa.c"
        "balance_controller.c"
        "state_machine.c"
        "wifi_manager.c"
        "microros_transport.c"

    INCLUDE_DIRS
        ## ── Header search paths ───────────────────────────────────────────
        ## "." means: look for #include "foo.h" in THIS directory (main/).
        ## This is needed so code in this component can #include "balance_controller.h"
        ## etc. without specifying a path prefix.
        "."

    REQUIRES
        ## ── Component dependencies ────────────────────────────────────────
        ## These must exactly match the component directory names under
        ## ~/esp/walle_ros2/components/ or the ESP-IDF install directory.
        sra-board-component
        micro_ros_espidf_component
        driver
        esp_wifi
        freertos
        nvs_flash
        esp_event
        esp_timer
)
