cmake_minimum_required(VERSION 3.16)

# This is the TOP-LEVEL CMakeLists.txt for the ESP-IDF project.
# It's different from what you saw in ROS2 packages.
#
# ESP-IDF uses CMake too, but structured differently:
#   - This file declares the project name
#   - It tells CMake WHERE to find extra components
#   - Then includes ESP-IDF's own build system
#
# WHAT IS A COMPONENT in ESP-IDF?
#   ESP-IDF organizes code into "components" — self-contained libraries.
#   ESP-IDF itself is made of components (wifi, freertos, driver, etc.)
#   You can add your own components in the `components/` directory.
#
# We have two external components:
#   1. micro_ros_espidf_component  → micro-ROS library for ESP32
#   2. sra-board-component         → SRA Board drivers (motors, IMU, LSA)

# Tell CMake to look for components in our components/ directory
# In addition to the default ESP-IDF component search paths
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/components/micro_ros_espidf_component"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/sra-board-component"
)

# Standard ESP-IDF bootstrap
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Declare the project name (matches the binary name that gets flashed)
project(walle_ros2)
