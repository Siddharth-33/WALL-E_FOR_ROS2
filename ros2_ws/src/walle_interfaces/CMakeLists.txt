cmake_minimum_required(VERSION 3.8)
project(walle_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── What is CMakeLists.txt? ────────────────────────────────────────
# CMake is a build system. This file tells CMake:
#   1. What other packages this package DEPENDS on
#   2. What files to GENERATE code from (our .msg and .srv files)
#   3. How to INSTALL everything so other packages can find it
#
# When you run `colcon build`, CMake reads this file first.
# ──────────────────────────────────────────────────────────────────

# Find ament_cmake — the ROS2 build system toolkit
find_package(ament_cmake REQUIRED)

# Find rosidl_default_generators — the tool that READS your .msg/.srv files
# and AUTO-GENERATES code in multiple languages:
#   - C headers   (for ESP32 micro-ROS firmware)
#   - Python classes (for host-side Python nodes)
#   - C++ classes (for any C++ nodes)
# You write ONE .msg file, rosidl generates code for ALL languages.
find_package(rosidl_default_generators REQUIRED)

# This package provides std_msgs/Header which we use in our messages.
# Any package whose types appear in YOUR .msg files must be listed here.
find_package(std_msgs REQUIRED)

# ── Tell rosidl WHICH files to generate code from ─────────────────
# List every .msg and .srv file here.
# After this runs, you'll be able to do in Python:
#   from walle_interfaces.msg import LineSensorArray
# And in C (ESP32):
#   #include <walle_interfaces/msg/line_sensor_array.h>
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LineSensorArray.msg"
  "msg/BalanceTelemetry.msg"
  "srv/SetPidGains.srv"
  "srv/SetMode.srv"
  DEPENDENCIES std_msgs
)

# Export runtime dependencies so other packages (walle_control, etc.)
# that depend on walle_interfaces can find it at runtime
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
