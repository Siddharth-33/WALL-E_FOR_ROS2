cmake_minimum_required(VERSION 3.8)
project(walle_description)

find_package(ament_cmake REQUIRED)

# ── What walle_description contains ──────────────────────────────────────────
# This package has NO C++ code — it's purely data files:
#   - urdf/walle.urdf.xacro  → robot model
#   - rviz/walle_view.rviz   → RViz2 configuration
#   - launch/                → launch files
#
# ament_cmake still manages it, but we just need to INSTALL the data files
# into the colcon install directory so other packages can find them.
#
# install(DIRECTORY ...) copies an entire directory into the install space.
# DESTINATION share/${PROJECT_NAME} puts it at:
#   ~/sid_ws/install/walle_description/share/walle_description/
# This is where FindPackageShare('walle_description') looks!
# ─────────────────────────────────────────────────────────────────────────────

install(DIRECTORY
  urdf
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
