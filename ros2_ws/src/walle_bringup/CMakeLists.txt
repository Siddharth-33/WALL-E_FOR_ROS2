cmake_minimum_required(VERSION 3.8)
project(walle_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files and config files to share directory
# Same pattern as walle_description — no C++ code, just data files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
