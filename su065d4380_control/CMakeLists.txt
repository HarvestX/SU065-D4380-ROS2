cmake_minimum_required(VERSION 3.8)
project(su065d4380_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ===================================================================
ament_auto_add_library(${PROJECT_NAME} SHARED src/su065d4380_system.cpp)
pluginlib_export_plugin_description_file(
  hardware_interface
  su065d4380_control.xml)
# ===================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
