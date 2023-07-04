cmake_minimum_required(VERSION 3.8)
project(su065d4380_tool)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(TARGET su065d4380_parameters)
generate_parameter_library(${TARGET} src/${TARGET}.yaml)

# Param Reader ======================================================
set(TARGET param_reader_node)
ament_auto_add_library(${TARGET} SHARED src/${TARGET}.cpp)
rclcpp_components_register_node(
  ${TARGET}
  PLUGIN "${PROJECT_NAME}::ParamReaderNode"
  EXECUTABLE ${TARGET}_exec)
target_link_libraries(${TARGET} su065d4380_parameters)

# Param Writer ======================================================
set(TARGET param_writer_node)
ament_auto_add_library(${TARGET} SHARED src/${TARGET}.cpp)
rclcpp_components_register_node(
  ${TARGET}
  PLUGIN "${PROJECT_NAME}::ParamWriterNode"
  EXECUTABLE ${TARGET}_exec)
target_link_libraries(${TARGET} su065d4380_parameters)

# TESTING ===========================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
