cmake_minimum_required(VERSION 3.8)
project(su065d4380_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# su065d4380 interface ==============================================
ament_auto_add_library(
  ${PROJECT_NAME}
  STATIC
    ./src/port_handler.cpp
    ./src/packet_handler.cpp
    ./src/packet_pool.cpp
    ./src/commander/common.cpp
    ./src/commander/parameter_commander.cpp
    ./src/commander/velocity_commander.cpp
    ./src/commander/info_commander.cpp
)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
# End su065d4380 interface ==========================================

# show data executable ==============================================
ament_auto_add_executable(show_data ./examples/show_data.cpp)
target_link_libraries(show_data ${PROJECT_NAME})
# End show data executable ==========================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gtest(
    test_packet_pool
    test/src/test_packet_pool.cpp)
  target_include_directories(test_packet_pool PRIVATE "include")
  target_link_libraries(test_packet_pool ${PROJECT_NAME})

  ament_add_gmock(
    test_parameter_commander
    test/src/commander/test_parameter_commander.cpp)
  target_link_libraries(test_parameter_commander ${PROJECT_NAME})

  ament_add_gmock(
    test_velocity_commander
    test/src/commander/test_velocity_commander.cpp)
  target_link_libraries(test_velocity_commander ${PROJECT_NAME})

  ament_add_gmock(
    test_info_commander
    test/src/commander/test_info_commander.cpp)
  target_link_libraries(test_info_commander ${PROJECT_NAME})
endif()

ament_auto_package()
