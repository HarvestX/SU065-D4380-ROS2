cmake_minimum_required(VERSION 3.8)
project(su065d4380_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# su065d4380 interface ==============================================
file(GLOB src_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)
file(GLOB commander_src_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/commander/*.cpp)
set(TARGET packet_handler)
set(MYLIB ${PROJECT_NAME}_${TARGET})
ament_auto_add_library(${MYLIB} SHARED ${src_files} ${commander_src_files})

# Testing ===========================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(TARGETS
    test_rx_info_packet
    test_tx_rx_vel_packet
    test_tx_rx_param_read_packet
    test_tx_rx_param_write_packet)

  foreach(TARGET ${TARGETS})
    ament_add_gmock(${TARGET} ./test/src/${TARGET}.cpp)
    target_link_libraries(${TARGET} ${MYLIB})
  endforeach()
endif()

ament_auto_package()
