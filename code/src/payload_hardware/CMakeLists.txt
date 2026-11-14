cmake_minimum_required(VERSION 3.16)
project(payload_hardware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- 1. ROS-free part: CAN codec + core logic + SocketCAN. Builds and tests on any Linux box. ----
add_library(payload_can_core STATIC src/can_core.cpp src/socketcan.cpp)
target_include_directories(payload_can_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(test_core test/test_core.cpp)
target_link_libraries(test_core payload_can_core)
enable_testing()
add_test(NAME core_tests COMMAND test_core ${CMAKE_CURRENT_SOURCE_DIR}/test/vectors.txt)

# ---- 2. ros2_control plugin: only when building inside a ROS 2 workspace ----
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(hardware_interface REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)

  add_library(payload_hardware SHARED src/payload_can_system.cpp)
  target_link_libraries(payload_hardware PUBLIC payload_can_core)
  ament_target_dependencies(payload_hardware PUBLIC hardware_interface pluginlib rclcpp rclcpp_lifecycle)
  pluginlib_export_plugin_description_file(hardware_interface payload_hardware.xml)

  install(TARGETS payload_hardware payload_can_core
          EXPORT export_${PROJECT_NAME}
          ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
  install(DIRECTORY include/ DESTINATION include)
  install(DIRECTORY config tools DESTINATION share/${PROJECT_NAME})
  install(PROGRAMS scripts/generate_can_table.py DESTINATION lib/${PROJECT_NAME})

  ament_export_include_directories(include)
  ament_export_libraries(payload_hardware payload_can_core)
  ament_export_targets(export_${PROJECT_NAME})
  ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
  ament_package()
else()
  message(STATUS "ament_cmake not found: building only the ROS-free core and its tests")
endif()
