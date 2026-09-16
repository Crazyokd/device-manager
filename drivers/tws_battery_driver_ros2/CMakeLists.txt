cmake_minimum_required(VERSION 3.8)
project(tws_battery_driver_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(device_manager_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BmsStatus.msg"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(${PROJECT_NAME}_core SHARED
  src/battery_snapshot.cpp
  src/socketcan_transport.cpp
  src/tws_battery_driver.cpp
  src/tws_battery_runtime.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  device_manager_core
  pluginlib
  rclcpp
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_core "${cpp_typesupport_target}")
pluginlib_export_plugin_description_file(
  device_manager_core tws_battery_runtime_plugins.xml)

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(
  device_manager_core
  pluginlib
  rclcpp
  sensor_msgs
  rosidl_default_runtime
)

if(BUILD_TESTING)
  find_package(device_manager_ros REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_soc_spike_filter test/test_soc_spike_filter.cpp)
  if(TARGET test_soc_spike_filter)
    target_include_directories(test_soc_spike_filter PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_compile_features(test_soc_spike_filter PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(
    test_battery_status_transition
    test/test_battery_status_transition.cpp
  )
  if(TARGET test_battery_status_transition)
    target_include_directories(test_battery_status_transition PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_compile_features(test_battery_status_transition PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(
    test_battery_state_projection
    test/test_battery_state_projection.cpp
  )
  if(TARGET test_battery_state_projection)
    target_include_directories(test_battery_state_projection PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_compile_features(test_battery_state_projection PRIVATE cxx_std_17)
    ament_target_dependencies(test_battery_state_projection sensor_msgs)
  endif()

  ament_add_gtest(
    test_modbus_response_assembler
    test/test_modbus_response_assembler.cpp
  )
  if(TARGET test_modbus_response_assembler)
    target_include_directories(test_modbus_response_assembler PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_compile_features(test_modbus_response_assembler PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(
    test_battery_register_blocks
    test/test_battery_register_blocks.cpp
  )
  if(TARGET test_battery_register_blocks)
    target_include_directories(test_battery_register_blocks PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_compile_features(test_battery_register_blocks PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(
    test_tws_battery_runtime
    test/test_tws_battery_runtime.cpp
  )
  if(TARGET test_tws_battery_runtime)
    target_include_directories(test_tws_battery_runtime PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_compile_features(test_tws_battery_runtime PRIVATE cxx_std_17)
    ament_target_dependencies(test_tws_battery_runtime
      device_manager_core
      rclcpp
      sensor_msgs
    )
    target_link_libraries(
      test_tws_battery_runtime ${PROJECT_NAME}_core "${cpp_typesupport_target}")
  endif()

  ament_add_gtest(
    test_tws_battery_runtime_plugin test/test_tws_battery_runtime_plugin.cpp)
  if(TARGET test_tws_battery_runtime_plugin)
    target_compile_features(test_tws_battery_runtime_plugin PRIVATE cxx_std_17)
    ament_target_dependencies(test_tws_battery_runtime_plugin
      device_manager_core
      device_manager_ros
      rclcpp
    )
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
