cmake_minimum_required(VERSION 3.8)
project(pager100_lora_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(device_manager_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_core SHARED
  src/pager100_lora_driver.cpp
  src/pager100_lora_runtime.cpp
  src/posix_serial_transport.cpp
  src/serial_frame.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  device_manager_core
  pluginlib
  rclcpp
  std_msgs
)
pluginlib_export_plugin_description_file(
  device_manager_core pager100_lora_runtime_plugins.xml)

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
  std_msgs
)

if(BUILD_TESTING)
  find_package(device_manager_ros REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_serial_frame test/test_serial_frame.cpp)
  if(TARGET test_serial_frame)
    target_include_directories(test_serial_frame PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_compile_features(test_serial_frame PRIVATE cxx_std_17)
    target_link_libraries(test_serial_frame ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(test_pager100_lora_runtime test/test_pager100_lora_runtime.cpp)
  if(TARGET test_pager100_lora_runtime)
    target_include_directories(test_pager100_lora_runtime PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_compile_features(test_pager100_lora_runtime PRIVATE cxx_std_17)
    ament_target_dependencies(test_pager100_lora_runtime device_manager_core)
    target_link_libraries(test_pager100_lora_runtime ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(
    test_pager100_lora_runtime_plugin test/test_pager100_lora_runtime_plugin.cpp)
  if(TARGET test_pager100_lora_runtime_plugin)
    target_compile_features(test_pager100_lora_runtime_plugin PRIVATE cxx_std_17)
    ament_target_dependencies(test_pager100_lora_runtime_plugin
      device_manager_core
      device_manager_ros
      rclcpp
      std_msgs
    )
    target_link_libraries(test_pager100_lora_runtime_plugin ${PROJECT_NAME}_core)
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
