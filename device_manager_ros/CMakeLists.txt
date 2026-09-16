cmake_minimum_required(VERSION 3.16)
project(device_manager_ros VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(CURL REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(device_manager_core REQUIRED)
find_package(device_manager_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}
  src/device_registration.cpp
  src/ros2_device_manager_api.cpp
  src/ros2_lifecycle_driver_adapter.cpp
  src/ros2_process_device_runtime_adapter.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME} CURL::libcurl yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}
  builtin_interfaces
  device_manager_core
  device_manager_msgs
  diagnostic_msgs
  lifecycle_msgs
  pluginlib
  rcl_interfaces
  rclcpp
  std_msgs
)

add_executable(device_manager src/main.cpp)
target_link_libraries(device_manager ${PROJECT_NAME})
ament_target_dependencies(device_manager rclcpp rclcpp_lifecycle)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS device_manager
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_library(device_manager_ros_test_runtime_plugin SHARED test/test_runtime_plugin.cpp)
  ament_target_dependencies(
    device_manager_ros_test_runtime_plugin device_manager_core pluginlib)
  pluginlib_export_plugin_description_file(
    device_manager_core test/test_runtime_plugins.xml)
  install(TARGETS device_manager_ros_test_runtime_plugin
    LIBRARY DESTINATION lib
  )
  add_executable(device_manager_ros_test_topic_source test/test_topic_source.cpp)
  ament_target_dependencies(
    device_manager_ros_test_topic_source diagnostic_msgs device_manager_msgs rclcpp std_msgs)
  install(TARGETS device_manager_ros_test_topic_source
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  ament_add_gtest(test_device_manager_ros test/test_device_manager_ros.cpp)
  ament_target_dependencies(test_device_manager_ros
    builtin_interfaces
    device_manager_core
    device_manager_msgs
    diagnostic_msgs
    lifecycle_msgs
    rcl_interfaces
    rclcpp
    rclcpp_lifecycle
    std_msgs
  )
  target_link_libraries(test_device_manager_ros ${PROJECT_NAME})

  find_program(AMENT_CPPCHECK_EXECUTABLE ament_cppcheck REQUIRED)
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/ament_cppcheck")
  ament_add_test(cppcheck
    COMMAND ${AMENT_CPPCHECK_EXECUTABLE}
      --xunit-file ${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/cppcheck.xunit.xml
      include src
      --include_dirs ${CMAKE_CURRENT_SOURCE_DIR}/include
    OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/ament_cppcheck/cppcheck.txt"
    RESULT_FILE "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/cppcheck.xunit.xml"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 300
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  CURL
  builtin_interfaces
  device_manager_core
  device_manager_msgs
  diagnostic_msgs
  lifecycle_msgs
  pluginlib
  rcl_interfaces
  rclcpp
  rclcpp_lifecycle
  std_msgs
  yaml-cpp
)
ament_package()
