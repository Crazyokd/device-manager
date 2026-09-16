cmake_minimum_required(VERSION 3.8)
project(oradar_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(device_manager_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(ord_sdk STATIC
  sdk/src/lidar_address.cpp
  sdk/src/ord_driver.cpp
  sdk/src/ord_driver_impl.cpp
  sdk/src/ord_driver_net.cpp
)
target_compile_features(ord_sdk PUBLIC cxx_std_17)
set_target_properties(ord_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(ord_sdk PUBLIC ASIO_STANDALONE=1)
target_compile_options(ord_sdk PRIVATE
  -Wno-reorder
  -Wno-sign-compare
  -Wno-unused-parameter
)
target_include_directories(ord_sdk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk/third_party/Asio/asio-1.18.2/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}/sdk>
)
target_link_libraries(ord_sdk pthread)

add_library(${PROJECT_NAME}_core SHARED
  src/oradar_lidar_client.cpp
  src/oradar_lidar_runtime.cpp
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
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_core ord_sdk)
pluginlib_export_plugin_description_file(
  device_manager_core oradar_lidar_runtime_plugins.xml)

install(TARGETS ${PROJECT_NAME}_core ord_sdk
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY sdk/include/
  DESTINATION include/${PROJECT_NAME}/sdk
)

ament_export_include_directories(include)
ament_export_dependencies(
  device_manager_core
  pluginlib
  rclcpp
  sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_oradar_lidar_runtime test/test_oradar_lidar_runtime.cpp)
  if(TARGET test_oradar_lidar_runtime)
    target_include_directories(test_oradar_lidar_runtime PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk/include>
    )
    target_compile_features(test_oradar_lidar_runtime PRIVATE cxx_std_17)
    ament_target_dependencies(test_oradar_lidar_runtime device_manager_core sensor_msgs)
    target_link_libraries(test_oradar_lidar_runtime ${PROJECT_NAME}_core)
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
