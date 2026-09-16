cmake_minimum_required(VERSION 3.8)
project(device_manager_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DeviceEvent.msg"
  "msg/TransitionRequest.msg"
  "msg/TransitionRecord.msg"
  "msg/DeviceState.msg"
  "msg/DeviceStateArray.msg"
  "srv/GetDevices.srv"
  "srv/ChangeDeviceState.srv"
  "srv/PatchDeviceParameters.srv"
  DEPENDENCIES builtin_interfaces diagnostic_msgs rcl_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
