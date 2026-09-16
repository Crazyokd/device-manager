cmake_minimum_required(VERSION 3.16)
project(device_manager_core VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME}
  src/device_manager.cpp
  src/device_manager_application.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_device_manager test/test_device_manager.cpp)
  target_link_libraries(test_device_manager ${PROJECT_NAME})

  ament_add_gtest(test_device_manager_application
    test/test_device_manager_application.cpp
    test/in_process_driver.cpp
  )
  target_link_libraries(test_device_manager_application ${PROJECT_NAME})

  ament_add_gtest(test_in_process_driver
    test/test_in_process_driver.cpp
    test/in_process_driver.cpp
  )
  target_link_libraries(test_in_process_driver ${PROJECT_NAME})

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
ament_package()
