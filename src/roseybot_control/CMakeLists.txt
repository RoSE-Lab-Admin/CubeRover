cmake_minimum_required(VERSION 3.8)
project(roseybot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  roseybot_arduino
  SHARED
  hardware/roseybot_system.cpp
)

target_compile_features(roseybot_arduino PUBLIC cxx_std_17)
target_include_directories(roseybot_arduino PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  roseybot_arduino PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

## add library link for libserial-dev
target_link_libraries(roseybot_arduino PUBLIC serial)

pluginlib_export_plugin_description_file(hardware_interface roseybot_arduino.xml)

# install compiled library
install(
  # note: backslash after "include" ensures contents of the folder are copied (rather than the folder itself)
  DIRECTORY hardware/include/
  DESTINATION include
)

install(TARGETS roseybot_arduino
  EXPORT export_roseybot_arduino
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  bringup description hardware
  DESTINATION share/${PROJECT_NAME}/
)

# check whether to build tests
if(BUILD_TESTING)
  # --- AUTOMATED C++ TEST DISCOVERY ---
  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB TEST_SOURCES "test/*.cpp")
  foreach(TEST_FILE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
    
    # Create the executable
    ament_add_gtest(${TEST_NAME} ${TEST_FILE})
    
    # Link to the robot library and ROS 2 core
    target_link_libraries(${TEST_NAME} roseybot_arduino)
  endforeach()

  # --- AUTOMATED PYTHON TEST DISCOVERY ---
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(all_python_tests test/)

  # --- LINTER (DISABLE FOR NOW) ---
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
endif()

# export dependencies so that donwstream packages know that they are needed
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_roseybot_arduino HAS_LIBRARY_TARGET)

ament_package()
