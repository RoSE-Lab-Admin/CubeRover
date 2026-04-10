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
$<INSTALL_INTERFACE:include/roseybot_arduino>
)
ament_target_dependencies(
  roseybot_arduino PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

## add library link for libserial-dev
target_link_libraries(roseybot_arduino PUBLIC serial)

pluginlib_export_plugin_description_file(hardware_interface roseybot_arduino.xml)

#install compiled library
install(
  DIRECTORY hardware/include
  DESTINATION include/roseybot_arduino
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


ament_package()
