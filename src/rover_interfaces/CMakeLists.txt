cmake_minimum_required(VERSION 3.8)
project(rover_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TelemData.msg"
  "srv/RoverCommand.srv"
  "srv/PositionCommand.srv"
  "srv/TurnCommand.srv"
  "srv/VelocityCommand.srv"
)


ament_package()
