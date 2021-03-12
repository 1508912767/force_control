cmake_minimum_required(VERSION 2.8.3)
project(ur5_cartesian_velocity_control)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  controller_interface
  control_msgs
  forward_command_controller
  control_toolbox
  realtime_tools
  urdf
  roscpp
  kdl_parser
  kdl_conversions
  ur5_admittance_control
)
set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall -Wno-deprecated-declarations")


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ur5_cartesian_velocity_control
  DEPENDS ur5_admittance_control
)

include_directories(include ${catkin_INCLUDE_DIRS})


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ur5_cartesian_velocity_control.cpp
# )
add_library(${PROJECT_NAME}
  include/ur5_cartesian_velocity_control/kinematic_chain_controller_base.h
  include/ur5_cartesian_velocity_control/cartesian_velocity_controller.h
  include/ur5_cartesian_velocity_control/cartesian_state_controller.h
  src/cartesian_velocity_controller.cpp
  src/cartesian_state_controller.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
