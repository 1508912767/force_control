cmake_minimum_required(VERSION 2.8.2)
project(ati_sensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  std_msgs
  geometry_msgs
  rospy
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs std_msgs
    INCLUDE_DIRS include
    LIBRARIES ati_sensor
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

add_library(ati_sensor src/ft_sensor.cpp)

find_package(LibXml2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})
target_link_libraries(ati_sensor ${LIBXML2_LIBRARIES} ${XENOMAI_RTDM_LIBRARIES})

add_executable(ft_sensor_node src/ft_sensor_node.cpp)
target_link_libraries(ft_sensor_node ati_sensor ${catkin_LIBRARIES})

