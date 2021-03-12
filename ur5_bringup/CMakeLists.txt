cmake_minimum_required(VERSION 2.8.3)
project(ur5_bringup)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

install(DIRECTORY launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

