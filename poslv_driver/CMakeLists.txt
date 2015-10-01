cmake_minimum_required(VERSION 2.8.3)
project(poslv_driver)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS -std=c++11)

## Find catkin macros and libraries
find_package(
  catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    poslv_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(POSLV libposlv REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES poslv_driver
  CATKIN_DEPENDS
    roscpp
    rospy
    poslv_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${POSLV_INCLUDE_DIRS}
)

add_library(
  poslv_driver
  src/PosLvNode.cpp
)
target_link_libraries(
  poslv_driver
  ${catkin_LIBRARIES}
  ${POSLV_LIBRARIES}
)

add_executable(
  poslv_driver_node
  src/poslv_driver_node.cpp
)
target_link_libraries(
  poslv_driver_node
  poslv_driver
)

##################
## Installation ##
##################
install(
  TARGETS
    poslv_driver
    poslv_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
  PROGRAMS
    bin/set_cmr.py
    bin/set_rtcm1.py
    bin/set_rtcm2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/poslv_driver/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*.h"
)

install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN "*.yaml"
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN "*.launch"
)
