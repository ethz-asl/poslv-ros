cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

set(CMAKE_CXX_FLAGS "-std=c++0x")

# libposlv
find_path(LIBPOSLV_INCLUDE_DIR sensor/POSLVComTCP.h
  /usr/include
  PATH_SUFFIXES
  libposlv
)
message(Found ${LIBPOSLV_INCLUDE_DIR})
find_library(LIBPOSLV_SENSOR_LIBRARY poslv-sensor
  /usr/lib
)
include_directories(
  ${LIBPOSLV_INCLUDE_DIR}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/PosLvNode.cpp)
target_link_libraries(${PROJECT_NAME} ${LIBPOSLV_SENSOR_LIBRARY})
rosbuild_add_executable(poslv_node src/poslv_node.cpp)
target_link_libraries(poslv_node ${PROJECT_NAME})
