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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Find LibPosLV
find_path(LIBPOSLV_INCLUDE_DIR sensor/POSLVComTCP.h
  /usr/include
  PATH_SUFFIXES
  libposlv
)

message(Found ${LIBPOSLV_INCLUDE_DIR})

find_library(LIBPOSLV_SENSOR poslv-sensor
  /usr/lib
)
find_library(LIBPOSLV_COM poslv-com
  /usr/lib
)
find_library(LIBPOSLV_TYPES poslv-types
  /usr/lib
)
find_library(LIBPOSLV_EXCEPTIONS poslv-exceptions
  /usr/lib
)

rosbuild_add_boost_directories()
include_directories(${LIBPOSLV_INCLUDE_DIR})
rosbuild_add_executable(applanix_node src/applanix_node.cpp)
target_link_libraries(applanix_node ${LIBPOSLV_SENSOR} ${LIBPOSLV_COM}
  ${LIBPOSLV_TYPES} ${LIBPOSLV_EXCEPTIONS})
