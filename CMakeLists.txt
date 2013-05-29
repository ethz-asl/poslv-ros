cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  poslv-ros
  VERSION 0.1
  RELEASE alpha
  SUMMARY "ROS driver for Applanix POS LV devices."
  AUTHOR "Jerome Maye"
  CONTACT "jerome.maye@mavt.ethz.ch"
  HOME "http://github.com/jmaye/poslv_ros"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX poslv-
  INSTALL /usr
  CONFIGURATION_DESTINATION /etc/poslv
  EXTRA_CXX_FLAGS -std=c++0x
)

remake_ros_pack_deb()

remake_ros_distribute_deb(
  DISTRIBUTION lucid
  SECTION libs
  DEPENDS remake pkg-config
  UPLOAD ppa:jmaye/ros
  PASS CMAKE_BUILD_TYPE POSLV_ROS_GIT_REVISION
)

remake_ros_distribute_deb(
  DISTRIBUTION precise
  SECTION libs
  DEPENDS remake pkg-config
  UPLOAD ppa:jmaye/ros
  PASS CMAKE_BUILD_TYPE POSLV_ROS_GIT_REVISION
)
