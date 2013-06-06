remake_ros_package(
  poslv
  DEPENDS roscpp rosbash diagnostic_updater std_msgs
  EXTRA_BUILD_DEPENDS libposlv-dev
  EXTRA_RUN_DEPENDS libposlv
  DESCRIPTION "Driver for Applanix POS LV devices."
)
