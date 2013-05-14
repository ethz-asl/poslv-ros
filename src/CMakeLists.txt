remake_ros_package(
  poslv
  DEPENDS roscpp diagnostic_updater
  EXTRA_BUILD_DEPENDS libposlv-dev
  EXTRA_RUN_DEPENDS libposlv
  DESCRIPTION "Driver for Applanix POS LV devices."
)
