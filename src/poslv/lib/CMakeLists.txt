remake_find_package(libposlv CONFIG)
remake_include(${LIBPOSLV_INCLUDE_DIRS})

remake_ros_package_add_library(poslv LINK ${LIBPOSLV_LIBRARIES})
