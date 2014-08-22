cmake_minimum_required (VERSION 2.6)
project (MT_OpenCV_StereoTest)
# The version number.
set (MT_OpenCV_StereoTest_VERSION_MAJOR 1)
set (MT_OpenCV_StereoTest_VERSION_MINOR 0)

# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/MT_OpenCV_StereoTestConfig.h.in"
  "${PROJECT_BINARY_DIR}/MT_OpenCV_StereoTestConfig.h"
  )

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc features2d calib3d contrib gpu)

# add the binary tree to the search path for include files
# so that we will find PointCloudConfig.h
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}")
include_directories("include")
add_subdirectory(src)

install(DIRECTORY data/ DESTINATION data)
install(FILE config.cfg DESTINATION .)

# build a CPack driven installer package
# include (InstallRequiredSystemLibraries)
set(CPACK_PACKAGE_VERSION_MAJOR "${MT_OpenCV_StereoTest_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${MT_OpenCV_StereoTest_VERSION_MINOR}")
set(CPACK_GENERATOR "ZIP;TGZ")
set(CPACK_SOURCE_GENERATOR "ZIP;TGZ")
set(CPACK_PACKAGE_DIRECTORY "cpack")
include (CPack)