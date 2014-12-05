#UndistortedCam
cmake_minimum_required (VERSION 2.6)
project (UndistortedCam)
# The version number.
set (UndistortedCam_VERSION_MAJOR 1)
set (UndistortedCam_VERSION_MINOR 0)

set (OutputName undistortedCam)

# configure a header file to pass some of the CMake settings
# to the source code
#configure_file (
#  "${PROJECT_SOURCE_DIR}/UndistortedCam_Config.h.in"
#  "${PROJECT_BINARY_DIR}/UndistortedCam_Config.h"
#  )

find_package(Boost REQUIRED COMPONENTS system program_options date_time chrono timer)
#find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc features2d calib3d contrib gpu)

# add the binary tree to the search path for include files
# so that we will find PointCloudConfig.h
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}")
include_directories("include")
add_subdirectory(src)

install(DIRECTORY data/ DESTINATION data )#${OutputName}/data)
install(FILES config.cfg DESTINATION .)#${OutputName})
install(DIRECTORY DESTINATION output)

# build a CPack driven installer package
set(CPACK_PACKAGE_VERSION_MAJOR "${UndistortedCam_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${UndistortedCam_VERSION_MINOR}")
set(CPACK_GENERATOR "ZIP;TGZ")
set(CPACK_SOURCE_GENERATOR "ZIP;TGZ")
set(CPACK_PACKAGE_DIRECTORY "cpack")
#include(CPack)
