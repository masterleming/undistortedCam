#UndistortedCam
# add the executable
add_executable(${OutputName}
	StereoCam.cpp
	main.cpp
	undistorted_cam.cpp
	ConfigParser.cpp
)

include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(${OutputName} ${OpenCV_LIBS} ${Boost_LIBRARIES})

# add the install targets
install (TARGETS ${OutputName} RUNTIME DESTINATION .)#${OutputName})
#include(CPack)
