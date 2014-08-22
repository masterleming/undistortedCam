# add the executable
add_executable(undistortedCam
	StereoCam.cpp
	main.cpp
	undistorted_cam.cpp
)

include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(undistortedCam ${OpenCV_LIBS} ${Boost_LIBRARIES})

# add the install targets
install (TARGETS undistortedCam RUNTIME DESTINATION .)
#install (FILES ${OpenCV_LIBS} ${Boost_LIBRARIES} DESTINATION bin)
#install(DIRECTORY ${Boost_LIBRARY_DIR} DESTINATION bin FILES_MATCHING PATTERN "*${Boost_LIBRARIES}*")
#install(DIRECTORY ${OpenCV_LIB_DIR_OPT} DESTINATION bin FILES_MATCHING PATTERN "*${OpenCV_LIBS}*")
#install (FILES ${Boost_PROGRAM_OPTIONS_LIBRARY_DEBUG} DESTINATION bin)
#install (TARGETS OpenCV RUNTIME DESTINATION bin)
