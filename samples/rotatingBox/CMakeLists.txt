include_directories(include)
add_executable(rotatingBox 
	RotatingBox.cpp
)
target_link_libraries(rotatingBox ${CinderSamples_LINK_LIBRARIES})