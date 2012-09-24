include_directories(include)
add_executable(SaveImage 
	src/SaveImageApp.cpp
)
target_link_libraries(SaveImage ${CinderSamples_LINK_LIBRARIES})