include_directories(include)
add_executable(resizeTest 
	resizeTest.cpp
)
target_link_libraries(resizeTest ${CinderSamples_LINK_LIBRARIES})