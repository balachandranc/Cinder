include_directories(include)
add_executable(VBOSample
	src/VBOSampleApp.cpp
)
target_link_libraries(VBOSample ${CinderSamples_LINK_LIBRARIES})