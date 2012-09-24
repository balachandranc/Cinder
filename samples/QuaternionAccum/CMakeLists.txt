include_directories(include)
add_executable(QuaternionAccum 
	src/QuaternionAccumApp.cpp
)
target_link_libraries(QuaternionAccum ${CinderSamples_LINK_LIBRARIES})