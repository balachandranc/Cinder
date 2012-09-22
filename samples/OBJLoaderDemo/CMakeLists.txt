include_directories(include)
add_executable(OBJLoaderDemo 
	src/OBJLoaderApp.cpp
)
target_link_libraries(OBJLoaderDemo ${CinderSamples_LINK_LIBRARIES})