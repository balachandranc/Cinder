include_directories(include)
add_executable(SurfaceBasic 
	src/SurfaceBasicApp.cpp
)
target_link_libraries(SurfaceBasic ${CinderSamples_LINK_LIBRARIES})