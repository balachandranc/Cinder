include_directories(include)
add_executable(Picking3D 
	src/Picking3DApp.cpp
)
target_link_libraries(Picking3D ${CinderSamples_LINK_LIBRARIES})