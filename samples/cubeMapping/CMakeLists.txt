include_directories(include)
add_executable(cubeMapping 
	src/CubeMap.cpp
	src/CubeMappingApp.cpp
)
target_link_libraries(cubeMapping ${CinderSamples_LINK_LIBRARIES})
file(COPY ../../samples/cubeMapping/shaders/CubeMapping_vert.glsl DESTINATION ./)
file(COPY ../../samples/cubeMapping/shaders/CubeMapping_frag.glsl DESTINATION ./)