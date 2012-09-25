include_directories(include)
add_executable(VoronoiGpu
	src/VoronoiGpu.cpp
	src/VoronoiGpuApp.cpp
)
target_link_libraries(VoronoiGpu ${CinderSamples_LINK_LIBRARIES})