include_directories(include)
add_executable(Triangulation
	src/TriangulationApp.cpp
)
target_link_libraries(Triangulation ${CinderSamples_LINK_LIBRARIES})