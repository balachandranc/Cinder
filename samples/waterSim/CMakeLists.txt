include_directories(include)
add_executable(waterSim
	src/grid.cpp
	src/particles.cpp
	src/waterSimApp.cpp
)
target_link_libraries(waterSim ${CinderSamples_LINK_LIBRARIES})