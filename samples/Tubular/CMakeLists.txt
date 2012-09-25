include_directories(include)
add_executable(Tubular
	src/Tube.cpp
	src/TubularApp.cpp
)
target_link_libraries(Tubular ${CinderSamples_LINK_LIBRARIES})