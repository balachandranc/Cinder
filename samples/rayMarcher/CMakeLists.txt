include_directories(include)
add_executable(rayMarcher 
	src/RayMarcher.cpp
	src/rayMarcherApp.cpp
)
target_link_libraries(rayMarcher ${CinderSamples_LINK_LIBRARIES})