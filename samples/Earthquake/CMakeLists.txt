include_directories(include)
add_executable(Earthquake 
	src/Earth.cpp
	src/EarthquakeApp.cpp
	src/POV.cpp
	src/Quake.cpp
)
target_link_libraries(Earthquake ${CinderSamples_LINK_LIBRARIES})