include_directories(include)
add_executable(NormalMapping 
	src/NormalMappingApp.cpp
)
target_link_libraries(NormalMapping ${CinderSamples_LINK_LIBRARIES})