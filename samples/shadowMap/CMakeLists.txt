include_directories(include)
add_executable(shadowMap 
	main.cpp
)
target_link_libraries(shadowMap ${CinderSamples_LINK_LIBRARIES})