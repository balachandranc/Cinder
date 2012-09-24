include_directories(include)
add_executable(slerpBasic 
	src/slerpBasicApp.cpp
)
target_link_libraries(slerpBasic ${CinderSamples_LINK_LIBRARIES})