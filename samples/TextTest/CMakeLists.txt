include_directories(include)
add_executable(TextTest
	src/TextTestApp.cpp
)
target_link_libraries(TextTest ${CinderSamples_LINK_LIBRARIES})