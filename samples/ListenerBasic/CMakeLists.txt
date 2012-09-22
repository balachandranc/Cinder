include_directories(include)
add_executable(ListenerBasic 
	src/ListenerBasicApp.cpp
	src/SquareListener.cpp
)
target_link_libraries(ListenerBasic ${CinderSamples_LINK_LIBRARIES})