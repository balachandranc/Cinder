include_directories(include)
add_executable(SerialCommunication 
	src/SerialCommunicationApp.cpp
)
target_link_libraries(SerialCommunication ${CinderSamples_LINK_LIBRARIES})