include_directories(include)
add_executable(TextBox 
	src/TextBoxApp.cpp
)
target_link_libraries(TextBox ${CinderSamples_LINK_LIBRARIES})