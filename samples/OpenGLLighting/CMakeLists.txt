include_directories(include)
add_executable(OpenGLLighting
	src/InfoPanel.cpp
	src/OpenGLLightingApp.cpp
)
target_link_libraries(OpenGLLighting ${CinderSamples_LINK_LIBRARIES})