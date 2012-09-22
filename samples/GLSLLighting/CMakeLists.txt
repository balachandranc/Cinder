include_directories(include)
add_executable(GLSLLighting 
	src/GLSLLightingApp.cpp
	src/InfoPanel.cpp
)
target_link_libraries(GLSLLighting ${CinderSamples_LINK_LIBRARIES})