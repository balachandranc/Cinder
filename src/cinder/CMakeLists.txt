add_subdirectory(app)
add_subdirectory(gl)
add_library(cinder 
	Camera.cpp
	Display.cpp
	#Timer.cpp
)
target_link_libraries(cinder app gl)