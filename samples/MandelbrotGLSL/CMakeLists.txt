include_directories(include)
add_executable(MandelbrotGLSL 
	src/MandelbrotGLSLApp.cpp
)
target_link_libraries(MandelbrotGLSL ${CinderSamples_LINK_LIBRARIES})