include_directories(include)
add_executable(HodginParticles 
	src/Emitter.cpp
	src/HodginParticlesApp.cpp
	src/Particle.cpp
)
target_link_libraries(HodginParticles ${CinderSamples_LINK_LIBRARIES})