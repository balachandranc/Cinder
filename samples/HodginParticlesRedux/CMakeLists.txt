include_directories(include)
add_executable(HodginParticlesRedux 
	src/Constraint.cpp
	src/Emitter.cpp
	src/Force.cpp
	src/ForceGravity.cpp
	src/ForcePerlin.cpp
	src/ForceRepulsion.cpp
	src/HodginParticlesReduxApp.cpp
	src/InfoPanel.cpp
	src/Particle.cpp
	src/ParticleController.cpp
)
target_link_libraries(HodginParticlesRedux ${CinderSamples_LINK_LIBRARIES})