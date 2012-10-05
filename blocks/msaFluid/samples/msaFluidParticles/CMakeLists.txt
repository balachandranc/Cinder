include_directories(include)
add_executable( msaFluidParticles
	src/msaFluidParticlesApp.cpp
	src/Particle.cpp
	src/ParticleSystem.cpp
)
target_link_libraries(msaFluidParticles 
	${CINDER_BLOCKS_LIB_DIR}/msaFluid/libmsafluid.a
	${CinderBlocks_LINK_LIBRARIES}
	)