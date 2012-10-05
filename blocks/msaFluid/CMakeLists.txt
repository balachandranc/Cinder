include_directories(include)
add_library( msafluid
	src/ciMsaFluidDrawerGl.cpp
	src/ciMsaFluidSolver.cpp
)

add_subdirectory(samples/msaFluidBasic)
add_subdirectory(samples/msaFluidParticles)