include_directories(include)
add_executable(RDiffusion 
	src/RDiffusionApp.cpp
)
target_link_libraries(RDiffusion ${CinderSamples_LINK_LIBRARIES})