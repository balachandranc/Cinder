include_directories(include)
add_executable(FrustumCulling 
	src/FrustumCullingApp.cpp
	src/InfoPanel.cpp
)
target_link_libraries(FrustumCulling ${CinderSamples_LINK_LIBRARIES})