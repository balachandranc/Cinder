add_executable(basicApp src/basicApp.cpp)
message("${CINDER_LIBRARIES}")
target_link_libraries(basicApp
	${Boost_LIBRARIES}
	${OPENGL_LIBRARIES}
	${CINDER_LIBRARIES}
	)