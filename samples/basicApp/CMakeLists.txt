add_executable(basicApp src/basicApp.cpp)
message("Cinder libraries: ${CINDER_LIBRARIES}")
message("Qt libraries: ${QT_LIBRARIES}")
target_link_libraries(basicApp
	${Boost_LIBRARIES}
	${OPENGL_LIBRARIES}
	${CINDER_LIBRARIES}
	${QT_LIBRARIES}
	${CURL_LIBRARIES}
	)