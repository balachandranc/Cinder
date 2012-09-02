add_executable(basicApp src/basicApp.cpp)
message("Cinder libraries: ${CINDER_LIBRARIES}")
message("Qt libraries: ${QT_LIBRARIES}")
message("Boost libraries: ${Boost_LIBRARIES}")
target_link_libraries(basicApp
	${OPENGL_LIBRARIES}
	${CINDER_LIBRARIES}
	${QT_LIBRARIES}
	${CURL_LIBRARIES}
	${Boost_LIBRARIES}
	)