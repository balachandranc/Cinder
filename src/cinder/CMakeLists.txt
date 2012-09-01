add_subdirectory(app)
add_subdirectory(gl)
include_directories(${CINDER_INCLUDE_DIRS})
message(" my message ${CINDER_INCLUDE_DIRS}")

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

add_library(cinder 
	Camera.cpp
	Display.cpp
	Triangulate.cpp
)
target_link_libraries(cinder app gl)