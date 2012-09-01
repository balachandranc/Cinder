CMAKE_MINIMUM_REQUIRED(VERSION 2.8) 

PROJECT(CinderSamples)
SET(CMAKE_BUILD_TYPE release) 
#SET(CMAKE_C_COMPILER "/usr/bin/clang")
#SET(CMAKE_CXX_COMPILER "/usr/bin/clang++")
#SET (CMAKE_LINKER  "/usr/bin/llvm-ld")

add_definitions(-D_UNICODE)
add_definitions(-DUNICODE)
add_definitions(-D__linux__) 

set(Boost_USE_STATIC_LIBS 		ON)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
find_package(Boost COMPONENTS system filesystem)

if (${Boost_FOUND})
	message("${Boost_LIBRARIES}")
endif()

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

if (${OPENGL_FOUND})
	message("The opengl libs: ${OPENGL_LIBRARIES}")
else()
	message("OpenGl not found")
endif()

#incude cinder and boost header files and libraries.
set(CINDER_ROOT "/home/bchandra/workspace/Cinder")
include_directories("${CINDER_ROOT}/include" "/usr/include/boost" "/usr/include")
set(CINDER_LIB_DIR ${CINDER_ROOT}/build/cinder)
set(CINDER_LIBRARIES 
	${CINDER_LIB_DIR}/libcinder.a
	${CINDER_LIB_DIR}/app/libapp.a
	${CINDER_LIB_DIR}/gl/libgl.a
	)

add_subdirectory(
	basicApp
	)
