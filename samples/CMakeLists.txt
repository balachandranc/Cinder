CMAKE_MINIMUM_REQUIRED(VERSION 2.8) 

PROJECT(CinderSamples)
SET(CMAKE_BUILD_TYPE debug) 
SET(CMAKE_C_COMPILER "/usr/bin/clang")
SET(CMAKE_CXX_COMPILER "/usr/bin/clang++")
SET(CMAKE_LINKER  "/usr/bin/llvm-ld")

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
find_package(Qt4 REQUIRED QtCore QtGui QtOpenGL) 
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
find_package(CURL)

message("Qt Definitions: ${QT_DEFINITIONS}")

if (${OPENGL_FOUND})
	message("The opengl libs: ${OPENGL_LIBRARIES}")
else()
	message("OpenGl not found")
endif()

#incude cinder and boost header files and libraries.
set(CINDER_ROOT "/home/bchandra/workspace/Cinder")
set(CINDER_INCLUDE_DIRS "${CINDER_ROOT}/include" 
	"/usr/include/boost" 
	"/usr/include"
	#"${CINDER_ROOT}/src/libtess2"
)
include_directories(${CINDER_INCLUDE_DIRS})
	
set(CINDER_LIB_DIR ${CINDER_ROOT}/build/cinder)
set(CINDER_LIBRARIES
	${CINDER_LIB_DIR}/app/libapp.a
	${CINDER_LIB_DIR}/gl/libgl.a
	${CINDER_LIB_DIR}/libcinder.a
	${CINDER_LIB_DIR}/ip/libip.a
	${CINDER_LIB_DIR}/../libtess2/libtess2.a 
	)

add_subdirectory(
	basicApp
	)
