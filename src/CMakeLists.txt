
CMAKE_MINIMUM_REQUIRED(VERSION 2.8) 

PROJECT(Cinder)
SET(CMAKE_BUILD_TYPE debug) 
SET(CMAKE_C_COMPILER "/usr/bin/clang")
SET(CMAKE_CXX_COMPILER "/usr/bin/clang++")
SET(CMAKE_LINKER  "/usr/bin/llvm-ld")

add_definitions(-D_UNICODE)
add_definitions(-DUNICODE)
add_definitions(-D__linux__) 

find_package(Boost COMPONENTS system filesystem thread random)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Qt4 REQUIRED QtCore QtGui QtOpenGL)

include(${QT_USE_FILE})

#incude cinder and boost header files and libraries.
set(CINDER_ROOT "./")
include_directories(
	${CINDER_ROOT}/../include 
	/usr/include/boost
	/usr/include/cairo
	${CINDER_ROOT}/libtess2
	${CINDER_ROOT}/AntTweakBar
	${QT_INCLUDES}
)

add_subdirectory(cinder)
add_subdirectory(libtess2)
add_subdirectory(AntTweakBar)