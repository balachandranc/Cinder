
CMAKE_MINIMUM_REQUIRED(VERSION 2.8) 

PROJECT(Cinder)
SET(CMAKE_BUILD_TYPE release) 

add_definitions(-D_UNICODE)
add_definitions(-DUNICODE)
add_definitions(-D__linux__) 

find_package(GLUT)
find_package(OpenGL)

#set(Boost_USE_STATIC_RUNTIME ON)

# set cmake to use static runtime.
foreach(flag_var
        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
   if(${flag_var} MATCHES "/MD")
      string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
   endif(${flag_var} MATCHES "/MD")
endforeach(flag_var)



#incude cinder and boost header files and libraries.
set(CINDER_ROOT "./")
include_directories(
	"${CINDER_ROOT}/../include" 
	"/usr/include/boost"
	"${CINDER_ROOT}/libtess2"
)

add_subdirectory(cinder)
add_subdirectory(libtess2)