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
find_package(Boost COMPONENTS system filesystem thread)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Qt4 REQUIRED QtCore QtGui QtOpenGL) 
include(${QT_USE_FILE})
find_package(CURL REQUIRED)

#incude cinder and boost header files and libraries.
set(CINDER_ROOT "/home/bchandra/workspace/Cinder")
set(CINDER_INCLUDE_DIRS "${CINDER_ROOT}/include" 
	/usr/include/boost
	/usr/include/cairo 
	/usr/include
	#"${CINDER_ROOT}/src/libtess2"
)
include_directories(${CINDER_INCLUDE_DIRS})
	
set(CINDER_LIB_DIR ${CINDER_ROOT}/build/cinder)
set(CINDER_LIBRARIES
	${CINDER_LIB_DIR}/app/libapp.a
	${CINDER_LIB_DIR}/gl/libgl.a
	${CINDER_LIB_DIR}/cairo/libcairo.a
	${CINDER_LIB_DIR}/libcinder.a
	${CINDER_LIB_DIR}/ip/libip.a
	${CINDER_LIB_DIR}/params/libparams.a
	${CINDER_LIB_DIR}/../libtess2/libtess2.a
	${CINDER_LIB_DIR}/../AntTweakBar/libAntTweakBar.a
	)

set(CAIRO_LIBRARIES
	/usr/lib/i386-linux-gnu/libcairo.a
	/usr/lib/i386-linux-gnu/libpixman-1.a
	)

message("Cinder libraries: ${CINDER_LIBRARIES}")
message("Qt libraries: ${QT_LIBRARIES}")
message("Boost libraries: ${Boost_LIBRARIES}")

set(CinderSamples_LINK_LIBRARIES 
	${CINDER_LIBRARIES}
	${CAIRO_LIBRARIES}
	${OPENGL_LIBRARIES}
	${QT_LIBRARIES}
	${CURL_LIBRARIES}
	${Boost_LIBRARIES}
	/usr/lib/i386-linux-gnu/libGLEW.so.1.6
)

#add_subdirectory(basicApp)
#add_subdirectory(ArcballDemo)
#add_subdirectory(BasicParticles)
#add_subdirectory(bezierPath)
##add_subdirectory(bspline)
#add_subdirectory(Buddhabrot)
#add_subdirectory(captureTest)
#add_subdirectory(ClipboardBasic)
#add_subdirectory(cubeMapping)
#add_subdirectory(Earthquake)
#add_subdirectory(EaseGallery)
#add_subdirectory(eventTest)
#add_subdirectory(FBOBasic)
#add_subdirectory(FBOMultipleTargets)
#add_subdirectory(flickrTest)
##add_subdirectory(fontSample)
#add_subdirectory(FrustumCulling)
#add_subdirectory(GLSLLighting)
#add_subdirectory(HodginParticles)
#add_subdirectory(HodginParticlesRedux)
#add_subdirectory(ImageFileBasic)
#add_subdirectory(ImageHeightField)
#add_subdirectory(imageWriterSample)
#add_subdirectory(ListenerBasic)
#add_subdirectory(MandelbrotGLSL)
#add_subdirectory(NormalMapping)
#add_subdirectory(OBJLoaderDemo)
#add_subdirectory(OpenGLLighting)
#add_subdirectory(ParamsBasic)
##add_subdirectory(perlinTest)
#add_subdirectory(Picking3D)
#add_subdirectory(QuaternionAccum)
#add_subdirectory(rayMarcher)
#add_subdirectory(RDiffusion)
#add_subdirectory(resizeTest)
#add_subdirectory(rotatingBox)
#add_subdirectory(SaveImage)
#add_subdirectory(SerialCommunication)
#add_subdirectory(shaderTest)
#add_subdirectory(shadowMap)
#add_subdirectory(slerpBasic)
#add_subdirectory(SurfaceBasic)
#add_subdirectory(TextBox)
#add_subdirectory(TextTest)
add_subdirectory(TextureFont)