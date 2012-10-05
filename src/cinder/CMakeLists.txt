add_subdirectory(app)
add_subdirectory(gl)
add_subdirectory(ip)
add_subdirectory(cairo)
add_subdirectory(params)
include_directories(${CINDER_INCLUDE_DIRS})

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

add_library(cinder 
	Camera.cpp
	Display.cpp
	Triangulate.cpp
	Area.cpp
	Path2d.cpp
	BSpline.cpp
	Exception.cpp
	CinderMath.cpp
	Rect.cpp
	TriMesh.cpp
	Stream.cpp
	Utilities.cpp
	Buffer.cpp
	DataSource.cpp
	Url.cpp
	UrlImplCurl.cpp
	AxisAlignedBox.cpp
	Ray.cpp
	ImageIo.cpp
	DataTarget.cpp
	Font.cpp
	Text.cpp
	Surface.cpp
	Channel.cpp
	Timer.cpp
	Rand.cpp
	Perlin.cpp
	Shape2d.cpp
	Capture.cpp
	CaptureImplV4l2.cpp
	Clipboard.cpp
	ImageSourcePng.cpp
	Xml.cpp
	Color.cpp
	ImageTargetFileQt.cpp
	ImageSourceFileQt.cpp
	ObjLoader.cpp
	Sphere.cpp
	Serial.cpp
	Matrix.cpp
)

target_link_libraries(cinder ${Boost_LIBRARIES})