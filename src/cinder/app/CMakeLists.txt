message("${CINDER_ROOT}/../include/cinder/app/AppImplQtBasic.h")

#QT4_WRAP_CPP(app_HEADERS_MOC /home/bchandra/workspace/Cinder/include/cinder/app/AppImplQtBasic.h)

#message("Moc files: ${app_HEADERS_MOC}")

file(COPY ../../../include/cinder/app/AppImplQtBasic.h DESTINATION ../../../src/cinder/app/)
QT4_GENERATE_MOC(AppImplQtBasic.h moc_AppImplQtBasic.cxx)

add_library( app
	App.cpp
	AppBasic.cpp
	AppImplQt.cpp
	AppImplQtBasic.cpp
	Renderer.cpp
	AppImplQtRendererGl.cpp

	#$app_HEADERS_MOC	
	moc_AppImplQtBasic.cxx
)