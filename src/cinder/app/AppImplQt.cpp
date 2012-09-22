/*
 Copyright (c) 2010, The Barbarian Group
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and
	the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
	the following disclaimer in the documentation and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/

#include "cinder/app/AppImplQt.h"
#include "cinder/app/App.h"
#include "cinder/Utilities.h"

#include <QFile>
#include <QApplication>
#include <QDir>
#include <QFileDialog>

using std::string;
using std::wstring;
using std::vector;
using std::pair;

namespace cinder { namespace app {

QApplication* AppImplQt::sQApp;

AppImplQt::AppImplQt( App *aApp )
	: mApp( aApp ), mWindowOffset( 0, 0 )
{
	/*
	Gdiplus::GdiplusStartupInput gdiplusStartupInput;
	Gdiplus::GdiplusStartup( &mGdiplusToken, &gdiplusStartupInput, NULL );
	*/
}

AppImplQt::~AppImplQt()
{
	// there's no way to ensure all GDI+ objects have been freed, so we don't do this
	// for now this seems fine
//	Gdiplus::GdiplusShutdown( mGdiplusToken );
}

void AppImplQt::hideCursor()
{
	//::ShowCursor( FALSE );
}

void AppImplQt::showCursor()
{
	//::ShowCursor( TRUE );
}

Vec2i AppImplQt::mouseLocation()
{
	/*
	POINT point;
	::GetCursorPos( &point );
	return Vec2i( point.x - mWindowOffset.x, point.y - mWindowOffset.y );\
	*/
	QPoint position = QCursor::pos();
	return Vec2i( position.x(), position.y() );
}

Buffer AppImplQt::loadResource( const std::string &relativePath )
{
	QString path = QCoreApplication::applicationDirPath() + QDir::separator() + QString( relativePath.c_str() );
	QFile file( path );
	file.open( QIODevice::ReadOnly );
	char * dataPtr = new char[ file.size() ];
	memcpy( dataPtr, file.readAll().data(), file.size() );
	return Buffer( (void *) dataPtr, (size_t) file.size() );
}

fs::path AppImplQt::getAppPath()
{
	return fs::path( QCoreApplication::applicationFilePath().toStdString() );
}

fs::path AppImplQt::getOpenFilePath( const fs::path &initialPath, vector<string> extensions )
{
	QStringList filterString( "File Types (");
	for( vector<string>::const_iterator extension = extensions.begin(); extension != extensions.end(); ++extension )
		filterString.append( ( string( "*." ) + *extension ).c_str() );
	filterString.append( ")");
	QString fileName = QFileDialog::getOpenFileName( NULL, "Open File", initialPath.c_str(), filterString.join( " ") );
	return fs::path( fileName.toStdString() );
}

fs::path AppImplQt::getFolderPath( const fs::path &initialPath )
{
	return fs::path( QFileDialog::getExistingDirectory( NULL, "Open Directory", initialPath.c_str() ).toStdString() );
}

fs::path AppImplQt::getSaveFilePath( const fs::path &initialPath, vector<string> extensions )
{
	QStringList filterString( "File Types (");
	for( vector<string>::const_iterator extension = extensions.begin(); extension != extensions.end(); ++extension )
		filterString.append( ( string( "*." ) + *extension ).c_str() );
	filterString.append( ")");
	QString fileName = QFileDialog::getSaveFileName( NULL, "Save File", initialPath.c_str(), filterString.join( " ") );
	return fs::path( fileName.toStdString() );
}

void AppImplQt::prepareLaunch()
{
	int argc = 0;
	char **argv;
	sQApp = new QApplication( argc, argv );
}

} } // namespace cinder::app
