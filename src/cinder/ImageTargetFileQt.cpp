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

#include "cinder/ImageTargetFileQt.h"
#include "cinder/Utilities.h"

#include <QList>
#include <QByteArray>
#include <QImageWriter>
#include <QImage>
#include <QColor>

using namespace std;

namespace cinder {

void ImageTargetFileQt::registerSelf()
{
	const int32_t PRIORITY = 2;
	ImageIoRegistrar::TargetCreationFunc func = ImageTargetFileQt::createRef;
	
// Disabling GIF because we don't handle converting to paletted formats yet
//	ImageIoRegistrar::registerTargetType( "gif", func, PRIORITY, "gif" );
//	getExtensionMap()["gif"] = GUID_ContainerFormatGif;

	QList<QByteArray> formats = QImageWriter::supportedImageFormats();
	for( QList<QByteArray>::const_iterator formatIt = formats.begin(); formatIt != formats.end(); ++formatIt ) {
		ImageIoRegistrar::registerTargetType( formatIt->data(), func, PRIORITY, formatIt->data() );
	}
}

ImageTargetRef ImageTargetFileQt::createRef( DataTargetRef dataTarget, ImageSourceRef imageSource, ImageTarget::Options options, const string &extensionData )
{
	return ImageTargetRef( new ImageTargetFileQt( dataTarget, imageSource, options, extensionData ) );
}

ImageTargetFileQt::ImageTargetFileQt( DataTargetRef dataTarget, ImageSourceRef imageSource, ImageTarget::Options options, const string &extensionData )
	: ImageTarget(), mDataTarget( dataTarget )
{
	if( !dataTarget->providesFilePath() )
		throw ImageIoExceptionFailedWrite();
	imageWriter = new QImageWriter( QString( dataTarget->getFilePath().c_str() ) );

	setSize( imageSource->getWidth(), imageSource->getHeight() );
	setChannelOrder( ImageIo::BGRA );
	setColorModel( ImageIo::CM_RGB );
	setDataType( ImageIo::UINT8 );

	mRowBytes = mWidth * ImageIo::channelOrderNumChannels( ImageIo::BGRA );

	mData = shared_ptr<uint8_t>( new uint8_t[mHeight * mRowBytes], boost::checked_array_delete<uint8_t> );
	image = new QImage( (uchar *) mData.get(), mWidth, mHeight, QImage::Format_ARGB32 );
	image->fill( QColor( "white" ).rgb() );
}


void* ImageTargetFileQt::getRowPointer( int32_t row )
{
	return &mData.get()[row * mRowBytes];
}


void ImageTargetFileQt::finalize()
{
	imageWriter->write( *image );
}

} // namespace cinder
