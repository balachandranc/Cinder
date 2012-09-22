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

#include "cinder/ImageSourceFileQt.h"
#include "cinder/Utilities.h"

#include <QImage>
#include <QBuffer>
#include <QByteArray>
#include <QImageReader>

namespace cinder {

///////////////////////////////////////////////////////////////////////////////
// ImageSourceFileQt
ImageSourceFileQtRef ImageSourceFileQt::createFileQtRef( DataSourceRef dataSourceRef, ImageSource::Options options )
{
	return ImageSourceFileQtRef( new ImageSourceFileQt( dataSourceRef, options ) );
}

ImageSourceFileQt::ImageSourceFileQt( DataSourceRef dataSourceRef, ImageSource::Options options )
	: ImageSource()
{
	if( dataSourceRef->isFilePath() ) {
		mImage = new QImage( QString( dataSourceRef->getFilePath().c_str() ) );
	} else {
		mImage = new QImage();
		Buffer buffer = dataSourceRef->getBuffer();
		QImageReader( new QBuffer( new QByteArray( (const char *) buffer.getData(), buffer.getDataSize() ) ) ).read( mImage );
	}

	setChannelOrder( ImageIo::BGRA );
	setColorModel( ImageIo::CM_RGB );
	setDataType( ImageIo::UINT8 );

	mWidth = mImage->width();
	mHeight = mImage->height();
	mRowBytes = mWidth * channelOrderNumChannels( mChannelOrder );
}

void ImageSourceFileQt::load( ImageTargetRef target )
{
	// get a pointer to the ImageSource function appropriate for handling our data configuration
	ImageSource::RowFunc func = setupRowFunc( target );
	
	const uint8_t *data = mImage->constBits();
	for( int32_t row = 0; row < mHeight; ++row ) {
		((*this).*func)( target, row, data );
		data += mRowBytes;
	}
}

void ImageSourceFileQt::registerSelf()
{
	const int32_t SOURCE_PRIORITY = 2;
	
	ImageIoRegistrar::registerSourceGeneric( ImageSourceFileQt::createRef, SOURCE_PRIORITY );
}

} // namespace ci
