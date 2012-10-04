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

#pragma once

#include "cinder/Cinder.h"
#include "cinder/Capture.h"
#include "cinder/Surface.h"

namespace cinder {

class CaptureImplV4l2 {
 public:
	class Device;

	CaptureImplV4l2( int32_t width, int32_t height, const Capture::DeviceRef device );
	CaptureImplV4l2( int32_t width, int32_t height );
	~CaptureImplV4l2();
	
	void start();
	void stop();
	
	bool		isCapturing();
	bool		checkNewFrame() const;

	int32_t		getWidth() const { return mWidth; }
	int32_t		getHeight() const { return mHeight; }
	
	Surface8u	getSurface() const;
	
	const Capture::DeviceRef getDevice() const { return mDevice; }
	
	static const std::vector<Capture::DeviceRef>&	getDevices( bool forceRefresh = false );

	class Device : public Capture::Device {
 	  public:
		bool						checkAvailable() const;
		bool						isConnected() const;
		Capture::DeviceIdentifier	getUniqueId() const { return mUniqueId; }
		const std::string			getName() const { return mName; }

		Device( const std::string &name, int uniqueId ) : Capture::Device(), mUniqueId( uniqueId ) { mName = name; }
	 protected:
		int				mUniqueId;
	};
 protected:

	struct buffer {
	        void   *start;
	        size_t  length;
	};

	void	init( int32_t width, int32_t height, const Capture::Device &device );

	int								mDeviceID;
	int								mDeviceFD;
	// this maintains a reference to the mgr so that we don't destroy it before
	// the last Capture is destroyed
	std::shared_ptr<class CaptureMgr>	mMgrPtr;
	bool								mIsCapturing;
	std::shared_ptr<class SurfaceCache>	mSurfaceCache;

	int32_t				mWidth, mHeight, mFormat, mFrameSize;
	mutable uint32_t	mNumBuffers;
	mutable struct buffer *mBuffers;
	mutable Surface8u	mCurrentFrame;
	Capture::DeviceRef	mDevice;
	std::string			mName;

	static bool							sDevicesEnumerated;
	static std::vector<Capture::DeviceRef>	sDevices;

 private:
	static void errnoExit( const char *str );
	static int xioctl( int fh, int request, void *arg );
	void initMmap( int fd, const char *devName, uint32_t *nBuffers );
	static int openDevice( const char *devName );
	static std::string loadName( int fd );
	static bool isCaptureDevice( int fd );
	void initDevice( int fd, const char *devName, int *width, int *height, int32_t *format, uint32_t *nBuffers );
	void startCapturing( int fd, uint32_t *nBuffers );
	int readFrame( int fd, uint32_t *nBuffers ) const;
	void stopCapturing( int fd );
	void uninitDevice( uint32_t *nBuffers );
	void closeDevice( int fd );

	static void copyUyvyBufferTo( void *dest, const struct buffer buffer );
	static void copyYuyvBufferTo( void *dest, const struct buffer buffer );
};

} //namespace

