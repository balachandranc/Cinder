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
	// this maintains a reference to the mgr so that we don't destroy it before
	// the last Capture is destroyed
	std::shared_ptr<class CaptureMgr>	mMgrPtr;
	bool								mIsCapturing;
	std::shared_ptr<class SurfaceCache>	mSurfaceCache;

	int32_t				mWidth, mHeight, mFormat;
	mutable uint32_t	mNumBuffers;
	mutable struct buffer *mBuffers;
	mutable Surface8u	mCurrentFrame;
	Capture::DeviceRef	mDevice;
	std::string			mName;

	static bool							sDevicesEnumerated;
	static std::vector<Capture::DeviceRef>	sDevices;

 private:
	static void errno_exit(const char*);
	static int xioctl(int, int, void*);
	void init_mmap(int, const char*, uint32_t*);
	static int open_device(const char*);
	static bool is_capture_device(int);
	void init_device(int, const char*, int*, int*, int32_t*, uint32_t*);
	void start_capturing(int, uint32_t*);
	static int read_frame(int, uint32_t*);
	void stop_capturing(int);
	void uninit_device(uint32_t*);
	void close_device(int);

	static void copy_uyvy_buffer_to( void *dest, const struct buffer buffer );
	static void copy_yuyv_buffer_to( void *dest, const struct buffer buffer );
};

} //namespace

