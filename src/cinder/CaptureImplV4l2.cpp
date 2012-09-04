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

#include "cinder/CaptureImplV4l2.h"
#include <boost/noncopyable.hpp>

#include <set>
using namespace std;


namespace cinder {

bool CaptureImplV4l2::sDevicesEnumerated = false;
vector<Capture::DeviceRef> CaptureImplV4l2::sDevices;

class CaptureMgr : private boost::noncopyable
{
 public:
	CaptureMgr();
	~CaptureMgr();

	static std::shared_ptr<CaptureMgr>	instance();
	//static videoInput*	instanceVI() { return instance()->mVideoInput; }

	static std::shared_ptr<CaptureMgr>	sInstance;
	static int						sTotalDevices;
	
 private:	
	//videoInput			*mVideoInput;
};
std::shared_ptr<CaptureMgr>	CaptureMgr::sInstance;
int							CaptureMgr::sTotalDevices = 0;

CaptureMgr::CaptureMgr()
{
	/*
	mVideoInput = new videoInput;
	mVideoInput->setUseCallback( true );
	*/
}

CaptureMgr::~CaptureMgr()
{
	//delete mVideoInput;
}

std::shared_ptr<CaptureMgr> CaptureMgr::instance()
{
	if( ! sInstance ) {
		sInstance = std::shared_ptr<CaptureMgr>( new CaptureMgr );
	}
	return sInstance;
}

class SurfaceCache {
 public:
	SurfaceCache( int32_t width, int32_t height, SurfaceChannelOrder sco, int numSurfaces )
		: mWidth( width ), mHeight( height ), mSCO( sco )
	{
		for( int i = 0; i < numSurfaces; ++i ) {
			mSurfaceData.push_back( std::shared_ptr<uint8_t>( new uint8_t[width*height*sco.getPixelInc()], checked_array_deleter<uint8_t>() ) );
			mDeallocatorRefcon.push_back( make_pair( this, i ) );
			mSurfaceUsed.push_back( false );
		}
	}
	
	Surface8u getNewSurface()
	{
		// try to find an available block of pixel data to wrap a surface around	
		for( size_t i = 0; i < mSurfaceData.size(); ++i ) {
			if( ! mSurfaceUsed[i] ) {
				mSurfaceUsed[i] = true;
				Surface8u result( mSurfaceData[i].get(), mWidth, mHeight, mWidth * mSCO.getPixelInc(), mSCO );
				result.setDeallocator( surfaceDeallocator, &mDeallocatorRefcon[i] );
				return result;
			}
		}

		// we couldn't find an available surface, so we'll need to allocate one
		return Surface8u( mWidth, mHeight, mSCO.hasAlpha(), mSCO );
	}
	
	static void surfaceDeallocator( void *refcon )
	{
		pair<SurfaceCache*,int> *info = reinterpret_cast<pair<SurfaceCache*,int>*>( refcon );
		info->first->mSurfaceUsed[info->second] = false;
	}

 private:
	vector<std::shared_ptr<uint8_t> >	mSurfaceData;
	vector<bool>					mSurfaceUsed;
	vector<pair<SurfaceCache*,int> >	mDeallocatorRefcon;
	int32_t				mWidth, mHeight;
	SurfaceChannelOrder	mSCO;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CaptureImplV4l2

bool CaptureImplV4l2::Device::checkAvailable() const
{
	//return ( mUniqueId < CaptureMgr::sTotalDevices ) && ( ! CaptureMgr::instanceVI()->isDeviceSetup( mUniqueId ) );
}

bool CaptureImplV4l2::Device::isConnected() const
{
	//return CaptureMgr::instanceVI()->isDeviceConnected( mUniqueId );
}

const vector<Capture::DeviceRef>& CaptureImplV4l2::getDevices( bool forceRefresh )
{
	if( sDevicesEnumerated && ( ! forceRefresh ) )
		return sDevices;

	sDevices.clear();

	//CaptureMgr::instance()->sTotalDevices = CaptureMgr::instanceVI()->listDevices( true );
	for( int i = 0; i < CaptureMgr::instance()->sTotalDevices; ++i ) {
		//sDevices.push_back( Capture::DeviceRef( new CaptureImplV4l2::Device( videoInput::getDeviceName( i ), i ) ) );
	}

	sDevicesEnumerated = true;
	return sDevices;
}

CaptureImplV4l2::CaptureImplV4l2( int32_t width, int32_t height, const Capture::DeviceRef device )
	: mWidth( width ), mHeight( height ), mCurrentFrame( width, height, false, SurfaceChannelOrder::BGR ), mDeviceID( 0 )
{
	mDevice = device;
	if( mDevice ) {
		mDeviceID = device->getUniqueId();
	}
	/*
	if( ! CaptureMgr::instanceVI()->setupDevice( mDeviceID, mWidth, mHeight ) )
		throw CaptureExcInitFail();
	mWidth = CaptureMgr::instanceVI()->getWidth( mDeviceID );
	mHeight = CaptureMgr::instanceVI()->getHeight( mDeviceID );
	*/
	mIsCapturing = true;
	mSurfaceCache = std::shared_ptr<SurfaceCache>( new SurfaceCache( mWidth, mHeight, SurfaceChannelOrder::BGR, 4 ) );

	mMgrPtr = CaptureMgr::instance();
}

CaptureImplV4l2::~CaptureImplV4l2()
{
	//CaptureMgr::instanceVI()->stopDevice( mDeviceID );
}

void CaptureImplV4l2::start()
{
	if( mIsCapturing ) return;
	/*
	if( ! CaptureMgr::instanceVI()->setupDevice( mDeviceID, mWidth, mHeight ) )
		throw CaptureExcInitFail();
	if( ! CaptureMgr::instanceVI()->isDeviceSetup( mDeviceID ) )
		throw CaptureExcInitFail();
	mWidth = CaptureMgr::instanceVI()->getWidth( mDeviceID );
	mHeight = CaptureMgr::instanceVI()->getHeight( mDeviceID );
	*/
	mIsCapturing = true;
}

void CaptureImplV4l2::stop()
{
	if( ! mIsCapturing ) return;

	//CaptureMgr::instanceVI()->stopDevice( mDeviceID );
	mIsCapturing = false;
}

bool CaptureImplV4l2::isCapturing()
{
	return mIsCapturing;
}

bool CaptureImplV4l2::checkNewFrame() const
{
	//return CaptureMgr::instanceVI()->isFrameNew( mDeviceID );
}

Surface8u CaptureImplV4l2::getSurface() const
{
	/*
	if( CaptureMgr::instanceVI()->isFrameNew( mDeviceID ) ) {
		mCurrentFrame = mSurfaceCache->getNewSurface();
		CaptureMgr::instanceVI()->getPixels( mDeviceID, mCurrentFrame.getData(), false, true );
	}
	*/
	return mCurrentFrame;
}

} //namespace
