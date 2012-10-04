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

#include <stdio.h>

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define MAX_VIDEO_DEVICES 5


#include <iostream>
#include <sstream>

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
// V4l2 Capture

void CaptureImplV4l2::errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

int CaptureImplV4l2::xioctl(int fh, int request, void *arg)
{
        int r;

        do {
                r = ioctl(fh, request, arg);
        } while (-1 == r && EINTR == errno);

        return r;
}

void CaptureImplV4l2::init_mmap(int fd, const char *dev_name, uint32_t *n_buffers)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        //req.count changed from 4 to 1.
        //req.count = 4;

        req.count = 1;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "%s does not support "
                                 "memory mapping\n", dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 1) {
                fprintf(stderr, "Insufficient buffer memory on %s\n",
                         dev_name);
                exit(EXIT_FAILURE);
        }

        mBuffers = (buffer *) calloc(req.count, sizeof(*mBuffers));

        if (!mBuffers) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (*n_buffers = 0; *n_buffers < req.count; ++(*n_buffers) ) {
                struct v4l2_buffer buf;

                CLEAR(buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = *n_buffers;

                if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
                        errno_exit("VIDIOC_QUERYBUF");

                mBuffers[*n_buffers].length = buf.length;
                mBuffers[*n_buffers].start =
                        mmap(NULL /* start anywhere */,
                              buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.offset);

                if (MAP_FAILED == mBuffers[*n_buffers].start)
                        errno_exit("mmap");
        }
}


int CaptureImplV4l2::open_device(const char *dev_name)
{
        struct stat st;
        int fd;

        if (-1 == stat(dev_name, &st)) {
                fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                return -1;
        }

        if (!S_ISCHR(st.st_mode)) {
                fprintf(stderr, "%s is no device\n", dev_name);
                return -1;
        }

        fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        if (-1 == fd) {
                fprintf(stderr, "Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                return -1;
        }

        return fd;
}

bool CaptureImplV4l2::is_capture_device( int fd )
{
    struct v4l2_capability cap;

    if( -1 == xioctl( fd, VIDIOC_QUERYCAP, &cap ) ) {
    	return false;
    }

    if( !( cap.capabilities & V4L2_CAP_VIDEO_CAPTURE ) ) {
    	return false;
    }

    return true;
}

void CaptureImplV4l2::init_device( int fd, const char *dev_name, int *width, int *height, int32_t *format, uint32_t *n_buffers )
{
        struct v4l2_capability cap;
        struct v4l2_cropcap cropcap;
        struct v4l2_crop crop;
        struct v4l2_format fmt;
        unsigned int min;

        if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "%s is no V4L2 device\n",
                                 dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_QUERYCAP");
                }
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
                fprintf(stderr, "%s is no video capture device\n",
                         dev_name);
                exit(EXIT_FAILURE);
        }


        /* Select video input, video standard and tune here. */

        CLEAR(cropcap);

        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
                crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                crop.c = cropcap.defrect; /* reset to default */

                if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
                        switch (errno) {
                        case EINVAL:
                                /* Cropping not supported. */
                                break;
                        default:
                                /* Errors ignored. */
                                break;
                        }
                }
        } else {
                /* Errors ignored. */
        }


        CLEAR( fmt );

        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        /*
        struct v4l2_fmtdesc enum_fmt;
        enum_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        for( int i = 0;; ++i ) {
        	enum_fmt.index = i;
        	if ( -1 == xioctl( fd, VIDIOC_ENUM_FMT, &enum_fmt ) )
        		break;
        }
         */

        int force_format = 1;
        if (force_format) {
                fmt.fmt.pix.width       = *width;
                fmt.fmt.pix.height      = *height;
                fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
                //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
                fmt.fmt.pix.field       = V4L2_FIELD_NONE;

                if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
                        errno_exit("VIDIOC_S_FMT");


                if( fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV && fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_UYVY ) {
                    fmt.fmt.pix.width       = *width;
                    fmt.fmt.pix.height      = *height;
                    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
                    fmt.fmt.pix.field       = V4L2_FIELD_NONE;

                    if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
                            errno_exit("VIDIOC_S_FMT");
                }

                if( fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV &&
                		fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_UYVY &&
                		fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_BGR24 ) {
                    fmt.fmt.pix.width       = *width;
                    fmt.fmt.pix.height      = *height;
                    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR32;
                    fmt.fmt.pix.field       = V4L2_FIELD_NONE;

                    if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
                            errno_exit("VIDIOC_S_FMT");
                }

                *width = fmt.fmt.pix.width;
                *height = fmt.fmt.pix.height;
                *format = fmt.fmt.pix.pixelformat;

                /* Note VIDIOC_S_FMT may change width and height. */
        } else {
                /* Preserve original settings as set by v4l2-ctl for example */
                if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
                        errno_exit("VIDIOC_G_FMT");
        }

        /* Buggy driver paranoia. */
        min = fmt.fmt.pix.width * 2;
        if (fmt.fmt.pix.bytesperline < min)
                fmt.fmt.pix.bytesperline = min;
        min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
        if (fmt.fmt.pix.sizeimage < min)
                fmt.fmt.pix.sizeimage = min;

        init_mmap( fd, dev_name, n_buffers );
}

void CaptureImplV4l2::start_capturing( int fd, uint32_t *n_buffers )
{
        enum v4l2_buf_type type;

        for( int i = 0; i < *n_buffers; ++i ) {
        	struct v4l2_buffer buf;

            CLEAR( buf );
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if( -1 == xioctl( fd, VIDIOC_QBUF, &buf ) )
            	errno_exit( "VIDIOC_QBUF" );
        }

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if( -1 == xioctl( fd, VIDIOC_STREAMON, &type ) )
        	errno_exit( "VIDIOC_STREAMON" );
}

int CaptureImplV4l2::read_frame( int fd, uint32_t *n_buffers )
{
        struct v4l2_buffer buf;

        CLEAR( buf );

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if( -1 == xioctl( fd, VIDIOC_DQBUF, &buf ) ) {
        	switch( errno ) {
            	case EAGAIN:
            		return 0;

            	case EIO:
                    /* Could ignore EIO, see spec. */

                    /* fall through */

                default:
                	errno_exit("VIDIOC_DQBUF");
        	}
        }

        assert( buf.index < *n_buffers );

        if( -1 == xioctl( fd, VIDIOC_QBUF, &buf ) )
        	errno_exit( "VIDIOC_QBUF" );

        return 1;
}

void CaptureImplV4l2::stop_capturing( int fd )
{
        enum v4l2_buf_type type;

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if( -1 == xioctl( fd, VIDIOC_STREAMOFF, &type ) )
        	errno_exit( "VIDIOC_STREAMOFF" );
}

void CaptureImplV4l2::uninit_device(  uint32_t *n_buffers )
{
        for( int i = 0; i < *n_buffers; ++i)
        	if( -1 == munmap( mBuffers[i].start, mBuffers[i].length ) )
        		errno_exit( "munmap" );

        free( mBuffers );
}

void CaptureImplV4l2::close_device( int fd )
{
        if (-1 == close(fd))
                errno_exit( "close" );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CaptureImplV4l2

bool CaptureImplV4l2::Device::checkAvailable() const
{
	//return ( mUniqueId < CaptureMgr::sTotalDevices ) && ( ! CaptureMgr::instanceVI()->isDeviceSetup( mUniqueId ) );
	return true;
}

bool CaptureImplV4l2::Device::isConnected() const
{
	//return CaptureMgr::instanceVI()->isDeviceConnected( mUniqueId );
	return true;
}

const vector<Capture::DeviceRef>& CaptureImplV4l2::getDevices( bool forceRefresh )
{
	if( sDevicesEnumerated && ( ! forceRefresh ) )
		return sDevices;

	sDevices.clear();

	for( int i = 0; i < MAX_VIDEO_DEVICES; ++i ) {
		std::stringstream ss;
		ss << "/dev/video" << i;
		std::string deviceName = ss.str();

		int fd = open_device( deviceName.c_str() );
		if( fd == -1 )
			continue;
		if( is_capture_device( fd ) ) {
			close( fd );
			sDevices.push_back( Capture::DeviceRef( new CaptureImplV4l2::Device( deviceName, i ) ) );
		}
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
	if( mDevice ) {
		mName = device->getName();
	} else {
		mName = "/dev/video0";
	}

	mDeviceID = open_device( mName.c_str() );
	init_device( mDeviceID, mName.c_str(), &mWidth, &mHeight, &mFormat, &mNumBuffers );

	mIsCapturing = false;
	mSurfaceCache = std::shared_ptr<SurfaceCache>( new SurfaceCache( mWidth, mHeight, SurfaceChannelOrder::BGR, 4 ) );

	mMgrPtr = CaptureMgr::instance();
}

CaptureImplV4l2::~CaptureImplV4l2()
{
	this->stop();
	uninit_device( &mNumBuffers );
	close_device( mDeviceID );
}

void CaptureImplV4l2::start()
{
	if( mIsCapturing )
		return;
	/*
	if( ! CaptureMgr::instanceVI()->setupDevice( mDeviceID, mWidth, mHeight ) )
		throw CaptureExcInitFail();
	if( ! CaptureMgr::instanceVI()->isDeviceSetup( mDeviceID ) )
		throw CaptureExcInitFail();
	mWidth = CaptureMgr::instanceVI()->getWidth( mDeviceID );
	mHeight = CaptureMgr::instanceVI()->getHeight( mDeviceID );
	*/
	start_capturing( mDeviceID, &mNumBuffers );
	mIsCapturing = true;
}

void CaptureImplV4l2::stop()
{
	if( ! mIsCapturing )
		return;

	stop_capturing( mDeviceID );
	mIsCapturing = false;
}

bool CaptureImplV4l2::isCapturing()
{
	return mIsCapturing;
}

bool CaptureImplV4l2::checkNewFrame() const
{
	//return CaptureMgr::instanceVI()->isFrameNew( mDeviceID );

	/*
    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(mDeviceID, &fds);

    tv.tv_sec = 0;
    tv.tv_usec = 1;

    r = select(mDeviceID + 1, &fds, NULL, NULL, &tv);

    if ( -1 == r )
    	return false;
    else
    	return true;
	 */
	return true;
}

void CaptureImplV4l2::copy_uyvy_buffer_to( void *dest, const struct buffer buffer )
{
	int stride = 4;
	int numFrames = buffer.length / stride;
	unsigned char *src = (unsigned char *) buffer.start;
	unsigned char *dst = (unsigned char *) dest;

	for( int i = 0; i < numFrames; i++ ) {
		int dstPos = 6 * i;
		int srcPos = stride * i;

		float u  = ( src[ srcPos + 0 ] - 128.0 );
		float v  = ( src[ srcPos + 2 ] - 128.0 );

		float y1 = 1.164 * ( src[ srcPos + 1 ] - 16 );
		float y2 = 1.164 * ( src[ srcPos + 3 ] - 16 );

		float rc = 1.596 * v, gc = - 0.813 * v - 0.391 * u, bc = 2.018 * u;

		float r1, g1, b1, r2, g2, b2;

		b1 = y1 + bc;
		g1 = y1 + gc;
		r1 = y1 + rc;

		b1 = (b1 < 0) ? 0 : (b1 > 255) ? 255: b1;
		g1 = (g1 < 0) ? 0 : (g1 > 255) ? 255: g1;
		r1 = (r1 < 0) ? 0 : (r1 > 255) ? 255: r1;

		dst[ dstPos + 0 ] = b1;
		dst[ dstPos + 1 ] = g1;
		dst[ dstPos + 2 ] = r1;

		b2 = y2 + bc;
		g2 = y2 + gc;
		r2 = y2 + rc;

		b2 = (b2 < 0) ? 0 : (b2 > 255) ? 255: b2;
		g2 = (g2 < 0) ? 0 : (g2 > 255) ? 255: g2;
		r2 = (r2 < 0) ? 0 : (r2 > 255) ? 255: r2;

		dst[ dstPos + 3 ] = b2;
		dst[ dstPos + 4 ] = g2;
		dst[ dstPos + 5 ] = r2;
	}
}


void CaptureImplV4l2::copy_yuyv_buffer_to( void *dest, const struct buffer buffer )
{
	int stride = 4;
	int numFrames = buffer.length / stride;
	unsigned char *src = (unsigned char *) buffer.start;
	unsigned char *dst = (unsigned char *) dest;

	for( int i = 0; i < numFrames; i++ ) {
		int dstPos = 6 * i;
		int srcPos = stride * i;

		float u  = ( src[ srcPos + 1 ] - 128.0 );
		float v  = ( src[ srcPos + 3 ] - 128.0 );

		float y1 = 1.164 * ( src[ srcPos + 0 ] - 16 );
		float y2 = 1.164 * ( src[ srcPos + 2 ] - 16 );

		float rc = 1.596 * v, gc = - 0.813 * v - 0.391 * u, bc = 2.018 * u;

		float r1, g1, b1, r2, g2, b2;

		b1 = y1 + bc;
		g1 = y1 + gc;
		r1 = y1 + rc;

		b1 = (b1 < 0) ? 0 : (b1 > 255) ? 255: b1;
		g1 = (g1 < 0) ? 0 : (g1 > 255) ? 255: g1;
		r1 = (r1 < 0) ? 0 : (r1 > 255) ? 255: r1;

		dst[ dstPos + 0 ] = b1;
		dst[ dstPos + 1 ] = g1;
		dst[ dstPos + 2 ] = r1;

		b2 = y2 + bc;
		g2 = y2 + gc;
		r2 = y2 + rc;

		b2 = (b2 < 0) ? 0 : (b2 > 255) ? 255: b2;
		g2 = (g2 < 0) ? 0 : (g2 > 255) ? 255: g2;
		r2 = (r2 < 0) ? 0 : (r2 > 255) ? 255: r2;

		dst[ dstPos + 3 ] = b2;
		dst[ dstPos + 4 ] = g2;
		dst[ dstPos + 5 ] = r2;
	}
}

Surface8u CaptureImplV4l2::getSurface() const
{
	/*
	if( CaptureMgr::instanceVI()->isFrameNew( mDeviceID ) ) {
		mCurrentFrame = mSurfaceCache->getNewSurface();
		CaptureMgr::instanceVI()->getPixels( mDeviceID, mCurrentFrame.getData(), false, true );
	}
	*/
	read_frame( mDeviceID, &mNumBuffers );
	mCurrentFrame = mSurfaceCache->getNewSurface();

	switch( mFormat ) {

	case V4L2_PIX_FMT_YUYV:
		copy_yuyv_buffer_to( mCurrentFrame.getData(), mBuffers[0]);
		break;

	case V4L2_PIX_FMT_UYVY:
		copy_uyvy_buffer_to( mCurrentFrame.getData(), mBuffers[0]);
		break;

	default:
		break;
	}

	return mCurrentFrame;
}

} //namespace
