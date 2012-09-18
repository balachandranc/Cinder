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

#include <iostream>

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

struct buffer {
        void   *start;
        size_t  length;
};

struct buffer          *buffers;
static unsigned int     n_buffers;

static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg)
{
        int r;

        do {
                r = ioctl(fh, request, arg);
        } while (-1 == r && EINTR == errno);

        return r;
}

static void init_mmap(int fd, const char *dev_name)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count = 4;
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

        if (req.count < 2) {
                fprintf(stderr, "Insufficient buffer memory on %s\n",
                         dev_name);
                exit(EXIT_FAILURE);
        }

        buffers = (buffer *) calloc(req.count, sizeof(*buffers));

        if (!buffers) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;

                CLEAR(buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

                if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
                        errno_exit("VIDIOC_QUERYBUF");

                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start =
                        mmap(NULL /* start anywhere */,
                              buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.offset);

                if (MAP_FAILED == buffers[n_buffers].start)
                        errno_exit("mmap");
        }
}


static int open_device(const char *dev_name)
{
        struct stat st;
        int fd;

        if (-1 == stat(dev_name, &st)) {
                fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }

        if (!S_ISCHR(st.st_mode)) {
                fprintf(stderr, "%s is no device\n", dev_name);
                exit(EXIT_FAILURE);
        }

        fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        if (-1 == fd) {
                fprintf(stderr, "Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }

        return fd;
}

static void init_device(int fd, const char *dev_name)
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


                if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
                        fprintf(stderr, "%s does not support streaming i/o\n",
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


        CLEAR(fmt);

        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        int force_format = 0;
        if (force_format) {
                fmt.fmt.pix.width       = 640;
                fmt.fmt.pix.height      = 480;
                fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
                //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;
                //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

                if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
                        errno_exit("VIDIOC_S_FMT");

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

        init_mmap(fd, dev_name);
}

static void start_capturing(int fd)
{
        unsigned int i;
        enum v4l2_buf_type type;

                for (i = 0; i < n_buffers; ++i) {
                        struct v4l2_buffer buf;

                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_MMAP;
                        buf.index = i;

                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
}

static int read_frame( int fd )
{
        struct v4l2_buffer buf;
        unsigned int i;

                CLEAR(buf);

                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;

                if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
                        switch (errno) {
                        case EAGAIN:
                                return 0;

                        case EIO:
                                /* Could ignore EIO, see spec. */

                                /* fall through */

                        default:
                                errno_exit("VIDIOC_DQBUF");
                        }
                }

                assert(buf.index < n_buffers);

                //process_image(buffers[buf.index].start, buf.bytesused);

                if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                        errno_exit("VIDIOC_QBUF");

        return 1;
}

static void stop_capturing( int fd )
{
        enum v4l2_buf_type type;

                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
                        errno_exit("VIDIOC_STREAMOFF");
}

static void uninit_device(void)
{
        unsigned int i;

                for (i = 0; i < n_buffers; ++i)
                        if (-1 == munmap(buffers[i].start, buffers[i].length))
                                errno_exit("munmap");
        free(buffers);
}

static void close_device( int fd )
{
        if (-1 == close(fd))
                errno_exit("close");

        fd = -1;
}

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

	/*
	CaptureMgr::instance()->sTotalDevices = CaptureMgr::instanceVI()->listDevices( true );
	for( int i = 0; i < CaptureMgr::instance()->sTotalDevices; ++i ) {
		sDevices.push_back( Capture::DeviceRef( new CaptureImplV4l2::Device( videoInput::getDeviceName( i ), i ) ) );
	}
	*/

	sDevices.push_back( Capture::DeviceRef( new CaptureImplV4l2::Device( "/dev/video0", 0 ) ) );

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
	init_device( mDeviceID, mName.c_str() );

	mIsCapturing = false;
	mSurfaceCache = std::shared_ptr<SurfaceCache>( new SurfaceCache( mWidth, mHeight, SurfaceChannelOrder::BGR, 4 ) );

	mMgrPtr = CaptureMgr::instance();
}

CaptureImplV4l2::~CaptureImplV4l2()
{
	this->stop();
	uninit_device();
	close_device( mDeviceID );
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
	start_capturing( mDeviceID );
	mIsCapturing = true;
}

void CaptureImplV4l2::stop()
{
	if( ! mIsCapturing ) return;

	//CaptureMgr::instanceVI()->stopDevice( mDeviceID );
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
    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(mDeviceID, &fds);

    /* Timeout. */
    tv.tv_sec = 0;
    tv.tv_usec = 1;

    r = select(mDeviceID + 1, &fds, NULL, NULL, &tv);

    if ( -1 == r )
    	return false;
    else
    	return true;

	return true;
}

Surface8u CaptureImplV4l2::getSurface() const
{
	/*
	if( CaptureMgr::instanceVI()->isFrameNew( mDeviceID ) ) {
		mCurrentFrame = mSurfaceCache->getNewSurface();
		CaptureMgr::instanceVI()->getPixels( mDeviceID, mCurrentFrame.getData(), false, true );
	}
	*/
	read_frame( mDeviceID );
	mCurrentFrame = mSurfaceCache->getNewSurface();
	memcpy( mCurrentFrame.getData(), buffers[0].start, buffers[0].length );
	return mCurrentFrame;
}

} //namespace
