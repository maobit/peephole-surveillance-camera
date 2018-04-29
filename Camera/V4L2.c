#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define LOG_NDEBUG 0
#define LOG_TAG "V4L2"
#include <memory.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "aw/CDX_Debug.h"

#include <fcntl.h> 
#include <sys/mman.h> 
#include <sys/select.h>
#include <sys/time.h>

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include "V4L2.h"

#define DEVICE_NAME  "/dev/video0"

#define MAX_DEVICE_NAME_LENGTH 24

typedef struct V4L2_CONTEXT{
	int mCamFd;
	int mDeviceID;
	int mBufferCnt;
	int mCaptureFormat;
	char mDeviceName[MAX_DEVICE_NAME_LENGTH];
	int width;
	int height;
	v4l2_mem_map_t mMapMem;
}V4L2_CONTEXT;


void *CreateV4l2Context()
{
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT *)malloc(sizeof(V4L2_CONTEXT));
	if(V4L2_Contect == NULL)
	{
		LOGE("V4L2_Contect == NULL");
	}

	memset(V4L2_Contect, 0, sizeof(V4L2_CONTEXT));
	//init param
	strcpy(V4L2_Contect->mDeviceName, DEVICE_NAME );
	V4L2_Contect->mDeviceID = 0;
	V4L2_Contect->mCaptureFormat = V4L2_PIX_FMT_NV12;
	//V4L2_Contect->mCaptureFormat = V4L2_PIX_FMT_YUYV;
	LOGD("V4L2 - V4L2_PIX_FMT_NV12 = %d", V4L2_PIX_FMT_NV12 );
	LOGD("V4L2 - V4L2_PIX_FMT_YUYV = %d", V4L2_PIX_FMT_YUYV );
	return (void *)V4L2_Contect;
}

void DestroyV4l2Context(void* v4l2ctx)
{
	if(!v4l2ctx)
	  return;
	
	free(v4l2ctx);
	v4l2ctx = NULL;
}


// set device node name, such as "/dev/video0"
int setV4L2DeviceName(void* v4l2ctx, const char * pname)
{
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;
	if(pname == NULL)
	{
		return -1;
	}
	strncpy(V4L2_Contect->mDeviceName, pname, strlen(pname));
	LOGV("%s: %s", __FUNCTION__, V4L2_Contect->mDeviceName);
	return 0;
}

// set different device id on the same CSI
int setV4L2DeviceID(void* v4l2ctx, int device_id)
{
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;
	V4L2_Contect->mDeviceID = device_id;
	return 0;
}

int openCameraDevice(void* v4l2ctx)
{
	int ret = -1;
	struct v4l2_input inp;
	struct v4l2_capability cap; 
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;

	// open V4L2 device
	V4L2_Contect->mCamFd = open(V4L2_Contect->mDeviceName, O_RDWR | O_NONBLOCK, 0);
	if (V4L2_Contect->mCamFd == -1) 
	{ 
        LOGE("ERROR opening %s", V4L2_Contect->mDeviceName); 
		return -1; 
	}

	inp.index = V4L2_Contect->mDeviceID;
	if (-1 == ioctl (V4L2_Contect->mCamFd, VIDIOC_S_INPUT, &inp))
	{
		LOGE("VIDIOC_S_INPUT error!");
		goto END_ERROR;
	}

	// check v4l2 device capabilities
	ret = ioctl (V4L2_Contect->mCamFd, VIDIOC_QUERYCAP, &cap); 
    if (ret < 0) 
	{ 
        LOGE("Error opening device: unable to query device."); 
        goto END_ERROR;
    } 

    if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0) 
	{ 
        LOGE("Error opening device: video capture not supported."); 
        goto END_ERROR;
    } 
  
    if ((cap.capabilities & V4L2_CAP_STREAMING) == 0) 
	{ 
        LOGE("Capture device does not support streaming i/o"); 
        goto END_ERROR;
    } 
	
	// try to support this format: NV21, YUYV
	// we do not support mjpeg camera now
	if (tryFmt(v4l2ctx, V4L2_PIX_FMT_NV12) == 0)
	{
		V4L2_Contect->mCaptureFormat = V4L2_PIX_FMT_NV12;
		LOGV("capture format: V4L2_PIX_FMT_NV12");
	}
	else if(tryFmt(v4l2ctx, V4L2_PIX_FMT_YUYV) == 0)
	{
		V4L2_Contect->mCaptureFormat = V4L2_PIX_FMT_YUYV;  // maybe usb camera
		LOGV("capture format: V4L2_PIX_FMT_YUYV");
	}
	else
	{
		LOGE("driver should surpport NV21/NV12 or YUYV format, but it does not!");
		goto END_ERROR;
	}

	LOGD("open camera device ok-------");
	return 0;

END_ERROR:

	if (V4L2_Contect->mCamFd != 0)
	{
		close(V4L2_Contect->mCamFd);
		V4L2_Contect->mCamFd = 0;
	}
	
	return -1;
}

void closeCameraDevice(void* v4l2ctx)
{
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;
	if (V4L2_Contect->mCamFd != 0)
	{
		close(V4L2_Contect->mCamFd);
		V4L2_Contect->mCamFd = 0;
	}
}

int v4l2GetCaptureFmt(void* v4l2ctx)
{
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;
	return V4L2_Contect->mCaptureFormat;
}

int v4l2SetVideoParams(void* v4l2ctx, int* width, int* height, int pix_fmt)
{
	int ret = -1;
	struct v4l2_format format;
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;

	LOGV("%s, line: %d, w: %d, h: %d, pfmt: %d", 
		__FUNCTION__, __LINE__, *width, *height, pix_fmt);

	
    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; 
    format.fmt.pix.width  = *width; 
    format.fmt.pix.height = *height; 
    if (V4L2_Contect->mCaptureFormat == V4L2_PIX_FMT_YUYV)
	{
    	format.fmt.pix.pixelformat = V4L2_Contect->mCaptureFormat; 
	}
	else
	{
		format.fmt.pix.pixelformat = pix_fmt; 
		V4L2_Contect->mCaptureFormat = pix_fmt;
	}
	
	format.fmt.pix.field = V4L2_FIELD_NONE;
	
	ret = ioctl(V4L2_Contect->mCamFd, VIDIOC_S_FMT, &format); 
	if (ret < 0) 
	{ 
		LOGE("VIDIOC_S_FMT Failed: %s", strerror(errno)); 
		return ret; 
	} 
	
	V4L2_Contect->width  = format.fmt.pix.width;
	V4L2_Contect->height = format.fmt.pix.height;

	*width = V4L2_Contect->width;
	*height = V4L2_Contect->height;
	
	LOGV("camera params: w: %d, h: %d, pfmt: %d, pfield: %d", 
		V4L2_Contect->width, V4L2_Contect->height, pix_fmt, V4L2_FIELD_NONE);

	return 0;
}

int v4l2ReqBufs(void* v4l2ctx, int* buffernum)
{
	int ret = -1;
	struct v4l2_requestbuffers rb; 
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;

	if(*buffernum > MAX_BUFFER_NUM)
	{
		LOGE("buffernum is too big");
		return -1;
	}
    V4L2_Contect->mBufferCnt = *buffernum;

	LOGV("TO VIDIOC_REQBUFS count: %d", V4L2_Contect->mBufferCnt);
	
	memset(&rb, 0, sizeof(rb));
    rb.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE; 
    rb.memory = V4L2_MEMORY_MMAP; 
    rb.count  = V4L2_Contect->mBufferCnt; 
	
	ret = ioctl(V4L2_Contect->mCamFd, VIDIOC_REQBUFS, &rb); 
    if (ret < 0) 
	{ 
        LOGE("Init: VIDIOC_REQBUFS failed: %s", strerror(errno)); 
		return ret;
    } 

	if (V4L2_Contect->mBufferCnt != (int)rb.count)
	{
		V4L2_Contect->mBufferCnt = (int)rb.count;
		*buffernum = rb.count;
		LOGD("VIDIOC_REQBUFS count: %d", V4L2_Contect->mBufferCnt);
	}

	return 0;
}


int v4l2QueryBuf(void* v4l2ctx)
{
	int ret = -1;
	int i;
	struct v4l2_buffer buf;
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;
	
	for (i = 0; i < V4L2_Contect->mBufferCnt; i++) 
	{  
        memset (&buf, 0, sizeof (struct v4l2_buffer)); 
		buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE; 
		buf.memory = V4L2_MEMORY_MMAP; 
		buf.index  = i; 
		
		ret = ioctl (V4L2_Contect->mCamFd, VIDIOC_QUERYBUF, &buf); 
        if (ret < 0) 
		{ 
            LOGE("Unable to query buffer (%s)", strerror(errno)); 
            return ret; 
        } 
 
        V4L2_Contect->mMapMem.mem[i] = mmap (0, buf.length, 
                            PROT_READ | PROT_WRITE, 
                            MAP_SHARED, 
                            V4L2_Contect->mCamFd, 
                            buf.m.offset); 
		V4L2_Contect->mMapMem.length = buf.length;
		LOGV("index: %d, mem: %x, len: %x, offset: %x", i, (int)V4L2_Contect->mMapMem.mem[i], buf.length, buf.m.offset);
 
        if (V4L2_Contect->mMapMem.mem[i] == MAP_FAILED) 
		{ 
			LOGE("Unable to map buffer (%s)", strerror(errno)); 
            return -1; 
        } 

		// start with all buffers in queue
        ret = ioctl(V4L2_Contect->mCamFd, VIDIOC_QBUF, &buf); 
        if (ret < 0) 
		{ 
            LOGE("VIDIOC_QBUF Failed"); 
            return ret; 
        } 
	} 

	return 0;
}

v4l2_mem_map_t * GetMapmemAddress(void* v4l2ctx)
{
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;
	return &V4L2_Contect->mMapMem;
}

int v4l2StartStreaming(void* v4l2ctx)
{
	int ret = -1; 
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE; 
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;
	
  	ret = ioctl (V4L2_Contect->mCamFd, VIDIOC_STREAMON, &type); 
	if (ret < 0) 
	{ 
		LOGE("StartStreaming: Unable to start capture: %s", strerror(errno)); 
		return ret; 
	} 

	LOGE("v4l2StartStreaming OK");
	return 0;
}

int v4l2StopStreaming(void* v4l2ctx)
{
	int ret = -1; 
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE; 
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;
	
	ret = ioctl (V4L2_Contect->mCamFd, VIDIOC_STREAMOFF, &type); 
	if (ret < 0) 
	{ 
		LOGE("StopStreaming: Unable to stop capture: %s", strerror(errno)); 
		return ret; 
	} 
	LOGV("V4L2Camera::v4l2StopStreaming OK");

	return 0;
}

int v4l2UnmapBuf(void* v4l2ctx)
{
	int ret = 0;
	int i;
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;
	
	for (i = 0; i < V4L2_Contect->mBufferCnt; i++) 
	{
		ret = munmap(V4L2_Contect->mMapMem.mem[i], V4L2_Contect->mMapMem.length);
        if (ret < 0) 
		{
            LOGE("v4l2CloseBuf Unmap failed"); 
			return ret;
		}
	}
	
	return 0;
}

void releasePreviewFrame(void* v4l2ctx, int index)
{
	int ret = 0;
	struct v4l2_buffer buf;
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;
	
	memset(&buf, 0, sizeof(struct v4l2_buffer));
	buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE; 
    buf.memory = V4L2_MEMORY_MMAP; 
	buf.index = index;
	
	// LOGV("r ID: %d", buf.index);
    ret = ioctl(V4L2_Contect->mCamFd, VIDIOC_QBUF, &buf); 
    if (ret != 0) 
	{
		// comment for temp, to do
         LOGE("releasePreviewFrame: VIDIOC_QBUF Failed: index = %d, ret = %d, %s", 
			buf.index, ret, strerror(errno)); 
    }
}


int v4l2WaitCameraReady(void* v4l2ctx)
{
	fd_set fds;		
	struct timeval tv;
	int r;
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;

	FD_ZERO(&fds);
	FD_SET(V4L2_Contect->mCamFd, &fds);		
	
	/* Timeout */
	tv.tv_sec  = 2;
	tv.tv_usec = 0;
	
	r = select(V4L2_Contect->mCamFd + 1, &fds, NULL, NULL, &tv);
	if (r == -1) 
	{
		LOGE("select err");
		return -1;
	} 
	else if (r == 0) 
	{
		LOGE("select timeout");
		return -1;
	}

	return 0;
}


int getPreviewFrame(void* v4l2ctx, struct v4l2_buffer *buf)
{
    int ret = 0;
    V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;
	
    buf->type   = V4L2_BUF_TYPE_VIDEO_CAPTURE; 
    buf->memory = V4L2_MEMORY_MMAP; 
 
    ret = ioctl(V4L2_Contect->mCamFd, VIDIOC_DQBUF, buf); 
    if (ret < 0) 
	{ 
        LOGW("GetPreviewFrame: VIDIOC_DQBUF Failed"); 
        return __LINE__; 			// can not return false
    }

	return 0;
}

int tryFmt(void* v4l2ctx, int format)
{	
	int i;
	struct v4l2_fmtdesc fmtdesc;
	fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;
	
	for(i = 0; i < 12; i++)
	{
		fmtdesc.index = i;
		if (-1 == ioctl (V4L2_Contect->mCamFd, VIDIOC_ENUM_FMT, &fmtdesc))
		{
			break;
		}
		LOGV("format index = %d, name = %s, v4l2 pixel format = %x\n",
			i, fmtdesc.description, fmtdesc.pixelformat);

		if (fmtdesc.pixelformat == format)
		{
			return 0;
		}
	}

	return -1;
}

int tryFmtSize(void* v4l2ctx, int * width, int * height)
{
    int ret = -1;
    struct v4l2_format fmt;
    V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;

    LOGV("V4L2Camera::TryFmtSize: w: %d, h: %d", *width, *height);

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; 
    fmt.fmt.pix.width  = *width; 
    fmt.fmt.pix.height = *height; 
    if (V4L2_Contect->mCaptureFormat == V4L2_PIX_FMT_YUYV)
    {
       fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    }
    else
    {
       fmt.fmt.pix.pixelformat = V4L2_Contect->mCaptureFormat; 
    }
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    ret = ioctl( V4L2_Contect->mCamFd, VIDIOC_TRY_FMT, &fmt ); 
    if (ret < 0) 
    { 
 	LOGE("VIDIOC_TRY_FMT Failed: %s", strerror(errno)); 
	return ret; 
    } 
    // driver surpport this size
    *width  = fmt.fmt.pix.width; 
    *height = fmt.fmt.pix.height; 
    return 0;
}

int getFrameRate(void* v4l2ctx)
{
	int ret = -1;
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;

	struct v4l2_streamparm parms;
	parms.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	ret = ioctl (V4L2_Contect->mCamFd, VIDIOC_G_PARM, &parms);
	if (ret < 0) 
	{
		LOGE("VIDIOC_G_PARM getFrameRate error\n");
		return ret;
	}

	int numerator = parms.parm.capture.timeperframe.numerator;
	int denominator = parms.parm.capture.timeperframe.denominator;
	
	LOGV("frame rate: numerator = %d, denominator = %d\n", numerator, denominator);

	return denominator / numerator;
}

int setImageEffect(void* v4l2ctx, int effect)
{
	int ret = -1;
	struct v4l2_control ctrl;
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;

	ctrl.id = V4L2_CID_COLORFX;
	ctrl.value = effect;
	ret = ioctl(V4L2_Contect->mCamFd, VIDIOC_S_CTRL, &ctrl);
	if(ret < 0) {
		LOGE("setImageEffect failed!");
	}

	return ret;
}

int setWhiteBalance(void* v4l2ctx, int wb)
{
	struct v4l2_control ctrl;
	int ret = -1;
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;

	ctrl.id = V4L2_CID_DO_WHITE_BALANCE;
	ctrl.value = wb;
	ret = ioctl(V4L2_Contect->mCamFd, VIDIOC_S_CTRL, &ctrl);
	if(ret < 0) {
		LOGE("setWhiteBalance failed!");
	}

	return ret;
}

int setExposure(void* v4l2ctx, int exp)
{
	int ret = -1;
	struct v4l2_control ctrl;
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;

	ctrl.id = V4L2_CID_EXPOSURE;
	ctrl.value = exp;
	ret = ioctl(V4L2_Contect->mCamFd, VIDIOC_S_CTRL, &ctrl);
	if(ret < 0) {
		LOGV("setExposure failed!");
	}

	return ret;
}

// flash mode
int setFlashMode(void* v4l2ctx, int mode)
{
	int ret = -1;
	struct v4l2_control ctrl;
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;

	ctrl.id = V4L2_CID_CAMERA_FLASH_MODE;
	ctrl.value = mode;
	ret = ioctl(V4L2_Contect->mCamFd, VIDIOC_S_CTRL, &ctrl);
	if(ret < 0) {
		LOGV("setFlashMode failed!");
	}

	return ret;
}

int enumSize(void* v4l2ctx, char * pSize, int len)
{
	int i;
	struct v4l2_frmsizeenum size_enum;
	char str[16];
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;

	size_enum.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	size_enum.pixel_format = V4L2_Contect->mCaptureFormat;

	if (pSize == NULL)
	{
		LOGE("error input params");
		return -1;
	}

	memset(str, 0, 16);
	memset(pSize, 0, len);
	
	for(i = 0; i < 20; i++)
	{
		size_enum.index = i;
		if (-1 == ioctl (V4L2_Contect->mCamFd, VIDIOC_ENUM_FRAMESIZES, &size_enum))
		{
			break;
		}
		// LOGV("format index = %d, size_enum: %dx%d", i, size_enum.discrete.width, size_enum.discrete.height);
		sprintf(str, "%dx%d", size_enum.discrete.width, size_enum.discrete.height);
		if (i != 0)
		{
			strcat(pSize, ",");
		}
		strcat(pSize, str);
	}

	return 0;
}

// af mode
int setAutoFocusMode(void* v4l2ctx, int af_mode)
{
	int ret = -1;
	struct v4l2_control ctrl;
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;

	ctrl.id = V4L2_CID_CAMERA_AF_MODE;
	ctrl.value = af_mode;
	ret = ioctl(V4L2_Contect->mCamFd, VIDIOC_S_CTRL, &ctrl);
	if(ret < 0) {
		LOGV("setAutoFocusMode failed!");
	}

	return ret;
}

// af ctrl
int setAutoFocusCtrl(void* v4l2ctx, int af_ctrl, void *areas)
{
	int ret = -1;
	struct v4l2_control ctrl;
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;

	ctrl.id = V4L2_CID_CAMERA_AF_CTRL;
	ctrl.value = af_ctrl;
	ctrl.user_pt = (unsigned int)areas;
	ret = ioctl(V4L2_Contect->mCamFd, VIDIOC_S_CTRL, &ctrl);
	if (ret < 0) {
		LOGE("setAutoFocusCtrl failed!");
	}
	return ret;
}

int getAutoFocusStatus(void* v4l2ctx, int af_ctrl)
{
	int ret = -1;
	struct v4l2_control ctrl;
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;

	ctrl.id = V4L2_CID_CAMERA_AF_CTRL;
	ctrl.value = af_ctrl;
	ret = ioctl(V4L2_Contect->mCamFd, VIDIOC_G_CTRL, &ctrl);
	if (ret >= 0) {
		LOGV("getAutoFocusCtrl ok");
	}

	return ret;
}

int v4l2setCaptureParams(void* v4l2ctx, struct v4l2_streamparm * params)
{
	int ret = -1;
	V4L2_CONTEXT *V4L2_Contect = (V4L2_CONTEXT*)v4l2ctx;

	ret = ioctl(V4L2_Contect->mCamFd, VIDIOC_S_PARM, params);
	if(ret < 0) {
		LOGE("v4l2setCaptureParams failed!");
	}

	return ret;
}

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

