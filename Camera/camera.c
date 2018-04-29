#define LOG_NDEBUG 0
#define LOG_TAG "Camera"
#include "aw/CDX_Debug.h"

#include <stdio.h>
#include <memory.h>

#include "V4L2.h"
#include "camera.h"

typedef struct CameraParm
{
	int deviceid;
	int framerate;
	int width;
	int height;
}CameraParm;


int CameraGetOneframe(void* v4l2ctx, struct v4l2_buffer *buffer)
{
	/* Wait till FPS timeout expires, or thread exit message is received. */
	int ret = v4l2WaitCameraReady(v4l2ctx);
	if (ret != 0)
	{
		LOGW("wait v4l2 buffer time out");
		return __LINE__;
	}
	
	// get one video frame
	struct v4l2_buffer buf;
	memset(&buf, 0, sizeof(buf));
	ret = getPreviewFrame(v4l2ctx,&buf);
	if (ret != 0)
	{
		return ret;
	}

	memcpy(buffer, &buf, sizeof(buf));
	return 0;
}

void CameraReturnOneframe(void* v4l2ctx, int id)
{
	releasePreviewFrame(v4l2ctx, id);
	return;
}

void *CreateCameraContext()
{
	return CreateV4l2Context();
}

void DestroyCameraContext(void* v4l2ctx)
{

	DestroyV4l2Context(v4l2ctx);
}

int OpenCamera(void* v4l2ctx)
{
	return openCameraDevice(v4l2ctx);
}

void CloseCamera(void* v4l2ctx)
{
	closeCameraDevice(v4l2ctx);
}

int StartCamera(void* v4l2ctx, int *width, int *height)
{
	// int pix_fmt = V4L2_PIX_FMT_NV12;
	// int pix_fmt = V4L2_PIX_FMT_YUYV;

	// Pixel format that should work with direct CSI interface camera
	// https://bananapi.gitbooks.io/bpi-accessories/content/zh/bpi-m2+camera.html
	int pix_fmt = V4L2_PIX_FMT_YUV420;

	int mframerate = 30;
	int mbuffernuber = BUFFER_NUMBER;

	// set capture mode
	struct v4l2_streamparm params;
  	params.parm.capture.timeperframe.numerator = 1;
	params.parm.capture.timeperframe.denominator = mframerate;
	params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	params.parm.capture.capturemode = V4L2_MODE_VIDEO;

	v4l2setCaptureParams(v4l2ctx, &params);

	// set v4l2 device parameters
	v4l2SetVideoParams(v4l2ctx, width, height, pix_fmt);

	// set fps
	v4l2setCaptureParams(v4l2ctx,&params);
	
	// v4l2 request buffers
	v4l2ReqBufs(v4l2ctx, &mbuffernuber);

	// v4l2 query buffers
	v4l2QueryBuf(v4l2ctx);
	
	// stream on the v4l2 device
	v4l2StartStreaming(v4l2ctx);

    return 0;
}

int StopCamera(void* v4l2ctx)
{
	LOGV("stopCamera");
	
	// v4l2 device stop stream
	v4l2StopStreaming(v4l2ctx);

	// v4l2 device unmap buffers
    v4l2UnmapBuf(v4l2ctx);

    return 0;
}

#if 0
int CameraSetParm()
{
	return 0;
}

int main()
{
	int ret = 0;
	void * mV4l2_ctx = NULL;
	int mCaptureFormat;
	int width = 1280;
	int height = 720;

	int number = 0;

	struct v4l2_buffer buf;
	memset(&buf, 0, sizeof(buf));
	
	mV4l2_ctx = CreateV4l2Context();

	setV4L2DeviceName(mV4l2_ctx, "/dev/video1");

	// open v4l2 camera device
	ret = OpenCamera(mV4l2_ctx);

	if (ret != 0)
	{
		LOGE("openCameraDevice failed\n");
		return ret;
	}

	StartCamera(mV4l2_ctx, &width, &height);

	while(number < 300) {

		CameraGetOneframe(mV4l2_ctx,&buf);
		CameraReturnOneframe(mV4l2_ctx,buf.index);
		LOGD("test, number: %d",number);

		number ++;
	}
	
	StopCamera(mV4l2_ctx);

	CloseCamera(mV4l2_ctx);
	DestroyV4l2Context(mV4l2_ctx);
	mV4l2_ctx = NULL;

	return 0;
}

#endif
