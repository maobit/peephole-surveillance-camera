#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

//#define LOG_NDEBUG 0
#define LOG_TAG "CameraSource"
#include <aw/CDX_Debug.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include "CameraSource.h"

#define DEVICE_NAME_VIDEO "/dev/video0"
  
typedef struct AWCameraContext
{
	int width;
	int height;
	int framerate;
	int devide_id;
	pthread_t thread_id;
	int isStart;
	pthread_mutex_t lock;
	void *v4l2ctx;
	CameraDataCallbackType callback;
}AWCameraContext;

int get_state(AWCameraContext *cameractx)
{
	int isStart;
	pthread_mutex_lock(&cameractx->lock);
	isStart = cameractx->isStart;
	pthread_mutex_unlock(&cameractx->lock);

	return isStart;
}

int set_state(AWCameraContext *cameractx, int state)
{
	pthread_mutex_lock(&cameractx->lock);
	cameractx->isStart = state;
	pthread_mutex_unlock(&cameractx->lock);
	
	return 0;
}

static void* CameraThread(void* pThreadData) {
	
	int result;
	int state = 1;

	AWCameraDevice* camera_dev = (AWCameraDevice *)(pThreadData);
	AWCameraContext* CameraCtx = NULL;
	struct v4l2_buffer buf;
	
	CameraCtx = (AWCameraContext*)(camera_dev->context);
	
	while(1) {

		state = get_state(CameraCtx);
		if(state == 0) {
			
			goto EXIT;
		}

		result = CameraGetOneframe(CameraCtx->v4l2ctx, &buf);

		if(result== 0) {

			if( CameraCtx->callback.cookie == NULL) {
				LOGE("the callback isn't register");
				CameraReturnOneframe(CameraCtx->v4l2ctx, buf.index);
			}else {
//				LOGD("CALL Callback");
				CameraCtx->callback.callback(CameraCtx->callback.cookie, &buf);
			}
		}else {
			LOGE("get frame error");
			//goto EXIT;
		}
	}
EXIT:
	set_state(CameraCtx,0);
	return (void *)0;
}

static int startcamera(AWCameraDevice *p)
{	
	AWCameraDevice* camera_dev = (AWCameraDevice *)(p);
	AWCameraContext* CameraCtx = NULL;
	int err = 0;
	
	if(!camera_dev)
		return -1;

	CameraCtx = (AWCameraContext*)(camera_dev->context);

	if(CameraCtx->callback.cookie == NULL) {
		LOGE("should set callback before start Camera");
		return -1;
	}
        camera_dev->isYUYV = 0;
	setV4L2DeviceName(CameraCtx->v4l2ctx, camera_dev->deviceName );
	OpenCamera(CameraCtx->v4l2ctx);

	StartCamera(CameraCtx->v4l2ctx, &CameraCtx->width, &CameraCtx->height);

        camera_dev->isYUYV = V4L2_PIX_FMT_YUYV == v4l2GetCaptureFmt(CameraCtx->v4l2ctx);

	err = getFrameRate(CameraCtx->v4l2ctx);
	LOGD("Camera FPS=%d", err );
	
	set_state(CameraCtx, 1);
	
	// Create the camera thread
	err = pthread_create(&CameraCtx->thread_id, NULL, CameraThread, camera_dev);
	if (err || !CameraCtx->thread_id) {
		LOGE("Create thread error");
		return -1;
	}
	
	return 0;
}

static int stopcamera(AWCameraDevice *p)
{
	AWCameraDevice* camera_dev = (AWCameraDevice *)(p);
	AWCameraContext* CameraCtx = NULL;
	int err = 0;
	
	if(!camera_dev)
		return -1;

	CameraCtx = (AWCameraContext *)(camera_dev->context);

	set_state(CameraCtx, 0);

	// Wait for thread to exit;
	pthread_join(CameraCtx->thread_id, (void*) &err);

	StopCamera(CameraCtx->v4l2ctx);
	CloseCamera(CameraCtx->v4l2ctx);
	
	return 0;
}

void* getV4L2ctx(AWCameraDevice *p)
{
	if (p == NULL) return NULL;
	
	AWCameraContext* CameraCtx = p->context;
	return CameraCtx->v4l2ctx;
}

int  getV4L2FormatAndSize(AWCameraDevice *p, int *width, int *height, int *pix_fmt )
{
	if (p == NULL) return -1;
	AWCameraContext* CameraCtx = p->context;

	*width   = CameraCtx->width;
	*height  = CameraCtx->height;
        *pix_fmt = v4l2GetCaptureFmt(CameraCtx->v4l2ctx);
        return 0;
}
  
static int returnFrame(AWCameraDevice *p, int id)
{
	AWCameraDevice* camera_dev = (AWCameraDevice *)(p);
	AWCameraContext* CameraCtx = NULL;
	
	if(!camera_dev)
		return -1;

	CameraCtx = (AWCameraContext *)(camera_dev->context);

	CameraReturnOneframe(CameraCtx->v4l2ctx, id);
	
	return 0;
}

int  setCameraDatacallback(struct AWCameraDevice *p, void *cookie, void *callback)
{
	AWCameraDevice* camera_dev = (AWCameraDevice *)(p);
	AWCameraContext* CameraCtx = NULL;
	LOGD("set call back22");

	if(!camera_dev)
		return -1;

	CameraCtx = (AWCameraContext *)(camera_dev->context);

	CameraCtx->callback.cookie = cookie;
	CameraCtx->callback.callback = callback;
	LOGD("CameraCtx->callback.cookie: %p", CameraCtx->callback.cookie);
	return 0;
}

static int getState(AWCameraDevice *p)
{	
	AWCameraDevice* camera_dev = (AWCameraDevice *)(p);
	AWCameraContext* CameraCtx = NULL;
	if(!camera_dev)
	   return 0;
	CameraCtx = (AWCameraContext*)(camera_dev->context);
	return get_state(CameraCtx);
}
  
AWCameraDevice* CreateCamera(int width, int height)
{
	AWCameraContext* CameraCtx = NULL;
	AWCameraDevice* camera_dev = (AWCameraDevice *)malloc(sizeof(AWCameraDevice));
	if(camera_dev == NULL) {
		LOGE("malloc AWCameraDevice failed");
		return NULL;
	}

	camera_dev->context = (void *)malloc(sizeof(AWCameraContext));
	if(camera_dev->context == NULL) {
		LOGE("malloc AWCameraContext failed");
		free(camera_dev);
		return NULL;
	}

	CameraCtx = (AWCameraContext *)(camera_dev->context);

	CameraCtx->devide_id = 0;
	CameraCtx->width = width;
	CameraCtx->height = height;
	CameraCtx->framerate = 30;

	CameraCtx->v4l2ctx = CreateCameraContext();

	if(!CameraCtx->v4l2ctx) {
		free(camera_dev->context);
		free(camera_dev);
		return NULL;
	}

	pthread_mutex_init(&CameraCtx->lock,NULL);

	camera_dev->startCamera = startcamera;
	camera_dev->stopCamera = stopcamera;
	camera_dev->setCameraDatacallback = setCameraDatacallback;
	camera_dev->returnFrame = returnFrame;
	camera_dev->getState    = getState;
	return camera_dev;
}

void DestroyCamera(AWCameraDevice* camera)
{
	AWCameraContext* CameraCtx = NULL;
	if(!camera)
		return;

	if(camera->context) {
	        CameraCtx = (AWCameraContext*)(camera->context);
		pthread_mutex_destroy(&CameraCtx->lock);
		free(camera->context);
		camera->context = NULL;
	}

}

#ifdef __cplusplus
}
#endif /* __cplusplus */

