#ifndef V4L2_HH
#define V4L2_HH

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "videodev.h"

// Number of Buffers to request... 
#define BUFFER_NUMBER  8
#define MAX_BUFFER_NUM 16

typedef struct v4l2_mem_map_t{
	void *	mem[MAX_BUFFER_NUM]; 
	int 	length;
}v4l2_mem_map_t;


void *CreateV4l2Context();
void DestroyV4l2Context(void* v4l2ctx);

int setV4L2DeviceName(void* v4l2ctx, const char * pname); // set device node name, such as "/dev/video0"
int setV4L2DeviceID(void* v4l2ctx, int device_id); // set different device id on the same CSI
int openCameraDevice(void* v4l2ctx); // open camera device
void closeCameraDevice(void* v4l2ctx); //close camera device

int v4l2GetCaptureFmt(void* v4l2ctx);  //can call this function after openCameraDevice()
int v4l2SetVideoParams(void* v4l2ctx, int* width, int* height,  int pix_fmt);
int v4l2ReqBufs(void* v4l2ctx, int* buffernum);
int v4l2QueryBuf(void* v4l2ctx);
v4l2_mem_map_t * GetMapmemAddress(void* v4l2ctx);

int v4l2StartStreaming(void* v4l2ctx);
int v4l2StopStreaming(void* v4l2ctx);
int v4l2UnmapBuf(void* v4l2ctx);
void releasePreviewFrame(void* v4l2ctx, int index);
int v4l2WaitCameraReady(void* v4l2ctx);

int getPreviewFrame(void* v4l2ctx, struct v4l2_buffer *buf);
int tryFmt(void* v4l2ctx, int format);
int tryFmtSize(void* v4l2ctx, int * width, int * height);
int getFrameRate(void* v4l2ctx);
int setImageEffect(void* v4l2ctx, int effect);
int setWhiteBalance(void* v4l2ctx, int wb);
int setExposure(void* v4l2ctx, int exp);
int setFlashMode(void* v4l2ctx, int mode); // flash mode
int enumSize(void* v4l2ctx, char * pSize, int len);
int setAutoFocusMode(void* v4l2ctx, int af_mode); // af mode
int setAutoFocusCtrl(void* v4l2ctx, int af_ctrl, void *areas); // af ctrl
int getAutoFocusStatus(void* v4l2ctx, int af_ctrl);
int v4l2setCaptureParams(void* v4l2ctx, struct v4l2_streamparm * params);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //V4L2_HH

