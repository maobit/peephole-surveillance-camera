#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "camera.h"

#include "Camera/V4L2.h"
  
typedef int (*CameraDataCallback)(void *cookie,  void *data);

typedef struct CameraDataCallbackType{
	void *cookie;   //SimpleRecorder
	CameraDataCallback callback;
}CameraDataCallbackType;


typedef struct AWCameraDevice
{
	void *context;
        int  isYUYV;
        const char *deviceName;
	int  (*startCamera)(struct AWCameraDevice *p);
	int  (*stopCamera)(struct AWCameraDevice *p);
	int  (*returnFrame)(struct AWCameraDevice *p, int id);
	int  (*setCameraDatacallback)(struct AWCameraDevice *p, void *cookie, void *callback);
	int  (*getState)(struct AWCameraDevice *p);
}AWCameraDevice;

AWCameraDevice *CreateCamera(int width, int height);
void DestroyCamera(AWCameraDevice* camera);
void* getV4L2ctx(AWCameraDevice *p);
int  getV4L2FormatAndSize(AWCameraDevice *p, int *width, int *height, int *pix_fmt );

#ifdef __cplusplus
}
#endif /* __cplusplus */

