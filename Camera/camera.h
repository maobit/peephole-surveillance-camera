#ifndef AW_CAMERA_H
#define AW_CAMERA_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
#include "videodev.h"


void *CreateCameraContext();
void DestroyCameraContext(void* v4l2ctx);
int OpenCamera(void* v4l2ctx);
void CloseCamera(void* v4l2ctx);

int StartCamera(void* v4l2ctx, int *width, int *height);
int StopCamera(void* v4l2ctx);

int CameraGetOneframe(void* v4l2ctx, struct v4l2_buffer *buffer);
void CameraReturnOneframe(void* v4l2ctx, int id);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //AW_CAMERA_H


