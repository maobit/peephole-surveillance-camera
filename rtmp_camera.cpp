/*
 * Copyright (c) 2016 Rosimildo DaSilva <rosimildo@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/stat.h>
#include <signal.h>
#include <string>

#include "aw/vencoder.h"
#include "libyuv.h"

#include "Camera/CameraSource.h"
#include "Camera/V4L2.h"
#include "water_mark.h"

#include "SimpleFIFO.h"
#include "cliOptions.h"

// for rtmp streaming
#include "librtmp_send264.h"
#include "librtmp/rtmp.h"
#include "librtmp/rtmp_sys.h"
#include "librtmp/amf.h"

#define MOTION_FLAG_FILE "/tmp/motion_sense"

// Hold command line options values...
static CmdLineOptions g_options;

unsigned int mwidth = 640;
unsigned int mheight = 480;
unsigned int input_size = mwidth * (mheight + mheight / 2);
int Y_size = mwidth * mheight;
int UV_size = mwidth * mheight / 2;

unsigned int duration = 0;

int g_msDelay = 0;

VencHeaderData sps_pps_data;
VencH264Param h264Param;
MotionParam motionParam;
VENC_CODEC_TYPE codecType = VENC_CODEC_H264;

// for rtmp
extern unsigned int  m_nFileBufSize;
extern unsigned int  nalhead_pos;
extern RTMP* m_pRtmp;
extern RTMPMetadata metaData;
extern unsigned char *m_pFileBuf;
extern unsigned char *m_pFileBuf_tmp;
extern unsigned char* m_pFileBuf_tmp_old;  //used for realloc

unsigned int tick = 0;
unsigned int tick_gap = 0;

int sps_pps_send = 0;

#define ENCODE_H264

#define WATERMARK

unsigned char enc_buffer[2560 * 1920] = {0};  // ����������Ƶ buffer

typedef struct Venc_context {
    VideoEncoder *pVideoEnc;     // ��Ƶ������ָ��
    VencBaseConfig base_cfg;      // ��������������
    AWCameraDevice *CameraDevice; // ������
    WaterMark      *waterMark;
    pthread_t thread_enc_id;      // ����ͷ�����������߳�
    int mstart;
    int fd_in;                   // �ܵ������ļ�����
} Venc_context;                   // ������������


SimpleFIFO<VencInputBuffer, 2> g_inFIFO;
pthread_cond_t g_cond(PTHREAD_COND_INITIALIZER);
pthread_mutex_t g_mutex(PTHREAD_MUTEX_INITIALIZER);

void process_in_buffer(Venc_context *venc_cxt, VencInputBuffer *input_buffer);

static void set_motion_flag() {
    char buf[512];
    sprintf(buf, "touch %s", MOTION_FLAG_FILE);
    system(buf);
}

void write_bitstream(Venc_context *venc_cxt) {
    VideoEncoder *pVideoEnc = venc_cxt->pVideoEnc;
    int result = 0;
    VencOutputBuffer output_buffer;
    memset(&output_buffer, 0, sizeof(VencOutputBuffer));
    result = GetOneBitstreamFrame(pVideoEnc, &output_buffer);
    if (result == 0) {
        // write the out to the H264 outputs, and also
        // writes a header, for each 50 frames......
        memset(enc_buffer, 0, sizeof(enc_buffer));

        // printf("tick %d\n", tick);
        tick += tick_gap;
        if (output_buffer.nSize1) {
            memcpy(enc_buffer, output_buffer.pData0, output_buffer.nSize0);
            memcpy(enc_buffer + output_buffer.nSize0, output_buffer.pData1, output_buffer.nSize1);
            int totalSize = output_buffer.nSize0 + output_buffer.nSize1;
            bool bIsKeyFrame = (output_buffer.pData0[4] & 0x1f) == 0x05 ? true : false;
            SendH264Packet(enc_buffer, totalSize, bIsKeyFrame, tick);
        } else {
            bool bIsKeyFrame = (output_buffer.pData0[4] & 0x1f) == 0x05 ? true : false;
            SendH264Packet(output_buffer.pData0, output_buffer.nSize0, bIsKeyFrame, tick);
        }

        int motion_flag = 0;
        //VideoEncGetParameter(pVideoEnc, VENC_IndexParamMotionDetectStatus, &motion_flag);
        if (motion_flag == 1) {
            set_motion_flag();
            printf("motion_flag = %d\n", motion_flag);
        }
        FreeOneBitStreamFrame(pVideoEnc, &output_buffer);
    } else {
        printf("Error getting bitstream\n");
    }
}

void process_in_buffer(Venc_context *venc_cxt, VencInputBuffer *input_buffer) {
    VideoEncoder *pVideoEnc = venc_cxt->pVideoEnc;
    int result = 0;

    //	VencOutputBuffer output_buffer;

    //struct timeval tmNow;
    //gettimeofday (&tmNow, NULL );
    //input_buffer->nPts = 1000000*(long long)tmNow.tv_sec + (long long)tmNow.tv_usec;
    //input_buffer->nPts = 900000*(long long)tmNow.tv_sec + (long long)tmNow.tv_usec;
    //long long nPts = 1000000 * (long long)tmNow.tv_sec + (long long)tmNow.tv_usec;
    //nPts = 9*(nPts/10);
    //input_buffer->nPts  = nPts;
    //printf("Cam - flush...\n" );
    result = FlushCacheAllocInputBuffer(pVideoEnc, input_buffer);
    if (result < 0) {
        printf("Flush alloc error.\n");
    }
    result = AddOneInputBuffer(pVideoEnc, input_buffer);
    if (result < 0) {
        printf("Add one input buffer\n");
    }
    result = VideoEncodeOneFrame(pVideoEnc);
    //printf("Cam - encode res = %d\n", result );
    // Update any output with RAWVIDEO data from the camera
    // in NV12 format....

    AlreadyUsedInputBuffer(pVideoEnc, input_buffer);
    ReturnOneAllocInputBuffer(pVideoEnc, input_buffer);
    if (result == 0) {
        write_bitstream(venc_cxt);
        if (h264Param.nCodingMode == VENC_FIELD_CODING && codecType == VENC_CODEC_H264) {
            write_bitstream(venc_cxt);
        }
    } else {
        printf("encoder fatal error\n");
    }
}


int CameraSourceCallback(void *cookie, void *data) {
    Venc_context *venc_cam_cxt = (Venc_context *) cookie;
    VideoEncoder *pVideoEnc = venc_cam_cxt->pVideoEnc;

    AWCameraDevice *CameraDevice = venc_cam_cxt->CameraDevice;
    VencInputBuffer input_buffer;
    int result = 0;
    struct v4l2_buffer *p_buf = (struct v4l2_buffer *) data;
    v4l2_mem_map_t *p_v4l2_mem_map = GetMapmemAddress(getV4L2ctx(CameraDevice));

    if (!venc_cam_cxt->mstart) {
        printf("p_buf->index = %d\n", p_buf->index);
        CameraDevice->returnFrame(CameraDevice, p_buf->index);
        return 0;
    }

    if (p_buf->length < input_size) {
        printf("Buffer small - index = %d\n", p_buf->index);
        CameraDevice->returnFrame(CameraDevice, p_buf->index);
        return 0;
    }

    //LOGD("Cam - p_buf->index = %d\n", p_buf->index);

    unsigned char *buffer = (unsigned char *) p_v4l2_mem_map->mem[p_buf->index];
    //int size_y = venc_cam_cxt->base_cfg.nInputWidth*venc_cam_cxt->base_cfg.nInputHeight;

    memset(&input_buffer, 0, sizeof(VencInputBuffer));
    result = GetOneAllocInputBuffer(pVideoEnc, &input_buffer);
    if (result < 0) {
        CameraDevice->returnFrame(CameraDevice, p_buf->index);
        printf("Alloc input buffer is full , skip this frame");
        return 0;
    }

    //LOGW("input buffer size=(%x) %d", p_buf->length, p_buf->length );
    //input_buffer.nPts  =   1000000 * (long long)p_buf->timestamp.tv_sec + (long long)p_buf->timestamp.tv_usec;
    //long long nPts = 1000000*(long long)p_buf->timestamp.tv_sec + (long long)p_buf->timestamp.tv_usec;
    //nPts = 9*(nPts/10);
    //input_buffer.nPts  = nPts;
    if (CameraDevice->isYUYV) { // YUYV
        libyuv::YUY2ToNV12(buffer, mwidth * 2, input_buffer.pAddrVirY, mwidth,
                           input_buffer.pAddrVirC, mwidth, mwidth, mheight);
         // set watermark
         venc_cam_cxt->waterMark->bgInfo.y = (unsigned char *) input_buffer.pAddrVirY;
         venc_cam_cxt->waterMark->bgInfo.c = (unsigned char *) input_buffer.pAddrVirC;
         waterMarkShowTime(venc_cam_cxt->waterMark);
    } else {
        //LOGD("Cam - convert from yuyv1 =%d\n", Y_size );
        memcpy((unsigned char *) input_buffer.pAddrVirY, buffer, Y_size);
        //LOGD("Cam - convert from yuyv2 =%d\n", UV_size );
        memcpy((unsigned char *) input_buffer.pAddrVirC, &buffer[Y_size], UV_size);
    }
    CameraDevice->returnFrame(CameraDevice, p_buf->index);

    pthread_mutex_lock(&g_mutex);
    bool res = g_inFIFO.enqueue(input_buffer);
    if (res) pthread_cond_signal(&g_cond);
    pthread_mutex_unlock(&g_mutex);
    if (!res) {
        printf("Unable to queue Buffer\n");
        ReturnOneAllocInputBuffer(pVideoEnc, &input_buffer);
    }

    //printf( "C");
    return 0;
}


static void *encoder_thread(void *pThreadData) {

    Venc_context *venc_cxt = (Venc_context *) pThreadData;
    printf("encoder thread running....\n");

    VencInputBuffer input_buffer;

    time_t tlast, tcurrent;
    int framecount = 0, totalframes = 0;

    // Make sure a motion is set at startup....
    set_motion_flag();
    bool doBuffer = false;

    time(&tlast);
    time(&tcurrent);

    while (venc_cxt->mstart) {
        totalframes++;
        time(&tcurrent);

        if ((tcurrent - tlast) > 1) {
            printf("frames/sec: %d\r", framecount);
            fflush(stdout);
            tlast = tcurrent;
            framecount = 0;
        } else {
            framecount++;
        }

        doBuffer = false;
        pthread_mutex_lock(&g_mutex);
        if (!g_inFIFO.count()) {
            pthread_cond_wait(&g_cond, &g_mutex);
        }
        if (g_inFIFO.count()) {
            input_buffer = g_inFIFO.dequeue();
            doBuffer = true;
        }
        pthread_mutex_unlock(&g_mutex);

        if (doBuffer) {
            process_in_buffer(venc_cxt, &input_buffer);
        }
    }
    return (void *) 0;
}


static int quit = 0;

void handle_int(int n) {
    printf("Quit handler - called\n");
    quit = 1;
}

void decode_sps_pps(VencHeaderData sps_pps_data) {
    unsigned char *sps_pps_buf = sps_pps_data.pBuffer;
    int tail =0;
    int sps_head = 0, sps_length = 0;
    int pps_head = 0, pps_length = 0;
    while(tail < sps_pps_data.nLength) {
        if(sps_pps_buf[tail] == 0x67) {
            sps_head = tail;
        } else if(sps_pps_buf[tail] == 0x68) {
            pps_head = tail;
        }
        tail++;
    }
    sps_length = pps_head - sps_head - 4;
    pps_length = sps_pps_data.nLength - pps_head;
    metaData.nSpsLen = sps_length;
    metaData.nPpsLen = pps_length;
    metaData.Sps = (unsigned char*) malloc(sizeof(unsigned char) * sps_length);
    metaData.Pps = (unsigned char*) malloc(sizeof(unsigned char) * pps_length);
    memcpy(metaData.Sps, sps_pps_buf + sps_head, sps_length);
    memcpy(metaData.Pps, sps_pps_buf + pps_head, pps_length);
}


int main(int argc, char **argv) {
    time_t time_start, time_now;
    int err = 0;
    err = processCmdLineOptions(g_options, argc, argv);
    VideoEncoder *pVideoEnc = NULL;
    VencBaseConfig baseConfig;
    VencAllocateBufferParam bufferParam;
    unsigned int src_width, src_height, dst_width, dst_height;

    mwidth = g_options.width;
    mheight = g_options.height;
    src_width = mwidth;
    src_height = mheight;
    duration = g_options.duration;

    // rtmpMetadata
    metaData.nWidth = mwidth;
    metaData.nHeight = mheight;
    metaData.nFrameRate = g_options.fps;

    tick_gap = 1000/metaData.nFrameRate;

    dst_width = g_options.width_out;
    dst_height = g_options.height_out;

    Y_size = mwidth * mheight;
    UV_size = mwidth * mheight / 2;

    printf("Size Image = %dx%d\n", mwidth, mheight);
    printf("Y_size = %d\n", Y_size);
    printf("UV_size = %d\n", UV_size);

    //intraRefresh
    //VencCyclicIntraRefresh sIntraRefresh;
    //sIntraRefresh.bEnable = 1;
    //sIntraRefresh.nBlockNumber = 10;

    //fix qp mode
    //VencH264FixQP fixQP;
    //fixQP.bEnable = 1;
    //fixQP.nIQp = g_options.qMin;
    //fixQP.nPQp = g_options.qMax;

    //* h264 param
    h264Param.bEntropyCodingCABAC = 1;
    h264Param.nBitrate = 1024 * g_options.bitrate; /* bps */
    h264Param.nFramerate = g_options.fps; /* fps */
    h264Param.nCodingMode = VENC_FRAME_CODING;
    h264Param.nMaxKeyInterval = g_options.keyInterval;
    h264Param.sProfileLevel.nProfile = VENC_H264ProfileMain;
    h264Param.sProfileLevel.nLevel = VENC_H264Level31;
    h264Param.sQPRange.nMinqp = g_options.qMin;
    h264Param.sQPRange.nMaxqp = g_options.qMax;

    Venc_context *venc_cxt = (Venc_context *) malloc(sizeof(Venc_context));
    memset(venc_cxt, 0, sizeof(Venc_context));
    memset(&baseConfig, 0, sizeof(VencBaseConfig));
    memset(&bufferParam, 0, sizeof(VencAllocateBufferParam));

    // ���ñ�������������
    baseConfig.nInputWidth = src_width;
    baseConfig.nInputHeight = src_height;
    baseConfig.nStride = src_width;

    baseConfig.nDstWidth = dst_width;
    baseConfig.nDstHeight = dst_height;

    if (g_options.input == "/dev/video0") {
        baseConfig.eInputFormat = VENC_PIXEL_YUV420SP;    // alias for NV12
    } else {
        baseConfig.eInputFormat = VENC_PIXEL_YUV420SP;  // I420
    }
    // baseConfig.eInputFormat = VENC_PIXEL_YUYV422;  // YUYV

    // buffer ��������
    bufferParam.nSizeY = baseConfig.nInputWidth * baseConfig.nInputHeight;
    bufferParam.nSizeC = baseConfig.nInputWidth * baseConfig.nInputHeight / 2;
    bufferParam.nBufferNum = 4;

    // ����������
    printf("Creating Encoder...\n");
    pVideoEnc = VideoEncCreate(codecType);
    printf("Created Encoder: %p\n", pVideoEnc);

    int value;
    VideoEncSetParameter(pVideoEnc, VENC_IndexParamH264Param, &h264Param);
    value = 0;
    VideoEncSetParameter(pVideoEnc, VENC_IndexParamIfilter, &value);
    value = 0; //degree
    VideoEncSetParameter(pVideoEnc, VENC_IndexParamRotation, &value);

    //VideoEncSetParameter(pVideoEnc, VENC_IndexParamH264FixQP, &fixQP);
    //VideoEncSetParameter(pVideoEnc, VENC_IndexParamH264CyclicIntraRefresh, &sIntraRefresh);

    venc_cxt->base_cfg = baseConfig;
    venc_cxt->pVideoEnc = pVideoEnc;


    VideoEncInit(pVideoEnc, &baseConfig);
    VideoEncGetParameter(pVideoEnc, VENC_IndexParamH264SPSPPS, &sps_pps_data);

#ifdef SPS_PPS
    unsigned int head_num = 0;
    printf("sps_pps size :%d\n", sps_pps_data.nLength);
    for (head_num = 0; head_num < sps_pps_data.nLength; head_num++)
    {
        printf("the sps_pps :%02x\n", *(sps_pps_data.pBuffer + head_num));
    }
#endif

#ifdef WATERMARK
    venc_cxt->waterMark = (WaterMark *)malloc(sizeof(WaterMark));
    memset(venc_cxt->waterMark, 0x0, sizeof(WaterMark));
    venc_cxt->waterMark->bgInfo.width  = mwidth;
    venc_cxt->waterMark->bgInfo.height = mheight;
    venc_cxt->waterMark->srcPathPrefix = (char *)"./watermark/res/icon_720p_";
    venc_cxt->waterMark->srcNum = 13;
    waterMarkInit(venc_cxt->waterMark);
#endif

    // fill up rtmp metadata sps pps info
    decode_sps_pps(sps_pps_data);

    motionParam.nMotionDetectEnable = 1;
    motionParam.nMotionDetectRatio = 1; /* 0~12, 0 is the best sensitive */
    //VideoEncSetParameter(pVideoEnc, VENC_IndexParamMotionDetectEnable, &motionParam );

    input_size = mwidth * (mheight + mheight / 2);
    printf("InputSize=%d\n", input_size);

    // ͨ�� vencoder ��������ͼ��֡ buffer
    AllocInputBuffer(pVideoEnc, &bufferParam);

    printf("create encoder ok\n");

    // ��������ͷ
    bool cameraOn = true;
    venc_cxt->mstart = 1;

    // connect rtmp server
    //��ʼ�������ӵ�������
    RTMP264_Connect(g_options.rtmpUrl.c_str());

    /* create source ��������ͷԴ������������Ϣ*/
    venc_cxt->CameraDevice = CreateCamera(mwidth, mheight);
    printf("create camera ok\n");
    /* set camera source callback ��������ͷԴ��callback*/
    venc_cxt->CameraDevice->setCameraDatacallback(venc_cxt->CameraDevice, (void *) venc_cxt,
                                                  (void *) &CameraSourceCallback);
    // Pass Device name to Camera... ������Ҫ����������ͷ������
    venc_cxt->CameraDevice->deviceName = g_options.input.c_str();
    /* start camera ��������ͷ*/
    venc_cxt->CameraDevice->startCamera(venc_cxt->CameraDevice);
    printf("Camera: is YUYV = %d\n", venc_cxt->CameraDevice->isYUYV);
    int w, h, fmt;
    getV4L2FormatAndSize(venc_cxt->CameraDevice, &w, &h, &fmt);
    printf("Camera: Width=%d, Height=%d, Pix_Fmt=%d\n", w, h, fmt);
    if (w != (int) mwidth || h != (int) mheight) {
        printf("Camera size mismatch !\n");
    }

    /* start encoder */
    venc_cxt->mstart = 1;
    /* create encode thread*/
    err = pthread_create(&venc_cxt->thread_enc_id, NULL, encoder_thread, venc_cxt);
    if (err || !venc_cxt->thread_enc_id) {
        printf("Create thread_enc_id fail !\n");
    }
    // This is right after the encoding thread has started, get the time to calculate the duration
    time(&time_start);
    struct sigaction sigact;
    memset(&sigact, 0, sizeof(sigact));
    sigact.sa_handler = handle_int;
    time(&time_now);
    while (!quit && venc_cxt->mstart &&
           ((duration > 0 && difftime(time_now, time_start) < duration) || duration == 0)) {
        if (cameraOn && !venc_cxt->CameraDevice->getState(venc_cxt->CameraDevice))
            break;
        sleep(1);
        // Get current time to check whether we need to stop to stay within the specified duration
        time(&time_now);
    }
    pthread_mutex_lock(&g_mutex);
    pthread_cond_signal(&g_cond);
    pthread_mutex_unlock(&g_mutex);
    /* stop encoder */
    venc_cxt->mstart = 0;
    if (venc_cxt->thread_enc_id != 0) {
        pthread_join(venc_cxt->thread_enc_id, NULL);
    }

    //�Ͽ����Ӳ��ͷ�������Դ
    RTMP264_Close();
#ifdef WATERMARK
    waterMarkExit(venc_cxt->waterMark);
    free(venc_cxt->waterMark);
    venc_cxt->waterMark = NULL;
#endif
    ReleaseAllocInputBuffer(pVideoEnc);
    VideoEncUnInit(pVideoEnc);
    VideoEncDestroy(pVideoEnc);
    venc_cxt->pVideoEnc = NULL;

    free(venc_cxt);
    venc_cxt = NULL;
    return 0;
}
