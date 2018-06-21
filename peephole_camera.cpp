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
#include <sys/types.h>
#include <signal.h>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <iostream>
#include <fstream>
#include <sstream>

#include "utils.h"
#include "motion.h"

#include "aw/vencoder.h"
#define LOG_TAG "main"
#include "aw/CDX_Debug.h"
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

// for motion detection
#define MOTION_THRESH 10
#define CROP_X 182
#define CROP_Y 109
#define CROP_WIDTH 346
#define CROP_HEIGHT 326
#define DOWNSAMPLE_RATIO 2
#define VIDEO_PATH "record"

using namespace cv;
using namespace std;

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
extern RTMPMetadata metaData;

Mat cameraYUV = Mat::zeros(mheight * 1.5, mwidth, CV_8UC1);
Mat cameraBGR = Mat::zeros(mheight, mwidth, CV_8UC3);

// for motion Detection
Mat preVideoFrame = Mat::zeros(CROP_HEIGHT / DOWNSAMPLE_RATIO, CROP_WIDTH / DOWNSAMPLE_RATIO, CV_8UC3);
Mat curVideoFrame = Mat::zeros(CROP_HEIGHT / DOWNSAMPLE_RATIO, CROP_WIDTH / DOWNSAMPLE_RATIO, CV_8UC3);
Mat nextVideoFrame = Mat::zeros(CROP_HEIGHT / DOWNSAMPLE_RATIO, CROP_WIDTH / DOWNSAMPLE_RATIO, CV_8UC3);
Mat dist(CROP_HEIGHT / DOWNSAMPLE_RATIO, CROP_WIDTH / DOWNSAMPLE_RATIO, CV_8UC1);
Mat blurDist(CROP_HEIGHT / DOWNSAMPLE_RATIO, CROP_WIDTH / DOWNSAMPLE_RATIO, CV_8UC1);
Mat threDist(CROP_HEIGHT / DOWNSAMPLE_RATIO, CROP_WIDTH / DOWNSAMPLE_RATIO, CV_8UC1);
Mat meanDist(CROP_HEIGHT / DOWNSAMPLE_RATIO, CROP_WIDTH / DOWNSAMPLE_RATIO, CV_8UC1);
Mat stdDist(1, 1, CV_8UC1);
Mat normalVideoFrame, smallVideoFrame, croppedVideoFrame;
char text[100] = {0};
int motion_flag = 0;
int record_start = 0, record_end = 0;
int record_peroid = 5;   // video length
FILE *fpRecord = NULL;


// for face recognition
string fn_haar = "/home/echo42/py/face/haarcascade_frontalface_default.xml";
string fn_csv = "/home/echo42/motion_detctor/faces.csv";
// These vectors hold the images and corresponding labels:
vector<Mat> images;
vector<int> labels;
int im_width = 0;
int im_height = 0;
// Create a FaceRecognizer and train it on the given images:
Ptr<FaceRecognizer> model;
CascadeClassifier haar_cascade;

unsigned int tick = 0;
unsigned int tick_gap = 0;

int sps_pps_send = 0;

#define ENCODE_H264

#define WATERMARK

unsigned char enc_buffer[2560 * 1920] = {0};

typedef struct Venc_context {
    VideoEncoder *pVideoEnc;
    VencBaseConfig base_cfg;
    AWCameraDevice *CameraDevice;
    WaterMark      *waterMark;
    pthread_t thread_enc_id;
    int mstart;
    int fd_in;
} Venc_context;


SimpleFIFO<VencInputBuffer, 2> g_inFIFO;
pthread_cond_t g_cond(PTHREAD_COND_INITIALIZER);
pthread_mutex_t g_mutex(PTHREAD_MUTEX_INITIALIZER);

void process_in_buffer(Venc_context *venc_cxt, VencInputBuffer *input_buffer);

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
        bool bIsKeyFrame = (output_buffer.pData0[4] & 0x1f) == 0x05 ? true : false;

        // record video if motion detected
        int nowtime = time((time_t*)NULL);
        // printf("record_start %d motion_flag %d\n", record_start, motion_flag);
        if(!record_start && motion_flag) {  // not recording now and motion detected, start recording
            record_start = nowtime;
            // start recording, create directory and video file
            time_t fptime;
            fptime = time(NULL); //获取日历时间

            struct tm *local;
            local = localtime(&fptime);  //获取当前系统时间

            char save_path[50], video_file[50];
            strftime(save_path, 50, "record/%Y%m%d", local);
            strftime(video_file, 50, "record/%Y%m%d/%Y%m%d%H%M%S.h264", local);
            if(!mkdirs(save_path)) {
                // LOGD("video_file %s.", video_file);
                fpRecord = fopen(video_file, "wb");
                if(fpRecord) {
                    LOGD("%s Created success!", video_file);
                }
            }
        }

        if(record_start && ((nowtime - record_start) >= record_peroid)) {  // still recoding and more than 5s
            if(motion_flag) { // if motion detected, recording for another 5s
                // keep recording for another 5s
                record_start = nowtime;
            } else {  // after recording for 5s, and there's no motion detected, stop recording
                record_start = 0;
                if(fpRecord) {
                    fclose(fpRecord);
                    LOGD("fpRecord Closed success!");
                    fpRecord = NULL;
                }
            }
        }

        if (output_buffer.nSize1) {
            // send h264 packet to rtmp server
            memcpy(enc_buffer, output_buffer.pData0, output_buffer.nSize0);
            memcpy(enc_buffer + output_buffer.nSize0, output_buffer.pData1, output_buffer.nSize1);
            int totalSize = output_buffer.nSize0 + output_buffer.nSize1;
            SendH264Packet(enc_buffer, totalSize, bIsKeyFrame, tick);

            // record video if motion detected
            if(record_start) {  // recording time less than 5s, continue recording
                if(bIsKeyFrame) {
                    fwrite(sps_pps_data.pBuffer, sizeof(char), sps_pps_data.nLength, fpRecord);
                }
                fwrite(enc_buffer, sizeof(char), totalSize, fpRecord);
            }

        } else {
            // send h264 packet to rtmp server
            SendH264Packet(output_buffer.pData0, output_buffer.nSize0, bIsKeyFrame, tick);

            if(record_start) {  // recording time less than 5s, continue recording
                if(bIsKeyFrame) {
                    fwrite(sps_pps_data.pBuffer, sizeof(char), sps_pps_data.nLength, fpRecord);
                }
                fwrite(output_buffer.pData0, sizeof(char), output_buffer.nSize0, fpRecord);
            }
        }

        FreeOneBitStreamFrame(pVideoEnc, &output_buffer);
    } else {
        printf("Error getting bitstream\n");
    }
}

void process_in_buffer(Venc_context *venc_cxt, VencInputBuffer *input_buffer) {
    VideoEncoder *pVideoEnc = venc_cxt->pVideoEnc;
    int result = 0;

    result = FlushCacheAllocInputBuffer(pVideoEnc, input_buffer);
    if (result < 0) {
        printf("Flush alloc error.\n");
    }
    result = AddOneInputBuffer(pVideoEnc, input_buffer);
    if (result < 0) {
        printf("Add one input buffer\n");
    }
    result = VideoEncodeOneFrame(pVideoEnc);

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

    // camera source buffer, format YUYV
    unsigned char *buffer = (unsigned char *) p_v4l2_mem_map->mem[p_buf->index];
    //int size_y = venc_cam_cxt->base_cfg.nInputWidth*venc_cam_cxt->base_cfg.nInputHeight;

    memset(&input_buffer, 0, sizeof(VencInputBuffer));
    result = GetOneAllocInputBuffer(pVideoEnc, &input_buffer);
    if (result < 0) {
        CameraDevice->returnFrame(CameraDevice, p_buf->index);
        printf("Alloc input buffer is full , skip this frame");
        return 0;
    }

    if (CameraDevice->isYUYV) { // YUYV
        libyuv::YUY2ToNV12(buffer, mwidth * 2, input_buffer.pAddrVirY, mwidth,
                           input_buffer.pAddrVirC, mwidth, mwidth, mheight);

        memcpy(cameraYUV.data, input_buffer.pAddrVirY, sizeof(unsigned char) * mheight * mwidth);
        memcpy(cameraYUV.data + mheight * mwidth, input_buffer.pAddrVirC, sizeof(unsigned char) * 0.5 * mheight * mwidth);

        cvtColor(cameraYUV, normalVideoFrame, CV_YUV2BGR_NV12);

        // crop and resize frame
        croppedVideoFrame = normalVideoFrame(Rect(Point(CROP_X, CROP_Y), Size(CROP_WIDTH, CROP_HEIGHT)));
        resize(croppedVideoFrame, nextVideoFrame, Size(), 1.0 / DOWNSAMPLE_RATIO, 1.0 / DOWNSAMPLE_RATIO);

        distMapBGR(preVideoFrame, nextVideoFrame, dist);

		curVideoFrame.copyTo(preVideoFrame);
		nextVideoFrame.copyTo(curVideoFrame);

		blur(dist, blurDist, Size(3, 3));

		threshold(blurDist, threDist, 100, 255, THRESH_BINARY);

		meanStdDev(threDist, meanDist, stdDist);

		sprintf(text, "STD - %d", stdDist.at<uchar>(0, 0));

		cv::putText(curVideoFrame, text, Point(20, 20), FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 1, 20);

		if(stdDist.at<uchar>(0, 0) > MOTION_THRESH) {
            motion_flag = 1;
			LOGD("%d motion detected", stdDist.at<uchar>(0, 0));
		} else {
            motion_flag = 0;
        }

        // apply face recognition if motion detected
        if(motion_flag) {
            Mat original = croppedVideoFrame.clone();
            Mat gray;
            cvtColor(original, gray, COLOR_BGR2GRAY);
            // Find the faces in the frame:
            vector< Rect_<int> > faces;
            haar_cascade.detectMultiScale(gray, faces);
            for(size_t i = 0; i < faces.size(); i++) {
                // Process face by face:
                Rect face_i = faces[i];
                // Crop the face from the image. So simple with OpenCV C++:
                Mat face = gray(face_i);

                Mat face_resized;
                cv::resize(face, face_resized, Size(im_width, im_height), 1.0, 1.0, INTER_CUBIC);
                int prediction = model->predict(face_resized);
                rectangle(original, face_i, Scalar(0, 255,0), 1);
                string box_text = format("Prediction = %d", prediction);
                // LOGD(box_text.c_str());
                int pos_x = std::max(face_i.tl().x - 10, 0);
                int pos_y = std::max(face_i.tl().y - 10, 0);
                putText(original, box_text, Point(pos_x, pos_y), FONT_HERSHEY_PLAIN, 1.0, Scalar(0,255,0), 2);
            }
            // Show the result:
            // imshow("face_recognizer", original);
        }

  		// imshow("distMap", blurDist);
  		// imshow("preview", curVideoFrame);

        // waitKey(1);

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

    return 0;
}


static void *encoder_thread(void *pThreadData) {

    Venc_context *venc_cxt = (Venc_context *) pThreadData;
    printf("encoder thread running....\n");

    VencInputBuffer input_buffer;

    time_t tlast, tcurrent;
    int framecount = 0, totalframes = 0;

    // Make sure a motion is set at startup....
    bool doBuffer = false;

    time(&tlast);
    time(&tcurrent);

    while (venc_cxt->mstart) {
        totalframes++;
        time(&tcurrent);

        // if ((tcurrent - tlast) > 1) {
        //     printf("frames/sec: %d\r", framecount);
        //     fflush(stdout);
        //     tlast = tcurrent;
        //     framecount = 0;
        // } else {
        //     framecount++;
        // }

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

    // init face recognition
    // Read in the data (fails if no valid input filename is given, but you'll get an error message):
    try {
        read_csv(fn_csv, images, labels, ';');
    } catch (cv::Exception& e) {
        cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << endl;
        // nothing more we can do
        exit(1);
    }

    try {
        im_width = images[0].cols;
        im_height = images[0].rows;
    } catch(Exception& e) {
        cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << endl;
        exit(1);
    }
    LOGD("im_width x im_height %d x %d\n", im_width, im_height);

    model = createEigenFaceRecognizer();
    model->train(images, labels);

    haar_cascade.load(fn_haar);

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

    baseConfig.nInputWidth = src_width;
    baseConfig.nInputHeight = src_height;
    baseConfig.nStride = src_width;

    baseConfig.nDstWidth = dst_width;
    baseConfig.nDstHeight = dst_height;

    if (g_options.input == "/dev/video0") {
        baseConfig.eInputFormat = VENC_PIXEL_YUV420SP;    // alias for NV21
    } else {
        baseConfig.eInputFormat = VENC_PIXEL_YUV420SP;  // I420
    }
    // baseConfig.eInputFormat = VENC_PIXEL_YUYV422;  // YUYV

    bufferParam.nSizeY = baseConfig.nInputWidth * baseConfig.nInputHeight;
    bufferParam.nSizeC = baseConfig.nInputWidth * baseConfig.nInputHeight / 2;
    bufferParam.nBufferNum = 4;

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
    VideoEncSetParameter(pVideoEnc, VENC_IndexParamMotionDetectEnable, &motionParam );

    input_size = mwidth * (mheight + mheight / 2);
    printf("InputSize=%d\n", input_size);

    AllocInputBuffer(pVideoEnc, &bufferParam);

    printf("create encoder ok\n");

    bool cameraOn = true;
    venc_cxt->mstart = 1;

    // connect rtmp server
    RTMP264_Connect(g_options.rtmpUrl.c_str());

    venc_cxt->CameraDevice = CreateCamera(mwidth, mheight);
    printf("create camera ok\n");
    venc_cxt->CameraDevice->setCameraDatacallback(venc_cxt->CameraDevice, (void *) venc_cxt,
                                                  (void *) &CameraSourceCallback);
    venc_cxt->CameraDevice->deviceName = g_options.input.c_str();

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
