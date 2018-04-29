#ifndef WATER_MARK_H
#define WATER_MARK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define MAX_PIC 20

typedef struct BackGroudLayerInfo
{
	unsigned int width;
	unsigned int height;
	unsigned char* y;
	unsigned char* c;
}BackGroudLayerInfo;

typedef struct SinglePicture
{
    unsigned char  id; //picture id
	unsigned char* y;
	unsigned char* c;
	unsigned char* alph;
}SinglePicture;

typedef struct WaterMarkInfo
{
	unsigned int width; //single pic width
	unsigned int height; //single pic height
	unsigned int picture_number; 
	SinglePicture single_pic[MAX_PIC];
}WaterMarkInfo;

typedef struct WaterMarkPositon
{
	unsigned int x;
	unsigned int y;
}WaterMarkPositon;

typedef struct ShowWaterMarkParam
{
    WaterMarkPositon  pos;     //the position of the waterMark
    unsigned char     number;
    unsigned char     id_list[MAX_PIC];  //the index of the picture of the waterMark
}ShowWaterMarkParam;

typedef struct watermark
{
	BackGroudLayerInfo bgInfo;		//input
	WaterMarkInfo srcInfo;			//output 
	char* srcPathPrefix;			//input
	int srcNum;						//input
}WaterMark;



int watermark_blending(BackGroudLayerInfo *bg_info, WaterMarkInfo *wm_info, ShowWaterMarkParam *wm_Param);
int watermark_blending_ajust_brightness(BackGroudLayerInfo *bg_info, WaterMarkInfo *wm_info, ShowWaterMarkParam *wm_Param);

int waterMarkInit(WaterMark* waterMark);
int waterMarkExit(WaterMark* waterMark);
  void waterMarkShowTime(WaterMark* waterMark);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif


