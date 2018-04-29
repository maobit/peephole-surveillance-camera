#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define LOG_NDEBUG 0
#define LOG_TAG "venc-file"
#include "aw/CDX_Debug.h"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include <time.h>
#include <water_mark.h>

static int data_convert(int input, int length, int *output)
{
	int tmp_data;
	tmp_data = input;

	if (length == 2) {
		output[0] = tmp_data/10;
		output[1] = tmp_data%10;
	} else if (length == 4) {
		output[0] = tmp_data/1000;
		tmp_data = tmp_data%1000;
		output[1] = tmp_data/100;
		tmp_data = tmp_data%100;
		output[2] = tmp_data/10;
		output[3] = tmp_data%10;
	} else {
		LOGD("error length\n");
		return -1;
	}

	return 0;

}

void argb2yuv420sp(unsigned char* src_p, unsigned char* alph, 
				unsigned int width, unsigned int height,
				unsigned char* dest_y, unsigned char* dest_c)
{
	int i,j;
	for(i = 0; i < (int)height; i++)
    {
		if((i&1) == 0)
		{
	    	for(j= 0; j< (int)width; j++)
			{
				*dest_y = (299*src_p[2]+587*src_p[1]+114*src_p[0])/1000;

				if((j&1) == 0)
				{
				   //cb
				   *dest_c++ = 128+(564*(src_p[0]-*dest_y)/1000);	
				}
				else
				{
				   // cr
				   *dest_c++ = 128+(713*(src_p[2]-*dest_y)/1000);
				}

				*alph++ = src_p[3];
				src_p +=4;
				dest_y++;
			}		
		}
		else
		{
			for(j= 0; j< (int)width; j++)
			{
				*dest_y = (299*src_p[2]+587*src_p[1]+114*src_p[0])/1000;
				*alph++ = src_p[3];
				src_p +=4;
				dest_y++;
			}	
		}
    }
	
	return;
}

	
// bg_width			background width
// bg_height        background height

// left             foreground position of left
// top              foreground position of top
// fg_width         foreground width
// fg_height        foreground height

// bg_y           	point to the background YUV420sp format y component data
// bg_c           	point to the background YUV420sp format c component data
// fg_y           	point to the foreground YUV420sp format y component data
// fg_c           	point to the foreground YUV420sp format c component data
// alph             point to foreground alph value

void yuv420sp_blending(unsigned int bg_width, unsigned int bg_height,
					   unsigned int left, unsigned int top,
					   unsigned int fg_width, unsigned int fg_height,
					   unsigned char *bg_y, unsigned char *bg_c,
					   unsigned char *fg_y, unsigned char *fg_c,
					   unsigned char *alph)
{
	unsigned char *bg_y_p = NULL;
	unsigned char *bg_c_p = NULL;
	int i = 0;
	int j = 0;

	bg_y_p = bg_y + top * bg_width + left;
	bg_c_p = bg_c + (top >>1)*bg_width + left;
	
	for(i = 0; i<(int)fg_height; i++)
	{
		if((i&1) == 0)
		{
			for(j=0; j< (int)fg_width; j++)
			{
				*bg_y_p = ((256 - *alph)*(*bg_y_p) + (*fg_y++)*(*alph))>>8;
				*bg_c_p = ((256 - *alph)*(*bg_c_p) + (*fg_c++)*(*alph))>>8;
				
				alph++;
				bg_y_p++;
				bg_c_p++;
			}

			bg_c_p = bg_c_p + bg_width - fg_width;
		}
		else
		{
			for(j=0; j< (int)fg_width; j++)
			{
				*bg_y_p = ((256 - *alph)*(*bg_y_p) + (*fg_y++)*(*alph))>>8;
				alph++;
				bg_y_p++;
			}			
		}

		bg_y_p = bg_y_p + bg_width - fg_width;
	}
}

// bg_width			background width
// bg_height        background height

// left             foreground position of left
// top              foreground position of top
// fg_width         foreground width
// fg_height        foreground height
// bg_y           	point to the background YUV420sp format y component data

// return value : 0 dark, 1 bright

int region_bright_or_dark(unsigned int bg_width, unsigned int bg_height,
					   unsigned int left, unsigned int top,
					   unsigned int fg_width, unsigned int fg_height,
					   unsigned char *bg_y)
{
	unsigned char *bg_y_p = NULL;

	int i = 0;
	int j = 0;
	int bright_line_number = 0;
	int value = 0;

	bg_y_p = bg_y + top * bg_width + left;
	
	for(i = 0; i<(int)fg_height; i++)
	{
		value = 0;
		for(j=0; j< (int)fg_width; j++)
		{
			value += *bg_y_p++;
		}

		value = value/fg_width;

		if(value > 128) {
			bright_line_number++;
		}		

		bg_y_p = bg_y_p + bg_width - fg_width;
	}

	if(bright_line_number > (int)fg_height/2) {
		return 1;
	}

	return 0;
}


// bg_width			background width
// bg_height        background height

// left             foreground position of left
// top              foreground position of top
// fg_width         foreground width
// fg_height        foreground height

// bg_y           	point to the background YUV420sp format y component data
// bg_c           	point to the background YUV420sp format c component data
// fg_y           	point to the foreground YUV420sp format y component data
// fg_c           	point to the foreground YUV420sp format c component data
// alph             point to foreground alph value

void yuv420sp_blending_adjust_brightness(unsigned int bg_width, unsigned int bg_height,
					   unsigned int left, unsigned int top,
					   unsigned int fg_width, unsigned int fg_height,
					   unsigned char *bg_y, unsigned char *bg_c,
					   unsigned char *fg_y, unsigned char *fg_c,
					   unsigned char *alph)
{
	unsigned char *bg_y_p = NULL;
	unsigned char *bg_c_p = NULL;
	int  is_brightness = 0;
	int i = 0;
	int j = 0;

	is_brightness =  region_bright_or_dark(bg_width,bg_height,left,top,
												fg_width,fg_height,bg_y);
	bg_y_p = bg_y + top * bg_width + left;
	bg_c_p = bg_c + (top >>1)*bg_width + left;
		
	if(is_brightness) {
		for(i = 0; i<(int)fg_height; i++)
		{
			if((i&1) == 0)
			{
				for(j=0; j< (int)fg_width; j++)
				{
					*bg_y_p = ((256 - *alph)*(*bg_y_p) + (256 - (*fg_y++))*(*alph))>>8;
					*bg_c_p = ((256 - *alph)*(*bg_c_p) + (*fg_c++)*(*alph))>>8;
					
					alph++;
					bg_y_p++;
					bg_c_p++;
				}

				bg_c_p = bg_c_p + bg_width - fg_width;
			}
			else
			{
				for(j=0; j< (int)fg_width; j++)
				{
					*bg_y_p = ((256 - *alph)*(*bg_y_p) + (256 - (*fg_y++))*(*alph))>>8;
					alph++;
					bg_y_p++;
				}			
			}

			bg_y_p = bg_y_p + bg_width - fg_width;
		}
	} else {
		for(i = 0; i<(int)fg_height; i++)
		{
			if((i&1) == 0)
			{
				for(j=0; j< (int)fg_width; j++)
				{
					*bg_y_p = ((256 - *alph)*(*bg_y_p) + (*fg_y++)*(*alph))>>8;
					*bg_c_p = ((256 - *alph)*(*bg_c_p) + (*fg_c++)*(*alph))>>8;
					
					alph++;
					bg_y_p++;
					bg_c_p++;
				}

				bg_c_p = bg_c_p + bg_width - fg_width;
			}
			else
			{
				for(j=0; j< (int)fg_width; j++)
				{
					*bg_y_p = ((256 - *alph)*(*bg_y_p) + (*fg_y++)*(*alph))>>8;
					alph++;
					bg_y_p++;
				}			
			}

			bg_y_p = bg_y_p + bg_width - fg_width;
		}
	}
}

int watermark_blending(BackGroudLayerInfo *bg_info, WaterMarkInfo *wm_info, ShowWaterMarkParam *wm_Param)
{
	int i;
	int id;
	if(wm_info->width*wm_Param->number > bg_info->width) {
		printf("watermark_blending error region\n");
		return -1;
	}

	for(i = 0; i<(int)wm_Param->number; i++)
	{
		id = wm_Param->id_list[i];
		yuv420sp_blending(bg_info->width,bg_info->height, (wm_Param->pos.x + wm_info->width*i),wm_Param->pos.y, 
										wm_info->width,wm_info->height,
										bg_info->y, bg_info->c,
						   				wm_info->single_pic[id].y, wm_info->single_pic[id].c,
						   				wm_info->single_pic[id].alph);	
	}
	return 0;
}

int watermark_blending_ajust_brightness(BackGroudLayerInfo *bg_info, WaterMarkInfo *wm_info, ShowWaterMarkParam *wm_Param)
{

	int i;
	int id;
	if(wm_info->width*wm_Param->number > bg_info->width) {
		printf("watermark_blending error region\n");
		return -1;
	}

	for(i = 0; i<wm_Param->number; i++)
	{
		id = wm_Param->id_list[i];
		yuv420sp_blending_adjust_brightness(bg_info->width,bg_info->height, (wm_Param->pos.x + wm_info->width*i),wm_Param->pos.y, 
										wm_info->width,wm_info->height,
										bg_info->y, bg_info->c,
										wm_info->single_pic[id].y, wm_info->single_pic[id].c,
										wm_info->single_pic[id].alph);	
	}
	return 0;
}


int waterMarkInit(WaterMark* waterMark)
{
	unsigned char *tmp_argb = NULL;
	char filename[64];
	int i;

	// init watermark pic info
	for(i = 0; i< waterMark->srcNum; i++)
	{
		FILE* icon_hdle = NULL;
		int width  = 0;
		int height = 0;

		//sprintf(filename, "%s%d.bmp", "/data/camera/icon_720p_",i);
		sprintf(filename, "%s%d.bmp", waterMark->srcPathPrefix, i);
	    icon_hdle   = fopen(filename, "r");
		if (icon_hdle == NULL) {
			printf("get wartermark %s error\n", filename);
			return -1;
		}

		//get watermark picture size
		fseek(icon_hdle, 18, SEEK_SET);
		fread(&width, 1, 4, icon_hdle);
		fread(&height, 1, 4, icon_hdle);
		fseek(icon_hdle, 54, SEEK_SET);

		if (height < 0)
		{
			height = height * (-1);
		}
		
		if(waterMark->srcInfo.width == 0) {
			waterMark->srcInfo.width = width;
			waterMark->srcInfo.height = height;
		}

		waterMark->srcInfo.single_pic[i].y = (unsigned char*)malloc(width * height * 5 / 2);
		waterMark->srcInfo.single_pic[i].alph = waterMark->srcInfo.single_pic[i].y + width * height;
		waterMark->srcInfo.single_pic[i].c = waterMark->srcInfo.single_pic[i].alph + width * height;
		waterMark->srcInfo.single_pic[i].id = i;

		if(tmp_argb == NULL) {
			tmp_argb = (unsigned char*)malloc(waterMark->srcInfo.width * waterMark->srcInfo.height * 4);
		}

		fread(tmp_argb, width * height * 4, 1, icon_hdle); 

		argb2yuv420sp(tmp_argb, waterMark->srcInfo.single_pic[i].alph, width, height,
				waterMark->srcInfo.single_pic[i].y, waterMark->srcInfo.single_pic[i].c);

		if (icon_hdle) {
			 fclose(icon_hdle);	
			 icon_hdle = NULL;
		}
	}

	waterMark->srcInfo.picture_number = i;
	if (tmp_argb != NULL)
	{
		free(tmp_argb);
		tmp_argb = NULL;
	}
	return 0;
}


int waterMarkExit(WaterMark* waterMark)
{
	int i;
	for(i = 0; i < waterMark->srcNum; i++)
	{
		if(waterMark->srcInfo.single_pic[i].y) {
			free(waterMark->srcInfo.single_pic[i].y);
			waterMark->srcInfo.single_pic[i].y = NULL;
		}
	}
	return 0;
}


void waterMarkShowTime(WaterMark* waterMark)
{
	ShowWaterMarkParam param;
	time_t	now;         
	struct	tm   *timenow;

	int year[4];
	int month[2];
	int day[2];
	int hour[2];
	int min[2];
	int sec[2];
/* get current time */

	time(&now);

	timenow = localtime(&now);
	data_convert(timenow->tm_year + 1900, 4, (int *)&year);
	data_convert(timenow->tm_mon+1, 2, (int *)&month);
	data_convert(timenow->tm_mday, 2, (int *)&day);
	data_convert(timenow->tm_hour, 2, (int *)&hour);
	data_convert(timenow->tm_min, 2, (int *)&min);
	data_convert(timenow->tm_sec, 2, (int *)&sec);

	param.pos.x = 32;
	param.pos.y = 32;
	param.id_list[0] = year[0];
	param.id_list[1] = year[1];
	param.id_list[2] = year[2];
	param.id_list[3] = year[3];
	param.id_list[4] = 11;
	param.id_list[5] = month[0];
	param.id_list[6] = month[1];
	param.id_list[7] = 11;
	param.id_list[8] = day[0];
	param.id_list[9] = day[1];
	param.id_list[10] = 10;
	param.id_list[11] = hour[0];
	param.id_list[12] = hour[1];
	param.id_list[13] = 12;
	param.id_list[14] = min[0];
	param.id_list[15] = min[1];
	param.id_list[16] = 12;
	param.id_list[17] = sec[0];
	param.id_list[18] = sec[1];
	param.number = 19;
	
//	time1 = GetNowUs();

#if 0
	watermark_blending(&waterMark->bgInfo, &waterMark->srcInfo, &param);
#else
	watermark_blending_ajust_brightness(&waterMark->bgInfo, &waterMark->srcInfo, &param);
#endif

//	time2 = GetNowUs();

//	printf("11water_mark_blending time: %lld(us)\n",time2 - time1);
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

