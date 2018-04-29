/**
 * Simplest Librtmp Send 264
 *
 * 雷霄骅，张晖
 * leixiaohua1020@126.com
 * zhanghuicuc@gmail.com
 * 中国传媒大学/数字电视技术
 * Communication University of China / Digital TV Technology
 * http://blog.csdn.net/leixiaohua1020
 *
 * 本程序用于将内存中的H.264数据推送至RTMP流媒体服务器。
 *
 */

/**
 * _NaluUnit
 * 内部结构体。该结构体主要用于存储和传递Nal单元的类型、大小和数据
 */ 
typedef struct _NaluUnit  
{  
	int type;  
    int size;  
	unsigned char *data;  
}NaluUnit;

/**
 * _RTMPMetadata
 * 内部结构体。该结构体主要用于存储和传递元数据信息
 */ 
typedef struct _RTMPMetadata  
{  
	// video, must be h264 type   
	unsigned int    nWidth;  
	unsigned int    nHeight;  
	unsigned int    nFrameRate;      
	unsigned int    nSpsLen;  
	unsigned char   *Sps;  
	unsigned int    nPpsLen;  
	unsigned char   *Pps;   
} RTMPMetadata,*LPRTMPMetadata;  

enum  
{  
	 VIDEO_CODECID_H264 = 7,  
}; 

/**
 * 初始化并连接到服务器
 *
 * @param url 服务器上对应webapp的地址
 *					
 * @成功则返回1 , 失败则返回0
 */ 
int RTMP264_Connect(const char* url);    
    
/**
 * 将内存中的一段H.264编码的视频数据利用RTMP协议发送到服务器
 *
 * @param read_buffer 回调函数，当数据不足的时候，系统会自动调用该函数获取输入数据。
 *					2个参数功能：
 *					uint8_t *buf：外部数据送至该地址
 *					int buf_size：外部数据大小
 *					返回值：成功读取的内存大小
 * @成功则返回1 , 失败则返回0
 */ 
int RTMP264_Send(int (*read_buffer)(unsigned char *buf, int buf_size));

/**
 * 断开连接，释放相关的资源。
 *
 */    
void RTMP264_Close();  

int SendH264Packet(unsigned char *data,unsigned int size,int bIsKeyFrame,unsigned int nTimeStamp);

