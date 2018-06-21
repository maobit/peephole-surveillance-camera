/**
 * Simplest Librtmp Send 264
 *
 * �����裬����
 * leixiaohua1020@126.com
 * zhanghuicuc@gmail.com
 * �й���ý��ѧ/���ֵ��Ӽ���
 * Communication University of China / Digital TV Technology
 * http://blog.csdn.net/leixiaohua1020
 *
 * ���������ڽ��ڴ��е�H.264����������RTMP��ý����������
 *
 */

/**
 * _NaluUnit
 * �ڲ��ṹ�塣�ýṹ����Ҫ���ڴ洢�ʹ���Nal��Ԫ�����͡���С������
 */

#ifndef __LIBRTMP_SEND264_H__
#define __LIBRTMP_SEND264_H__

typedef struct _NaluUnit
{
	int type;
    int size;
	unsigned char *data;
}NaluUnit;

/**
 * _RTMPMetadata
 * �ڲ��ṹ�塣�ýṹ����Ҫ���ڴ洢�ʹ���Ԫ������Ϣ
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
 * ��ʼ�������ӵ�������
 *
 * @param url �������϶�Ӧwebapp�ĵ�ַ
 *
 * @�ɹ��򷵻�1 , ʧ���򷵻�0
 */
int RTMP264_Connect(const char* url);

/**
 * ���ڴ��е�һ��H.264��������Ƶ��������RTMPЭ�鷢�͵�������
 *
 * @param read_buffer �ص������������ݲ�����ʱ����ϵͳ���Զ����øú�����ȡ�������ݡ�
 *					2���������ܣ�
 *					uint8_t *buf���ⲿ���������õ�ַ
 *					int buf_size���ⲿ���ݴ�С
 *					����ֵ���ɹ���ȡ���ڴ���С
 * @�ɹ��򷵻�1 , ʧ���򷵻�0
 */
int RTMP264_Send(int (*read_buffer)(unsigned char *buf, int buf_size));

/**
 * �Ͽ����ӣ��ͷ����ص���Դ��
 *
 */
void RTMP264_Close();

int SendH264Packet(unsigned char *data,unsigned int size,int bIsKeyFrame,unsigned int nTimeStamp);

#endif
