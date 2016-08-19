#define _RTC_C
 
#include "../main/AllModule.h"
 
/******************************************
* �������ƣ�I2C_RTC_DataSetOrRead
* ������������ʼI2C
* ��ڲ�������
* ���ڲ�������
* �� �� ֵ����
* ȫ�ֱ�������
* ���ú�������
* ����      ����               ����
* Carl.L    2009��07��29��     ����
* xxxxxx    xxxx��xx��xx��     �޸�xx����
*******************************************/
void I2C_RTC_DataSetOrRead(u8* pBuffer, u8 addr, u8 NumByte,u8 operation)
{
 
if(pBuffer == NULL)
{
 
return;
 
}
 
if(operation == I2C_READ)
{
 
I2C_EE_BufferRead(pBuffer ,addr ,NumByte ,RTC_ADDRESS_READ);
 
}
else if(operation == I2C_WRITE)
{
 
//�Ƚ���д����
u8_rtcSetBuffer = 0x80;
I2C_EE_BufferWrite(&u8_rtcSetBuffer ,0x10 ,1 ,RTC_ADDRESS_WRITE);  //��WRTC1=1 ,��ַ10H  
u8_rtcSetBuffer = 0x84;
I2C_EE_BufferWrite(&u8_rtcSetBuffer ,0x0F ,1 ,RTC_ADDRESS_WRITE);  //��WRTC2,WRTC3=1  ,��ַ0FH  
 
//д����
I2C_EE_BufferWrite(pBuffer ,addr ,NumByte ,RTC_ADDRESS_WRITE);
 
//д��ֹ
u8_rtcSetBuffer = 0;
I2C_EE_BufferWrite(&u8_rtcSetBuffer ,0x0F ,1 ,RTC_ADDRESS_WRITE);  //��WRTC2,WRTC3=0   ,��ַ0FH  
//u8_rtcSetBuffer = 0;
I2C_EE_BufferWrite(&u8_rtcSetBuffer ,0x10 ,1 ,RTC_ADDRESS_WRITE);  //��WRTC1=0  ,��ַ10H  
 
}
 
 
}
 
/******************************************
* �������ƣ�GetOrSetSystemTime
* ������������ȡ��������Systemʱ��
* ��ڲ�����operation--��������
* ���ڲ�������
* �� �� ֵ����
* ȫ�ֱ�������
* ���ú�������
* ����      ����               ����
* Carl.L    2009��07��29��     ����
* xxxxxx    xxxx��xx��xx��     �޸�xx����
*******************************************/
void GetOrSetSystemTime(u8 operation)
{
 
u8 u8_rtcTimesBuffer[7] = {
0
};  //��,��,ʱ,����,��,��,��
u32 u32_temp = 0;
u8 u8_Buf[6] = {
0
};
 
if(((operation != UNIX_TIME_SET) && (operation != UNIX_TIME_READ)))
{
 
return;
 
}
 
if(operation == UNIX_TIME_READ)
{
 
//��ȡʵʱʱ��оƬʱ�䣬0x00ΪRTC�ļĴ�����ַ
I2C_RTC_DataSetOrRead(u8_rtcTimesBuffer,0x00 ,7 ,I2C_READ);
 
//���Ȱ�Сʱ�ֽڵ����λ����,��λΪ24/12Сʱ��ѡ��
u8_rtcTimesBuffer[2] &= 0x7F;
 
//start of ת����Beijingʱ�䣬�룬�֣�ʱ
RtcTimeDatasConversion(u8_rtcTimesBuffer,&SystemTimeStr.BeiJing_time[2],3,RTC_FORM_TO_BEIJING);
//u8_rtcTimesBuffer[3]Ϊ���ڣ�����Ҫ���ֽ�
//ת����Beijingʱ�䣬�룬�֣�ʱ
RtcTimeDatasConversion(&u8_rtcTimesBuffer[4],&SystemTimeStr.BeiJing_time[5],3,RTC_FORM_TO_BEIJING);
 
SystemTimeStr.BeiJing_time[0] = 0x00;  //�����8λ
SystemTimeStr.BeiJing_time[1] = 0x00;  //�����8λ
//����ת����16����Ȼ������ֽ�����ֽ�
u32_temp = SystemTimeStr.BeiJing_time[7] + 2000; //20xx��
SystemTimeStr.BeiJing_time[7] = u32_temp; //��8λ
SystemTimeStr.BeiJing_time[8] = u32_temp >> 8;  //��8λ
//end of ת����Beijingʱ��
 
//start of ת����unixʱ��
u8_Buf[0] = u32_temp - UNIX_BASE_YEAR;  //��
u8_Buf[1] = SystemTimeStr.BeiJing_time[6] - UNIX_BASE_MONTH;  //��
u8_Buf[2] = SystemTimeStr.BeiJing_time[5] - UNIX_BASE_DATE;  //��
u8_Buf[3] = SystemTimeStr.BeiJing_time[4] - UNIX_BASE_HOUR;  //ʱ
u8_Buf[4] = SystemTimeStr.BeiJing_time[3] - UNIX_BASE_MIN;  //��
u8_Buf[5] = SystemTimeStr.BeiJing_time[2];  //��
 
u32_temp = UnixTimeConversion(u8_Buf[0],u8_Buf[1],u8_Buf[2],u8_Buf[3],u8_Buf[4],u8_Buf[5]); 
SystemTimeStr.Unix_time[0] = u32_temp;  //0-8bit
SystemTimeStr.Unix_time[1] = u32_temp >> 8;  //8-16bit
SystemTimeStr.Unix_time[2] = u32_temp >> 16;  //16-24bit
SystemTimeStr.Unix_time[3] = u32_temp >> 24;  //24-32bit
//end of ת����unixʱ��
 
}
else   //����ʱ������һ��Ҫ�Ǳ���ʱ��ĸ�ʽ!!!
{
 
//ת����TRCоƬ����Ҫ�ĸ�ʽ
//ת�����ʽ
RtcTimeDatasConversion(&SystemTimeStr.BeiJing_time[2],u8_rtcTimesBuffer,3,BEIJING_FORM_TO_RTC);
u8_rtcTimesBuffer[2] |= 0x80;  //ʹ��24Сʱ����
 
//��ȡ������
I2C_RTC_DataSetOrRead(&u8_rtcTimesBuffer[3],0x03 ,1 ,I2C_READ);
RtcTimeDatasConversion(&SystemTimeStr.BeiJing_time[4],&u8_rtcTimesBuffer[4],2,BEIJING_FORM_TO_RTC);
u32_temp = (SystemTimeStr.BeiJing_time[8] << 8) + SystemTimeStr.BeiJing_time[7];
if(u32_temp > 2000)
{
 
u32_temp -= 2000;
RtcTimeDatasConversion((u8 *)u32_temp,&u8_rtcTimesBuffer[6],1,BEIJING_FORM_TO_RTC);
 
}
 
I2C_RTC_DataSetOrRead(u8_rtcTimesBuffer,0x00 ,7 ,I2C_WRITE);
 
 
}
 
 
}
 
/******************************************
* �������ƣ�RtcTimeDatasConversion
* ����������Rtcʱ��оƬ���ݸ�ʽת��
* ��ڲ�����pBuffer--�����׵�ַ
*           NumByte--ת�����ݵĳ���
*           operation--��������
* ���ڲ�������
* �� �� ֵ����
* ȫ�ֱ�������
* ���ú�������
* ����      ����               ����
* Carl.L    2009��07��29��     ����
* xxxxxx    xxxx��xx��xx��     �޸�xx����
*******************************************/
void RtcTimeDatasConversion(u8* pBufIn, u8* pBufferOut, u8 NumByte, u8 operation)
{
 
u8 i = 0;
u8 high_4bit = 0;
u8 Low_4bit = 0;
 
if((pBufIn == NULL) || (pBufferOut == NULL))
{
 
return;
 
}
 
if((operation != RTC_FORM_TO_BEIJING) && (operation != BEIJING_FORM_TO_RTC))
{
 
return;
 
}
 
if(operation == RTC_FORM_TO_BEIJING)
{
 
for(i = 0;i < NumByte;i++)
{
 
high_4bit = (*pBufIn & 0xF0) >> 4;
Low_4bit = *pBufIn & 0x0F;
 
*pBufferOut = (high_4bit * 10) + Low_4bit;
 
pBufIn++;
pBufferOut++;
 
}
 
}
else
{
 
for(i = 0;i < NumByte;i++)
{
 
high_4bit = (*pBufIn / 10) << 4;
Low_4bit = *pBufIn & 0x0F;
 
*pBufferOut = high_4bit + Low_4bit;
 
pBufIn++;
pBufferOut++;
 
}
 
 
}
 
}