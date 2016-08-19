#ifndef __I2C_H
#define	__I2C_H

#include "stm32f10x.h"

//*********������IO�ڶ���*********
typedef unsigned char uchar;
typedef unsigned int  uint;
#define		RTC_Address			0x64//RTC������ַ
//uchar   data1[8];  		// EEPROM����
//uchar	Sram[8]={"01234567"};	//ͨ�����ݻ�����

typedef	struct
{
	uchar second;
	uchar	minute;
	uchar	hour;
	uchar	week;
	uchar	day;
	uchar	month;
	uchar	year;
}S_Time;	
	

/***********************************************************************
***PB7 SDA
***********************************************************************/
//#define SDA_IN	    PBin(7)     //����
#define SDA_OUT	      PBout(7)    //���

/***********************************************************************
***PB6 SCL
***********************************************************************/
#define SCL_IN	    PBin(6)	//����
#define SCL_OUT	    PBout(6) 	//���


#define  SDA_IN	   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)
/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			{p->BSRR=i;}			//����Ϊ�ߵ�ƽ		
#define digitalLo(p,i)			{p->BRR=i;}				//����͵�ƽ
#define digitalToggle(p,i)		{p->ODR ^=i;}			//�����ת״̬

/* �������IO�ĺ� */
#define SDA_TOGGLE		digitalToggle(GPIOB,GPIO_Pin_7)
#define SDA_H					digitalHi(GPIOB,GPIO_Pin_7)
#define SDA_L					digitalLo(GPIOB,GPIO_Pin_7)

#define SCL_TOGGLE		digitalToggle(GPIOB,GPIO_Pin_7)
#define SCL_H					digitalHi(GPIOB,GPIO_Pin_7)
#define SCL_L					digitalLo(GPIOB,GPIO_Pin_7)

#define		RTC_Address			0x64//RTC������ַ
//EEPROM/NVSRAM��ַ���塣β׺ΪA���ڲ�����EEPROM�洢��
#define		EE_FirstPage_Address		0xA0//��һҳ��β׺B��C��D��E��F���У�
#define		EE_SecondPage_Address		0xA2//�ڶ�ҳ��β׺B��C���У�
#define		EE_ThirdPage_Address		0xA4//����ҳ��β׺C�ͣ�
#define		EE_FourthPage_Address		0xA6//����ҳ��β׺C�ͣ�
#define		EE_FifthPage_Address		0xA8//����ҳ��β׺C�ͣ�
#define		EE_SixthPage_Address		0xAA//����ҳ��β׺C�ͣ�
#define		EE_SeventhPage_Address		0xAC//����ҳ��β׺C�ͣ�
#define		EE_EighthPage_Address		0xAE//�ڰ�ҳ��β׺C�ͣ�

#define		true  			1
#define 	false 			0

/**********************************************************************
*����ֵ����
***********************************************************************/
#define true  1
#define false 0
static void I2C_GPIO_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 

	/* ʹ���� I2C1 �йص�ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);  
    
  /* PB6-I2C1_SCL��PB7-I2C1_SDA*/
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;	       // ��©���
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}


//************************************************************
// ʵʱʱ��SD24XX��дC51��ʾ����
// AT89S52   11.0592MHz
// E-mail:   FAE@whwave.com.cn
// TEL��		 0755-83114387
// Last update:                   2014/11/14
//************************************************************

void SD2405_GPIO_Config(void);
/*********I2C��ʱ4us***********/
void I2CWait(void);
/********����SD24XX��I2C����********/
uchar I2CStart(void);
/********�ر�SD24XX��I2C����*******/
void I2CStop(void);
/*********���� ACK*********/
void I2CAck(void);
/*********����NO ACK*********/
void I2CNoAck(void);
/*********��ȡACK�ź�*********/
uchar I2CWaitAck(void);
/************MCU��SD24XX����һ���ֽ�*************/
void I2CSendByte(uchar demand);
/*********MCU��SD24XX����һ�ֽ�*********/
uchar I2CReceiveByte(void);
/******I2Cдһ���ֽ�******/
uchar I2CWriteOneByte(uchar DeviceAddress,uchar add, uchar date);
/******I2C��һ���ֽڳ���******/
uchar I2CReadOneByte(uchar DeviceAddress,uchar add);
/******дSD24XX�������******/
uchar WriteTimeOn(void);
/******дSD24XX��ֹ����******/
uchar WriteTimeOff(void);
/******��SD24XXʵʱ���ݼĴ���******/
uchar I2CReadDate(S_Time	*psRTC);
/******дSD24XXʵʱ���ݼĴ���******/
uchar I2CWriteDate(S_Time	SetRTC);
/******����SD24XX�����ж���ʾ������ʾ******/
void WriteALARM(void);
/******����SD24XX����ʱ�ж���ʾ******/
void SetDjs(void);
/******�ر�SD24XX����ʱ�жϳ���******/
void ClrDjs(void) ;
/******����SD24XXƵ���ж���ʾ******/
void SetFrq(void);
/******�ر�SD24XXƵ���ж�******/
void ClrFrq(void);


#endif /* __I2C_H */
