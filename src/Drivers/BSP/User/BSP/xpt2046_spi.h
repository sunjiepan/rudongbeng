#ifndef __XPT2046_SPI_H
#define __XPT2046_SPI_H
#include "stm32f10x_conf.h"
//XPT2046 ��STM32107VCT6�����߷�ʽ
//MISO          PC11/SPI3_MISO REMAP
//MOSI          PC12/SPI3_MISO
//SCK          PC10/SPI3_SCK
//TP_CS         PC9
//INT          PC5

// A/D ͨ��ѡ�������ֺ͹����Ĵ���
//#define	CHX 	0x90
//#define	CHY 	0xD0

#define TP_CS()  GPIO_ResetBits(GPIOB,GPIO_Pin_12)
#define TP_DCS() GPIO_SetBits(GPIOB,GPIO_Pin_12)

extern u16 X_Addata,Y_Addata;
//extern void delay_ms(u16 Nms);������ʱҲ������Ŷ
void XPT2046_Configuration(void);//����GPIO/SPI3/EXTI�ж� 
u16 XPT2046_ReadX(void);
u16 XPT2046_ReadY(void);
#endif

