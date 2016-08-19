#ifndef __LED_SEG_H
#define	__LED_SEG_H

#include "stm32f10x.h"

/** the macro definition to trigger the led on or off 
  * 1 - off
  *0 - on
  */
#define ON  1
#define OFF 0

/* ���κ꣬��������������һ��ʹ�� */
#define SPI_SEG_CS(a)	if (a)	\
					GPIO_SetBits(GPIOA,GPIO_Pin_4);\
					else		\
					GPIO_ResetBits(GPIOA,GPIO_Pin_4)
					
#define SPI_SEG_SCK(a)	if (a)	\
					GPIO_SetBits(GPIOA,GPIO_Pin_5);\
					else		\
					GPIO_ResetBits(GPIOA,GPIO_Pin_5)
#define SPI_SEG_MOSI(a)	if (a)	\
					GPIO_SetBits(GPIOA,GPIO_Pin_7);\
					else		\
					GPIO_ResetBits(GPIOA,GPIO_Pin_7)	

/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			{p->BSRR=i;}			//����Ϊ�ߵ�ƽ		
#define digitalLo(p,i)			{p->BRR=i;}				//����͵�ƽ
#define digitalToggle(p,i)		{p->ODR ^=i;}			//�����ת״̬


/* �������IO�ĺ� */
#define SPI_SEG_CS_TOGGLE		digitalToggle(GPIOA,GPIO_Pin_4)
#define SPI_SEG_CS_H		    digitalHi(GPIOA,GPIO_Pin_4)
#define SPI_SEG_CS_L			digitalLo(GPIOA,GPIO_Pin_4)

#define SPI_SEG_SCK_TOGGLE		digitalToggle(GPIOA,GPIO_Pin_5)
#define SPI_SEG_SCK_H		    digitalHi(GPIOA,GPIO_Pin_5)
#define SPI_SEG_SCK_L			  digitalLo(GPIOA,GPIO_Pin_5)

#define SPI_SEG_MOSI_TOGGLE		digitalToggle(GPIOA,GPIO_Pin_7)
#define SPI_SEG_MOSI_H		    digitalHi(GPIOA,GPIO_Pin_7)
#define SPI_SEG_MOSI_L				digitalLo(GPIOA,GPIO_Pin_7)
void SPI_SEG_Config(void);
void WriteCmdCode(unsigned char sdate, unsigned char cnt);  //��λ��д
void WriteDate(unsigned char sdate, unsigned char cnt);   //��λ
void WriteCmd(unsigned char cmd);
void WriteEnCmd(unsigned char cmd);
void WriteOneDate(unsigned char saddr, unsigned char sdate);
void WriteSameDate(unsigned char saddr, unsigned char sdate, unsigned char cnt);
void TM1681Init(void);
void TM1681PerSeg(void);
void Writeclose(unsigned char saddr);
#endif /* __LED_SEG_H */
