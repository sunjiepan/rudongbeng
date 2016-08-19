#ifndef __BUZZER_H
#define	__BUZZER_H

#include "stm32f10x.h"

/** the macro definition to trigger the led on or off 
  * 1 - off
  *0 - on
  */
#define ON  1
#define OFF 0

/* ���κ꣬��������������һ��ʹ�� */
#define BUZ_1(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_5);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_5)
					
#define TM1637_CLK(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_0);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_0)
	

/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			{p->BSRR=i;}			//����Ϊ�ߵ�ƽ		
#define digitalLo(p,i)			{p->BRR=i;}				//����͵�ƽ
#define digitalToggle(p,i)		{p->ODR ^=i;}			//�����ת״̬


/* �������IO�ĺ� */
#define BUZ_TOGGLE		digitalToggle(GPIOB,GPIO_Pin_5)
#define BUZ_H		digitalHi(GPIOB,GPIO_Pin_5)
#define BUZ_L			digitalLo(GPIOB,GPIO_Pin_5)

#define LED2_TOGGLE		digitalToggle(GPIOF,GPIO_Pin_7)
#define LED2_H		digitalHi(GPIOF,GPIO_Pin_7)
#define LED2_L			digitalLo(GPIOF,GPIO_Pin_7)

#define LED3_TOGGLE		digitalToggle(GPIOF,GPIO_Pin_8)
#define LED3_H		digitalHi(GPIOF,GPIO_Pin_8)
#define LED3_L			digitalLo(GPIOF,GPIO_Pin_8)

void BUZ_GPIO_Config(void);
void buzzer(unsigned char data);

#endif /* __BUZZER_H */
