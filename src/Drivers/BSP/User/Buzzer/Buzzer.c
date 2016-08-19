/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   ledӦ�ú����ӿ�
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� iSO STM32 ������ 
  * ��̳    :http://www.chuxue123.com
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "Buzzer.h"   
extern void Delay_1(__IO uint32_t nCount);
 /**
  * @brief  ��ʼ������LED��IO
  * @param  ��
  * @retval ��
  */
void BUZ_GPIO_Config(void)
{		
		/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
		GPIO_InitTypeDef GPIO_InitStructure;

		/*����GPIOD��GPIOC������ʱ��*/
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
			/*ѡ��Ҫ���Ƶ�GPIOD����*/															   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	//���D2

		/*��������ģʽΪͨ���������*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*������������Ϊ50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*���ÿ⺯������ʼ��GPIOD0*/
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_ResetBits(GPIOB, GPIO_Pin_5);	
		
}
void buzzer(unsigned char data)
{
			switch(data)
				{
					case 1:		BUZ_1(ON);
										Delay_1(0xfffff);
										BUZ_1(OFF);
										data=0;
										break;
					case 2:		BUZ_1(ON);
										Delay_1(0xfffff);
										BUZ_1(OFF);
										break;
				}
}

/*********************************************END OF FILE**********************/
