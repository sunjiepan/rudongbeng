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
  
#include "bsp_led.h"   

 /**
  * @brief  ��ʼ������LED��IO
  * @param  ��
  * @retval ��
  */
void LED_GPIO_Config(void)
{		
		/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
		GPIO_InitTypeDef GPIO_InitStructure;

		/*����GPIOD��GPIOC������ʱ��*/
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOG, ENABLE); 

		/*ѡ��Ҫ���Ƶ�GPIOD����*/															   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;	//���D2

		/*��������ģʽΪͨ���������*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*������������Ϊ50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*���ÿ⺯������ʼ��GPIOD0*/
		GPIO_Init(GPIOG, &GPIO_InitStructure);	
		
		/*ѡ��Ҫ���Ƶ�GPIOC����*/															   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; //���D5

		/*���ÿ⺯������ʼ��GPIOF7*/
		GPIO_Init(GPIOD, &GPIO_InitStructure);
		
				  

		/* �ر�����led��	*/
		GPIO_SetBits(GPIOD, GPIO_Pin_13);
		
		/* �ر�����led��	*/
		GPIO_SetBits(GPIOG, GPIO_Pin_14);	 
}
/*********************************************END OF FILE**********************/
