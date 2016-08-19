/**
  ******************************************************************************
  * @file    bsp_exti.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   I/O���ж�Ӧ��bsp
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� iSO STM32 ������ 
  * ��̳    :http://www.chuxue123.com
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "exti.h"

 /**
  * @brief  ����Ƕ�������жϿ�����NVIC
  * @param  ��
  * @retval ��
  */
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  /* Set the Vector Table base location at 0x20000000 */ 
	//NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); //�����ж�������
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
//  /*TIM2�ж����ȼ�����*/
//		
//  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	  
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
  /* GPIOA�����ж�Դ */
	
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	
}

/**
  * @brief  ���� I/O����
  * @param  ��
  * @retval ��
  */
static void EXTI_line_GPIO_Config(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3; //ѡ������
  GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IPU; //ѡ������ģʽ��������
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;          //���Ƶ�����50MHz
  GPIO_Init(GPIOA,&GPIO_InitStructure);                //����GPIOA
	
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11| GPIO_Pin_12; //ѡ������
  GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IPU;//GPIO_Mode_IPU; //ѡ������ģʽ��������
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;          //���Ƶ�����50MHz
  GPIO_Init(GPIOA,&GPIO_InitStructure);                //����GPIOA
	
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_3|GPIO_Pin_4; //ѡ������
  GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IPU; //ѡ������ģʽ��������
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;          //���Ƶ�����50MHz
  GPIO_Init(GPIOB,&GPIO_InitStructure);                //����GPIOB
	
}
 /**
  * @brief  ���� PA0 Ϊ���жϿڣ��������ж����ȼ�
  * @param  ��
  * @retval ��
  */
void EXTI_GPIO_Config(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	/* config the extiline clock and AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
	
	EXTI_line_GPIO_Config();									
	/* config the NVIC */
	NVIC_Configuration();

	/* EXTI line mode config */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3);
  
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource9);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource10);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource11);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource12);
	
	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);

  EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line1|EXTI_Line2|EXTI_Line3|EXTI_Line4;//ѡ����·
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//����Ϊ�ж����󡬷��¼�����
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //˫�����ж�
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure); 
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line8|EXTI_Line9|EXTI_Line10|EXTI_Line11|EXTI_Line12;//ѡ����·
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//����Ϊ�ж����󡬷��¼�����
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½����ж�
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure); 
}
/*********************************************END OF FILE**********************/
