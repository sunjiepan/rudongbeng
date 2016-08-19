#include "xpt2046_spi.h"

static void XPT2046_INT_EXIT_Init(void)//�ж������½���
{
    EXTI_InitTypeDef EXTI_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);		//ʹ��IO���ù��ܣ�ʹ���жϹ�����Ҫ������
    /* Connect Button EXTI Line to Button GPIO Pin */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource7);  

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = EXTI_Line7;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж�ģʽΪ�ж�ģʽ
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  //�½��س���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; //ʹ���ж���
    EXTI_Init(&EXTI_InitStructure);  //���ݲ�����ʼ���жϼĴ���

}


/*******************************************************************************
* Function Name  : InterruptConfig
* Description    : Configures the used IRQ Channels and sets their priority.NVIC_Configuration
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void XPT2046_InterruptConfig(void)//NVIC
{ 
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Set the Vector Table base address at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
  
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  /* Enable the EXTI5 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; //�趨�ж�ԴΪPG7
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//�ж�ռ���ȼ�Ϊ1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//�����ȼ�Ϊ0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ж�
  NVIC_Init(&NVIC_InitStructure);//���ݲ�����ʼ���жϼĴ���
}
void XPT2046_Configuration(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable SPI3and GPIO clocks */

  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);   
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF,ENABLE);
		
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE ) ;
  //Configure PB2 pin: TP_CS pin 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
	// Set as Output AF push-pull - SCK and MOSI PC10 PC12//SPI3_MISO PC11    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  //�ж϶˿�GPIO���� PG7
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//��������
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//����
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  //SPI3 ����
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;//����˵�����������2M //72/256=
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);
  /* Enable SPI2  */
  SPI_Cmd(SPI2, ENABLE);
  //�ж�����
  XPT2046_INT_EXIT_Init();
  XPT2046_InterruptConfig();
  //GPIO_SetBits(GPIOC, GPIO_Pin_12);//mosi=1
  //clk=0
}  
u8 XPT2046_WriteByte(u8 byte)
{
	unsigned char Data = 0; 
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

  /* Send Half Word through the SPI3 peripheral */
  SPI_I2S_SendData(SPI2, byte);

  /* Wait to receive a Half Word */
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

  /* Get the received data  */
	Data = SPI_I2S_ReceiveData(SPI2);
	 /* Return the Half Word read from the SPI bus */
  return Data;
}

void SpiDelay(unsigned int DelayCnt)
{
		unsigned int i;
		for(i=0;i<DelayCnt;i++);
}

u16 XPT2046_ReadX(void)
{ 
	   u16 x=0;
	   TP_CS();
	   SpiDelay(10);
	   XPT2046_WriteByte(0x90);
	   SpiDelay(10);
	   x=XPT2046_WriteByte(0x00);
	   x<<=8;
	   x+=XPT2046_WriteByte(0x00);
	   SpiDelay(10);
	   TP_DCS(); 
	   x = x>>3;
	   return (x);
}

u16 XPT2046_ReadY(void)
{
	  u16 y=0;
	  TP_CS();
	  SpiDelay(10);
	  XPT2046_WriteByte(0xD0);
	  SpiDelay(10);
	  y=XPT2046_WriteByte(0x00);
	  y<<=8;
	  y+=XPT2046_WriteByte(0x00);
	  SpiDelay(10);
	  TP_DCS();
	  y = y>>3; 
	  return (y);
}
