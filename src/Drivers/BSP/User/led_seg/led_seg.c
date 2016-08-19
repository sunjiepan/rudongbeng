/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   ledӦ�ú����ӿ�
  */
  
#include "led_seg.h"
#include "SysTick.h"
extern void Delay_1(__IO uint32_t nCount);
 /**
  * @brief  ��ʼ������LED��IO
  * @param  ��
  * @retval ��
  */
void SPI_SEG_Config(void)
{		
		/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
		GPIO_InitTypeDef GPIO_InitStructure;

		/*����GPIOD��GPIOC������ʱ��*/
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
			/*ѡ��Ҫ���Ƶ�GPIOD����*/															   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7;	

		/*��������ģʽΪͨ���������*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;   

		/*������������Ϊ50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*���ÿ⺯������ʼ��GPIOD0*/
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);	
		GPIO_ResetBits(GPIOA, GPIO_Pin_7);				
}

extern int16_t cache[7],temp[1];
extern unsigned char NumDis[10];
unsigned char perseg[4]={0x01,0x02,0x04,0x08},x=1;
unsigned char colon=0xCF;//��ʾ������ð��
void WriteCmdCode(unsigned char sdate, unsigned char cnt)  //��λ��д
{
		unsigned char i=0;
	  
		for(i=0; i<cnt; i++)
		{
			  SPI_SEG_SCK_L;
				Delay_us(4);
			  if(sdate&0x80){SPI_SEG_MOSI_H;}else{SPI_SEG_MOSI_L;}
				Delay_us(5);
				SPI_SEG_SCK_H;
				Delay_us(6);
				sdate<<=1;
		}
}

void WriteDate(unsigned char sdate, unsigned char cnt)   //��λ
{
		unsigned char i=0;
	  
		for(i=0; i<cnt; i++)
		{
			  SPI_SEG_SCK_L;
				Delay_us(4);
			  if(sdate&0x01){SPI_SEG_MOSI_H;}else{SPI_SEG_MOSI_L;}
				Delay_us(5);
				SPI_SEG_SCK_H;
				Delay_us(6);
				sdate>>=1;
		}
}


void WriteCmd(unsigned char cmd)
{
		SPI_SEG_CS_L;
		Delay_us(10);
		WriteCmdCode(0x80, 3);  // д��100
		WriteCmdCode(cmd, 8);		// д������
	  WriteCmdCode(0x00,1);		// X
		SPI_SEG_SCK_L; Delay_us(5);SPI_SEG_MOSI_L;
		Delay_us(10);
		SPI_SEG_CS_H;
}

void WriteEnCmd(unsigned char cmd)
{
	 WriteCmdCode(cmd, 8);		//д������
	 WriteCmdCode(0x00,1);		
	 SPI_SEG_SCK_L;  Delay_us(5);SPI_SEG_MOSI_L;
	 Delay_us(10);
}


void WriteOneDate(unsigned char saddr, unsigned char sdate)
{
		SPI_SEG_CS_L;
		Delay_us(10);
		WriteCmdCode(0xa0, 3);   		//д��101
		WriteCmdCode(saddr<<1,7);	 	//д���ַ
		WriteDate(sdate,4);				//д������
		SPI_SEG_SCK_L; Delay_us(5);SPI_SEG_MOSI_L;
		Delay_us(10);
		SPI_SEG_CS_H;
}

void WriteSameDate(unsigned char saddr, unsigned char sdate, unsigned char cnt)
{	
		unsigned char i=0;
		SPI_SEG_CS_L;
		Delay_us(10);
		WriteCmdCode(0xa0, 3);   		//д��101
		WriteCmdCode(saddr<<1,7);	 	//д���ַ
		for(i=0; i<cnt; i++)
		{
				WriteDate(sdate,4);	
				SPI_SEG_SCK_L; Delay_us(5);SPI_SEG_MOSI_L;
		}
		Delay_us(10);
		SPI_SEG_CS_H;
}

void Writeseg_Date(unsigned char saddr,int16_t data)
{	
		unsigned char seg_h=0,seg_l=0;
	  unsigned char a=0,b=0,c=0,d=0;
		if(data<0)
	{
		d=((-data)%60)%10;
		c=((-data)%60)/10;
		b=((-data)/60)%10;
		a=10;
	}
	else
	{
	d=(data%60)%10;
	c=(data%60)/10;
	b=(data/60)%10;
	a=(data/60)/10;
	}
	  
		
	  seg_h=(NumDis[d]&0xF0);
		seg_h>>=4;
		seg_l=(NumDis[d]&0x0F);
		WriteOneDate(saddr,seg_l);
	  saddr+=1;
	  WriteOneDate(saddr,seg_h);
		
	  saddr+=1;
		seg_h=(NumDis[c]&0xF0);
		seg_h>>=4;
		seg_l=(NumDis[c]&0x0F);
		WriteOneDate(saddr,seg_l);
	  saddr+=1;
	  WriteOneDate(saddr,seg_h);
	
		saddr+=1;
	  colon=NumDis[b];
	  if(x==0)	colon|=0x80;
		if(x==1)	colon&=0x7F;
		seg_h=(colon&0xF0);
		seg_h>>=4;
		seg_l=(colon&0x0F);
		WriteOneDate(saddr,seg_l);
	  saddr+=1;
	  WriteOneDate(saddr,seg_h);
		saddr+=1;
		
		colon=NumDis[a];
		if(x==0)	colon|=0x80;
		if(x==1)	colon&=0x7F;
		seg_h=(colon&0xF0);
		seg_h>>=4;
		seg_l=(colon&0x0F);
		WriteOneDate(saddr,seg_l);
	  saddr+=1;
	  WriteOneDate(saddr,seg_h);
}


void Writetemp_Date(unsigned char saddr,int data)
{	
		unsigned char seg_h=0,seg_l=0;
	  unsigned char b=0,c=0,d=0;

		d=data%10;
		c=(data/10)%10;
		b=data/100;
	
	  seg_h=(NumDis[d]&0xF0);
		seg_h>>=4;
		seg_l=(NumDis[d]&0x0F);
		WriteOneDate(saddr,seg_l);
	  saddr+=1;
	  WriteOneDate(saddr,seg_h);
		
	  saddr+=1;
		colon=NumDis[c];
	  colon|=0x80;
	  seg_h=(colon&0xF0);
		seg_h>>=4;
		seg_l=(colon&0x0F);
		WriteOneDate(saddr,seg_l);
	  saddr+=1;
	  WriteOneDate(saddr,seg_h);
	
		saddr+=1;
	  seg_h=(NumDis[b]&0xF0);
		seg_h>>=4;
		seg_l=(NumDis[b]&0x0F);
		WriteOneDate(saddr,seg_l);
	  saddr+=1;
	  WriteOneDate(saddr,seg_h);

}

void Writeclose(unsigned char saddr)
{	
		
		WriteOneDate(saddr,0);
	  saddr+=1;
	  WriteOneDate(saddr,0);
		
	  saddr+=1;
		WriteOneDate(saddr,0);
	  saddr+=1;
	  WriteOneDate(saddr,0);
	
		saddr+=1;
	  WriteOneDate(saddr,0);
	  saddr+=1;
	  WriteOneDate(saddr,0);
		saddr+=1;
		WriteOneDate(saddr,0);
	  saddr+=1;
	  WriteOneDate(saddr,0);
}

/***
		��������:��ʼ������
***/
void TM1681Init(void)
{
	SPI_SEG_CS_L;SPI_SEG_SCK_L; SPI_SEG_MOSI_H;
  Delay_us(10);
	WriteCmd(0x02);   //LEDOFF
	WriteCmd(0x00);		//SYSDIS
	WriteCmd(0x20);		//8com-NMOS
	WriteCmd(0x1A);   //RC
	WriteCmd(0xa6);   // һ���Զ�
	WriteCmd(0x08);   // �ر���˸
	WriteCmd(0x03);   //LEDON	
	WriteCmd(0x01);		//SYSEN
}

void TM1681PerSeg(void)
{
		unsigned char i=0,j=0;
		unsigned char saddr=0x00, m=perseg[0];
	
		for(i=0; i<54; i++)
		{
				for(j=0; j<4; j++)
				{
						m=m|perseg[j];
						WriteOneDate(saddr,m);				
					Delay_ms(100);
				}
				saddr+=1;
				m=perseg[0];
		}
}
/*********************************************END OF FILE**********************/
