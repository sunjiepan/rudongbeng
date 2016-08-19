
#include "i2c.h"
#include "SysTick.h"
/**
  * @brief  I2C1 I/O����
  * @param  ��
  * @retval ��
  */

S_Time  S_RTC={0x55,0x59,0x14,0x01,0x12,0x11,0x14};//��ʼ��ʱ��ṹ�����������ʱ�䣺2014��11��12�� 14:59:55  ����һ��
//           55�� 59�� 14ʱ ��һ 10�� 11�� 14��

 void SD2405_GPIO_Config(void)
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

static void SD2405_Mode_IPU(void)
{
 	  GPIO_InitTypeDef GPIO_InitStructure;

	  	/*ѡ��Ҫ���Ƶ�DS18B20_PORT����*/	
	  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_7;;

	   /*��������ģʽΪ��������ģʽ*/ 
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	

	  /*���ÿ⺯������ʼ��DS18B20_PORT*/
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
}


static void SD2405_Mode_Out_PP(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;

	 	/*ѡ��Ҫ���Ƶ�DS18B20_PORT����*/															   
  	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_7;;	

	/*��������ģʽΪͨ���������*/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*������������Ϊ50MHz */   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/*���ÿ⺯������ʼ��DS18B20_PORT*/
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
}



/*********I2C��ʱ4us***********/
void I2CWait(void)//4us
{	
	Delay_us(4);
}

/********����SD24XX��I2C����********/
uchar I2CStart(void)
{
	SDA_H;
	SCL_H;
	I2CWait();
	SD2405_Mode_IPU();
	if(!SDA_IN)return false;	//SDA��Ϊ�͵�ƽ������æ,�˳�
	SD2405_Mode_Out_PP();
	SDA_L;
	I2CWait();
	SD2405_Mode_IPU();
	while(!SDA_IN)return false;	//SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
	SCL_L;
	I2CWait();
	return true;
}

/********�ر�SD24XX��I2C����*******/
void I2CStop(void)
{
	SD2405_Mode_Out_PP();
	SDA_L;
	SCL_L;
	I2CWait();
	SCL_H;
	I2CWait();
	SDA_H;
}

/*********���� ACK*********/
void I2CAck(void)
{	
	SD2405_Mode_Out_PP();
	SDA_L;
	SCL_L;
	I2CWait();
	SCL_H;
	I2CWait();
	SCL_L;
}

/*********����NO ACK*********/
void I2CNoAck(void)
{	
	SD2405_Mode_Out_PP();
	SDA_H;
	SCL_L;
	I2CWait();
	SCL_H;
	I2CWait();
	SCL_L;
}

/*********��ȡACK�ź�*********/
uchar I2CWaitAck(void) 	 //����Ϊ:1=��ACK,0=��ACK
{
	SD2405_Mode_Out_PP();
	SCL_L;
	SDA_H;		//����SDAΪ���루�������͵ĵ�Ƭ����Ҫ����IO��������Ĵ�����
	I2CWait();
	SCL_H;
	I2CWait();
	SD2405_Mode_IPU();
	while(SDA_IN)
	{
		SCL_L;
		return false;
	}
	SCL_L;
	return true;
}

/************MCU��SD24XX����һ���ֽ�*************/
void I2CSendByte(uchar demand) //���ݴӸ�λ����λ//
{
	uchar i=8;                       
	                                 
	
	while(i--)
	{
		SD2405_Mode_Out_PP();
		SCL_L;
		Delay_us(1);
		if(demand&0x80)	{SDA_H;}	else {SDA_L;}
		demand<<=1;
		I2CWait();
		SCL_H;
		I2CWait();
	}
	SCL_L;
}

/*********MCU��SD24XX����һ�ֽ�*********/
uchar I2CReceiveByte(void)      //���ݴӸ�λ����λ//
{
	uchar i=8;
	uchar ddata=0;
	SDA_H;			//����SDAΪ���루�������͵ĵ�Ƭ����Ҫ����IO��������Ĵ�����
	while(i--)
	{
		ddata<<=1;      //���ݴӸ�λ��ʼ��ȡ
		SCL_L;
		I2CWait();
		SCL_H;
		I2CWait();	//�Ӹ�λ��ʼ ddata|=SDA;ddata<<=1
		if(SDA_IN)
		{
			ddata|=0x01;
		}
	}
	SCL_L;
	return ddata;
}

/******I2Cдһ���ֽ�******/
uchar I2CWriteOneByte(uchar DeviceAddress,uchar add, uchar date)
{		
	if(!I2CStart())return false;
	I2CSendByte(DeviceAddress);      
	I2CWaitAck();   
	I2CSendByte(add);		//����д��ַ      
	I2CWaitAck();	
	I2CSendByte(date);		//д����
	I2CWaitAck();	
	I2CStop(); 
	return	true;
}

/******I2C��һ���ֽڳ���******/
uchar I2CReadOneByte(uchar DeviceAddress,uchar add)
{		
	uchar dat;
	if(!I2CStart())return false;
	I2CSendByte(DeviceAddress);      
	if(!I2CWaitAck()){I2CStop(); return false;}
  	I2CSendByte(add);		//����Ҫ���ĵ�ַ
	I2CWaitAck();
	I2CStart();	
 	I2CSendByte(DeviceAddress+1);	
	I2CWaitAck();	
	dat=I2CReceiveByte();		//������
	I2CNoAck();
	I2CStop(); 
	return  dat;
}

/******дSD24XX�������******/
uchar WriteTimeOn(void)
{		
	if(!I2CWriteOneByte(RTC_Address,0x10,0x80))return false;
	I2CWriteOneByte(RTC_Address,0x0f,0x84);
	return	true;
}

/******дSD24XX��ֹ����******/
uchar WriteTimeOff(void)
{		
	if(!I2CWriteOneByte(RTC_Address,0x0f,0))return false;
	I2CWriteOneByte(RTC_Address,0x10,0);
	return	true;
}

/******��SD24XXʵʱ���ݼĴ���******/
uchar I2CReadDate(S_Time	*psRTC)
{
	
	if(!I2CStart())return false;
	I2CSendByte(RTC_Address+1); 
    if(!I2CWaitAck()){I2CStop(); return false;}
	psRTC->second=I2CReceiveByte();
	I2CAck();
	psRTC->minute=I2CReceiveByte();
	I2CAck();
	psRTC->hour=I2CReceiveByte();
	I2CAck();
	psRTC->week=I2CReceiveByte();
	I2CAck();
	psRTC->day=I2CReceiveByte();
	I2CAck();
	psRTC->month=I2CReceiveByte();
	I2CAck();
	psRTC->year=I2CReceiveByte();
	I2CNoAck();		//��ʱ����ɣ�����NoAck
	I2CStop();
	return	true;
}

/******дSD24XXʵʱ���ݼĴ���******/
uchar I2CWriteDate(S_Time	SetRTC)	//дʱ�����Ҫ��һ�ζ�ʵʱʱ��Ĵ���(00H~06H)����д�룬
{                               //�����Ե�����7��ʱ�������е�ĳһλ����д����,������ܻ�����ʱ�����ݵĴ����λ. 
                                //Ҫ�޸�����ĳһ������ , Ӧһ����д��ȫ�� 7 ��ʵʱʱ������.
	S_Time 	*psRTC;
    psRTC=&SetRTC;
	WriteTimeOn();				//ʹ�ܣ�����
	if(!I2CStart())return false;
	I2CSendByte(RTC_Address); 
	if(!I2CWaitAck()){I2CStop(); return false;}
	I2CSendByte(0x00);			//����д��ʼ��ַ      
	I2CWaitAck();	
	I2CSendByte(psRTC->second);		//second     
	I2CWaitAck();	
	I2CSendByte(psRTC->minute);		//minute      
	I2CWaitAck();	
	I2CSendByte(psRTC->hour|0x80);		//hour ,ͬʱ����Сʱ�Ĵ������λ��0��Ϊ12Сʱ�ƣ�1��Ϊ24Сʱ�ƣ�
	I2CWaitAck();	
	I2CSendByte(psRTC->week);		//week      
	I2CWaitAck();	
	I2CSendByte(psRTC->day);		//day      
	I2CWaitAck();	
	I2CSendByte(psRTC->month);		//month      
	I2CWaitAck();	
	I2CSendByte(psRTC->year);		//year      
	I2CWaitAck();	
	I2CStop();
	
	WriteTimeOff();				//ʹ�ܣ�����
	return	true;
}


/******����SD24XX�����ж���ʾ������ʾ******/
void WriteALARM(void)				//���ñ���ʱ�䣺2015��2��14�� 8��00
{						//ֻ������δ����ʱ�����Ч
	WriteTimeOn();
	I2CWriteOneByte(RTC_Address,0x09,0x08);	//8ʱ
	I2CWriteOneByte(RTC_Address,0x0b,0x14);	//14��
	I2CWriteOneByte(RTC_Address,0x0c,0x02);	//02��
	I2CWriteOneByte(RTC_Address,0x0d,0x15);	//15��
	I2CWriteOneByte(RTC_Address,0x0e,0x74);	//���ñ�������ʹ���ꡢ�¡��ա�Сʱ������
	I2CWriteOneByte(RTC_Address,0x10,0x92);	//����INT�ж�ѡͨ��INTS1��INTS0�����������ж�������λ��INTAE��
	WriteTimeOff();
}

/******�ر�SD24XX�����жϳ���******/
void ClrALARM(void)							//�رձ����ж�
{
	WriteTimeOn();
	I2CWriteOneByte(RTC_Address,0x10,0x90);
	WriteTimeOff();
}
/******����SD24XX����ʱ�ж���ʾ******/
void SetDjs(void)							//���õ���ʱ�ж�
{
	WriteTimeOn();
	I2CWriteOneByte(RTC_Address,0x10,0x0f);//���嵹��ʱ�ж�������λ��INTDE��
	I2CWriteOneByte(RTC_Address,0x10,0xf4);//�����������жϣ�IM=1��INT�ж�ѡͨ��INTS1��INTS0�������õ���ʱ�ж�������λ��INTDE��
	I2CWriteOneByte(RTC_Address,0x11,0x30);//ѡ��ʱ��Ƶ��Դ��TDS1��TDS0��Ϊ1/60HZ
	I2CWriteOneByte(RTC_Address,0x13,0x05);//����ʱ��ֵ�Ĵ���������8λ����ʱ������ֵ��5min��
	WriteTimeOff();
}

/******�ر�SD24XX����ʱ�жϳ���******/
void ClrDjs(void)          
{
	WriteTimeOn();
	I2CWriteOneByte(RTC_Address,0x10,0xf0);
	WriteTimeOff();
}
/******����SD24XXƵ���ж���ʾ******/
void SetFrq(void)					
{
	WriteTimeOn();
	I2CWriteOneByte(RTC_Address,0x10,0xa1);	//ѡͨƵ���жϣ�INTS1��INTS0��������Ƶ���ж�������λ��INTFE��
	I2CWriteOneByte(RTC_Address,0x11,0x09);	//����2HzƵ���ж�
	WriteTimeOff();
}
/******�ر�SD24XXƵ���ж�******/
void ClrFrq(void)         
{
	WriteTimeOn();
	I2CWriteOneByte(RTC_Address,0x10,0xa0);
	WriteTimeOff();
}

//�ر����ѣ���дʵʱʱ������ʱ (00H~06H), �����Ե��� �� 7 ��ʱ�������е�ĳһλ����д���� ,
//������ܻ�����ʱ�����ݵĴ����λ , ����Ҫ�޸�����ĳһ������ , Ӧһ����д��ȫ�� 7 ��ʵʱʱ������ .
/*********************************************END OF FILE**********************/
