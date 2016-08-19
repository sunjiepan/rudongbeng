#include "TM1681.h"
#include "delay.h"
#include "usart.h"
#include "Buzzer.h" 
#include "SysTick.h"
/** Write multiple bits in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 */
static unsigned char temp;
static unsigned char x=1;
//unsigned char channel_1=1;
unsigned char Enable_1,Enable_2,Enable_3,Enable_4,Enable_5,Enable_6;
extern unsigned char channel;
extern unsigned char NumDis[24],TIME[10],G_1,K1;
extern int16_t cache[6],data[6];
 //__IO u32 Time_1;
extern __IO u32 Total_time;
void TM1637_Delay_us(unsigned  int Nus)
{
 for(;Nus>0;Nus--)
	{
	  __nop();
	}
}

void TM1637_Start(void)
{
	TM1637_DIO_OUT(); 
  TM1637_CLK (1);
	TM1637_OUT_DIO1 = 1;
	TM1637_Delay_us(2);
	TM1637_OUT_DIO1 = 0;
	TM1637_Delay_us(2);
	TM1637_CLK ( 0);
}
void TM1637_Ack(void)
{
	TM1637_DIO_IN(); 
  TM1637_CLK ( 0);
	TM1637_Delay_us(5);
	while(TM1637_READ_DIO1);
	TM1637_CLK (1);
	TM1637_Delay_us(2);
}
void TM1637_Stop(void)
{
	TM1637_DIO_OUT(); 
	TM1637_CLK ( 0);
	TM1637_Delay_us(2);
	TM1637_OUT_DIO1 = 0;
	TM1637_Delay_us(2);
	TM1637_CLK (1);
	TM1637_Delay_us(2);
	TM1637_OUT_DIO1 = 1;
	TM1637_Delay_us(2);
	TM1637_CLK( 0);
	TM1637_OUT_DIO1 = 0;
}
void TM1637_WriteByte(unsigned char oneByte)
{
 unsigned char i;
	TM1637_DIO_OUT(); 
	for(i=0;i<8;i++)
	{
	 TM1637_CLK(0);
		if(oneByte&0x01)
		{
			TM1637_OUT_DIO1 = 1;
		}
		else
		{
			TM1637_OUT_DIO1 = 0;
		}
		TM1637_Delay_us(3);
		TM1637_CLK(1);
		oneByte=oneByte>>1;
	}
}

void TM1637_DisplayChar_1(unsigned char ch,unsigned char p)
{
	if(ch>23)ch=0;//��ֹ����Խ��
	TM1637_Start();
	TM1637_WriteByte(0x44);//0x44�̶�ģʽ�����ڿ�����ʾλ��0x40�������ʾ��ַ�Լ�ģʽ���ﲻʹ��
	TM1637_Ack();
	TM1637_Stop();
	TM1637_Start();
	
	TM1637_WriteByte(0xC0+p);//0X00��ַ��ʼ��ʾ
	TM1637_Ack();
		
	TM1637_WriteByte(NumDis[ch]);//��ʾ
	TM1637_Ack();
	
	TM1637_Stop();
	TM1637_Start();
	TM1637_WriteByte(0x8C);
	TM1637_Ack();
	TM1637_Stop();
		
}
void TM1637_Display_1(void)
{
	unsigned char a=0,b=0,c=0,d=0;
	if(cache[0]<0)
	{
		d=((-cache[0])%60)%10;
		c=((-cache[0])%60)/10;
		b=((-cache[0])/60)%10;
		a=0x40;
	}
	else
	{
	d=(cache[0]%60)%10;
	c=(cache[0]%60)/10;
	b=(cache[0]/60)%10;
	a=(cache[0]/60)/10;
	}
	TM1637_Start();
	TM1637_WriteByte(0x40);//0x40�������ʾ��ַ�Լ�1ģʽ
	TM1637_Ack();
	TM1637_Stop();
	TM1637_Start();
	TM1637_WriteByte(0xC0);//0X00��ַ��ʼ��ʾ
	TM1637_Ack();
		
	TM1637_WriteByte(TIME[a]);//��ʾ1
	TM1637_Ack();
	if(x==1)
	{
		temp=TIME[b];
		x++;
	}
  temp^=0x80;  //10000000^11001111=01001111;01001111^10000000=11001111
	TM1637_WriteByte(temp);//��ʾ2��������ð��
	TM1637_Ack();
	
	TM1637_WriteByte(TIME[c]);//��ʾ0
	TM1637_Ack();
		
	TM1637_WriteByte(TIME[d]);//��ʾ0
	TM1637_Ack();
		
	TM1637_WriteByte(0xFF);
	TM1637_Ack();
		
	TM1637_WriteByte(0xFF);
	TM1637_Ack();
	
	TM1637_Stop();
	TM1637_Start();
	TM1637_WriteByte(0x8c);
	TM1637_Ack();
	TM1637_Stop();
}

void time_1(void)
{
	
	if(x>=2)
			{
						if((Enable_1==1)&&(channel==0))
						{
								G_1=1;
								cache[0]--;
								if(cache[0]<0)
								{
									if((-cache[0])>599)
									{
										Enable_1=0;
									}
									K1=1;				
								}	
							}
				x=1;
			}
}

void Digital_off_1(void)
{
	TM1637_Start();
	TM1637_WriteByte(0x88);
	TM1637_Ack();
	TM1637_Stop();
	
}

