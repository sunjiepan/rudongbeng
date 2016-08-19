#include "TM1637.h"
#include "delay.h"
#include "usart.h"
#include "Buzzer.h" 
/** Write multiple bits in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 */
static unsigned char temp;
static unsigned char x=1;
//unsigned char channel_3;
unsigned char Enable_3;
extern unsigned char channel;
extern unsigned char NumDis[24],TIME[10],G_3,K1;
extern int16_t cache[6],data[6];
void TM1637_Start_3(void)
{
	TM1637_DIO_OUT(); 
  TM1637_CLK (1);
	TM1637_OUT_DIO3 = 1;
	TM1637_Delay_us(2);
	TM1637_OUT_DIO3 = 0;
	TM1637_Delay_us(2);
	TM1637_CLK ( 0);
}
void TM1637_Ack_3(void)
{
	TM1637_DIO_IN(); 
  TM1637_CLK ( 0);
	TM1637_Delay_us(5);
	while(TM1637_READ_DIO3);
	TM1637_CLK (1);
	TM1637_Delay_us(2);
}
void TM1637_Stop_3(void)
{
	TM1637_DIO_OUT(); 
	TM1637_CLK ( 0);
	TM1637_Delay_us(2);
	TM1637_OUT_DIO3 = 0;
	TM1637_Delay_us(2);
	TM1637_CLK (1);
	TM1637_Delay_us(2);
	TM1637_OUT_DIO3 = 1;
	TM1637_Delay_us(2);
	TM1637_CLK( 0);
	TM1637_OUT_DIO3 = 0;
}
void TM1637_WriteByte_3(unsigned char oneByte)
{
 unsigned char i;
	TM1637_DIO_OUT(); 
	for(i=0;i<8;i++)
	{
	 TM1637_CLK(0);
		if(oneByte&0x01)
		{
			TM1637_OUT_DIO3 = 1;
		}
		else
		{
			TM1637_OUT_DIO3 = 0;
		}
		TM1637_Delay_us(3);
		TM1637_CLK(1);
		oneByte=oneByte>>1;
	}
}

void TM1637_DisplayChar_3(unsigned char ch,unsigned char p)
{
	if(ch>23)ch=0;//��ֹ����Խ��
	TM1637_Start_3();
	TM1637_WriteByte_3(0x44);//0x44�̶�ģʽ�����ڿ�����ʾλ��0x40�������ʾ��ַ�Լ�ģʽ���ﲻʹ��
	TM1637_Ack_3();
	TM1637_Stop_3();
	TM1637_Start_3();
	
	TM1637_WriteByte_3(0xC0+p);//0X00��ַ��ʼ��ʾ
	TM1637_Ack_3();
		
	TM1637_WriteByte_3(NumDis[ch]);//��ʾ
	TM1637_Ack_3();
	
	TM1637_Stop_3();
	TM1637_Start_3();
	TM1637_WriteByte_3(0x8C);
	TM1637_Ack_3();
	TM1637_Stop_3();
		
}
void TM1637_Display_3(void)
{
	unsigned char a=0,b=0,c=0,d=0;
	if(cache[2]<0)
	{
		d=((-cache[2])%60)%10;
		c=((-cache[2])%60)/10;
		b=((-cache[2])/60)%10;
		a=0x40;
	}
	else
	{
	d=(cache[2]%60)%10;
	c=(cache[2]%60)/10;
	b=(cache[2]/60)%10;
	a=(cache[2]/60)/10;
	}
	TM1637_Start_3();
	TM1637_WriteByte_3(0x40);//0x40�������ʾ��ַ�Լ�1ģʽ
	TM1637_Ack_3();
	TM1637_Stop_3();
	TM1637_Start_3();
	TM1637_WriteByte_3(0xC0);//0X00��ַ��ʼ��ʾ
	TM1637_Ack_3();
		
	TM1637_WriteByte_3(TIME[a]);//��ʾ1
	TM1637_Ack_3();
	if(x==1)
	{
		temp=TIME[b];
		x++;
	}  
  temp^=0x80;  //10000000^11001111=01001111;01001111^10000000=11001111
	TM1637_WriteByte_3(temp);//��ʾ2��������ð��
	TM1637_Ack_3();
	
	TM1637_WriteByte_3(TIME[c]);//��ʾ1
	TM1637_Ack_3();
		
	TM1637_WriteByte_3(TIME[d]);//��ʾ4
	TM1637_Ack_3();
		
	TM1637_WriteByte_3(0xFF);
	TM1637_Ack_3();
		
	TM1637_WriteByte_3(0xFF);
	TM1637_Ack_3();
	
	TM1637_Stop_3();
	TM1637_Start_3();
	TM1637_WriteByte_3(0x8C);
	TM1637_Ack_3();
	TM1637_Stop_3();
}

void time_3(void)
{
	if(x>=2)
			{
				if((Enable_3==1)&&(channel==0))
					{	
						G_3=1;
						cache[2]--;
						if(cache[2]<0)
						{
							if((-cache[2])>599)
							{
								Enable_3=0;
								
							}
							K1=1;
						}	
					}
				x=1;
			}
}
void Digital_off_3(void)
{
	TM1637_Start_3();
	TM1637_WriteByte_3(0x88);
	TM1637_Ack_3();
	TM1637_Stop_3();
	
}
