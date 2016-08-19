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
//unsigned char channel_6;
unsigned char Enable_6;
extern unsigned char channel;
extern unsigned char NumDis[24],TIME[10],G_6,K1;
extern int16_t cache[6],data[6];

void TM1637_Start_6(void)
{
	TM1637_DIO_OUT(); 
  TM1637_CLK (1);
	TM1637_OUT_DIO6 = 1;
	TM1637_Delay_us(2);
	TM1637_OUT_DIO6 = 0;
	TM1637_Delay_us(2);
	TM1637_CLK ( 0);
}
void TM1637_Ack_6(void)
{
	TM1637_DIO_IN(); 
  TM1637_CLK ( 0);
	TM1637_Delay_us(5);
	while(TM1637_READ_DIO6);
	TM1637_CLK (1);
	TM1637_Delay_us(2);
}
void TM1637_Stop_6(void)
{
	TM1637_DIO_OUT(); 
	TM1637_CLK ( 0);
	TM1637_Delay_us(2);
	TM1637_OUT_DIO6 = 0;
	TM1637_Delay_us(2);
	TM1637_CLK (1);
	TM1637_Delay_us(2);
	TM1637_OUT_DIO6 = 1;
	TM1637_Delay_us(2);
	TM1637_CLK( 0);
	TM1637_OUT_DIO6 = 0;
}
void TM1637_WriteByte_6(unsigned char oneByte)
{
 unsigned char i;
	TM1637_DIO_OUT(); 
	for(i=0;i<8;i++)
	{
	 TM1637_CLK(0);
		if(oneByte&0x01)
		{
			TM1637_OUT_DIO6 = 1;
		}
		else
		{
			TM1637_OUT_DIO6= 0;
		}
		TM1637_Delay_us(3);
		TM1637_CLK(1);
		oneByte=oneByte>>1;
	}
}


void TM1637_DisplayChar_6(unsigned char ch,unsigned char p)
{
	if(ch>23)ch=0;//��ֹ����Խ��
	TM1637_Start_6();
	TM1637_WriteByte_6(0x44);//0x44�̶�ģʽ�����ڿ�����ʾλ��0x40�������ʾ��ַ�Լ�ģʽ���ﲻʹ��
	TM1637_Ack_6();
	TM1637_Stop_6();
	TM1637_Start_6();
	
	TM1637_WriteByte_6(0xC0+p);//0X00��ַ��ʼ��ʾ
	TM1637_Ack_6();
		
	TM1637_WriteByte_6(NumDis[ch]);//��ʾ
	TM1637_Ack_6();
	
	TM1637_Stop_6();
	TM1637_Start_6();
	TM1637_WriteByte_6(0x8C);
	TM1637_Ack_6();
	TM1637_Stop_6();
		
}
void TM1637_Display_6(void)
{
	unsigned char a=0,b=0,c=0,d=0;
	if(cache[5]<0)
	{
		d=((-cache[5])%60)%10;
		c=((-cache[5])%60)/10;
		b=((-cache[5])/60)%10;
		a=0x40;
	}
	else
	{
	d=(cache[5]%60)%10;
	c=(cache[5]%60)/10;
	b=(cache[5]/60)%10;
	a=(cache[5]/60)/10;
	}
	TM1637_Start_6();
	TM1637_WriteByte_6(0x40);//0x40�������ʾ��ַ�Լ�1ģʽ
	TM1637_Ack_6();
	TM1637_Stop_6();
	TM1637_Start_6();
	TM1637_WriteByte_6(0xC0);//0X00��ַ��ʼ��ʾ
	TM1637_Ack_6();
		
	TM1637_WriteByte_6(TIME[a]);//��ʾ1
	TM1637_Ack_6();
	 if(x==1)
	{
		temp=TIME[b];
		x++;
	} 
  temp^=0x80;  //10000000^11001111=01001111;01001111^10000000=11001111
	TM1637_WriteByte_6(temp);//��ʾ2��������ð��
	TM1637_Ack_6();
	
	TM1637_WriteByte_6(TIME[c]);//��ʾ1
	TM1637_Ack_6();
		
	TM1637_WriteByte_6(TIME[d]);//��ʾ4
	TM1637_Ack_6();
		
	TM1637_WriteByte_6(0xFF);
	TM1637_Ack_6();
		
	TM1637_WriteByte_6(0xFF);
	TM1637_Ack_6();
	
	TM1637_Stop_6();
	TM1637_Start_6();
	TM1637_WriteByte_6(0x8C);
	TM1637_Ack_6();
	TM1637_Stop_6();
}

void time_6(void)
{
	if(x>=2)
			{
				if((Enable_6==1)&&(channel==0))
					{
						G_6=1;
						cache[5]--;
						if(cache[5]<0)
						{
							if((-cache[5])>599)
							{
								Enable_6=0;
							}
						K1=1;
						}	
					}
				x=1;
			}
}
void Digital_off_6(void)
{
	TM1637_Start_6();
	TM1637_WriteByte_6(0x88);
	TM1637_Ack_6();
	TM1637_Stop_6();
}


