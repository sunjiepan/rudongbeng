//#include "Dac.h"
//#include "Max6675.h"
//#include "Modbus.h"
#include "Pid.h"
//#include "Flash.h"

//#include "Process.h"

#define MIN_ERROR_VALUE 2
#define MAX_ERROR_VALUE 60

//volatile uint8_t  get_now_temp_flag   = 0;
//volatile uint8_t  pwm_con_time_flag = 0;
//volatile uint8_t  pid_tune_flag = 1;//��ʼΪ0 ��pid�׶� ����Ĭ�ϵ�ֵ    1 Ϊ����������
//volatile uint8_t  enable_pid_sec_flag = 0;
//volatile uint8_t  pid_self_sec_flag   = 0;
volatile uint8_t  pid_self_first_status_flag = 0;
volatile uint8_t  once_add_1_flag     = 0;
volatile uint8_t  enable_calc_min_max_flag = 0;
volatile uint8_t  Pid_Setok = 0;

uint16_t time0_temp2=0;
int32_t zero_across_counter = 0;
int64_t pid_self_time_sec = 0;

int32_t cool_ack_counter    = 0;
int32_t hot_ack_counter     = 0;
int32_t k_pid_self_counter = 0;




float  Proportion  = 0.0;           //  �������� Proportional Const
float  Integral    = 0.0;           //  ���ֳ��� Integral Const        
float  Derivative  = 0.0;           //  ΢�ֳ��� Derivative Const

float  LastError   = 0.0;           //  Error[-1]
float  PrevError   = 0.0;           //  Error[-2]
float  SumError    = 0.0;           //  Sums of Errors
float  dError      = 0.0;
float  Error       = 0.0;
//pid
float           SV_value           = 50.0; //�趨�¶�ֵ
float           PV_value           = 0.0;  //���ڲ������ĵ�ǰ�¶�ֵ
float           KV_value           = 0.0;  //�¶Ȳ�ֵ
//float           PV_oldvalue        = 0.0;  //���ڲ������ĵ�ǰ�¶�ֵ
//volatile float  P_value            = 0.0;  //������ ����56.3����56.3%  0.0--200.0
//int             I_value            = 0;  //����ʱ��  ��  0-3600
//int             D_value            = 0;   //΢��ʱ��  ��  0-900
int32_t  pid_result = 0;                //DAC���

float max_temp  = 0.0 ;  //��ʼ�¶ȵ���0
float min_temp  = 100.0 ;//��ʼ�¶ȵ���100
float sum_temp  = 0.0 ;  //��ʼ�¶ȵ���0
float aver_temp	= 0.0 ;

float  KC = 1.0;  //�ٽ����ϵ��  ��ʼĬ�ϵ�ֵ
int32_t    TC = 40;   //������      ��ʼĬ�ϵ�ֵ

float T_Hight = 0.0;
float T_LOW   = 100.0; //�¶�
int64_t TIME_Hight = 0;
int64_t TIME_LOW   = 0;	//�������
float pid_self_calc_buffer[4]={0.0,0.0,0.0,0.0};


//0 ��� �ٽ����� KC �� ������ TC
  // KC = (4*d)/(3.14*A)  ---> d = 5(�����ֵ) ; A = ��¼���¶����ֵ�����ֵ�Ĳ�ֵ��0.5�� ����(T_Hight - T_LOW)*0.5
  // KC = (4*5)/(3.14*((T_Hight - T_LOW)*0.5)) = 40/3.14/(T_Hight - T_LOW) =  12.7/(T_Hight - T_LOW)
  // KC = 12.7/(T_Hight - T_LOW)
  // TC = 2 * (TIME_Hight - TIME_LOW) ---> 2 * ( �ߵ��¶ȶ�Ӧʱ�� - �͵��¶ȶ�Ӧʱ�� )
  // TC = 2 * (TIME_Hight - TIME_LOW)
  //1 ��� ����ı���ϵ�� �������� ΢������
  //Proportion = 0.6*KC
  //I_value    = 0.5*TC
  //D_value    = 0.125*TC
  //2 �������� ������ ����ϵ�� ΢��ϵ�� 
  //P_value     = (1/Proportion)*100
  //Integral	  = Proportion/I_value = (0.6*KC)/(0.5*TC)
  //Derivative  = Proportion*D_value = (0.6*KC)*(0.125*TC)  
  //3��ʾ�õ�3��������ֵ
  //P_value     = (1/Proportion)*100  �ٷֱ�
  //I_value     = 0.5*TC				�� 
  //D_value     = 0.125*TC			�� 
  //4pid�����õ�3��������ֵ
  //Proportion  = 0.6*KC
  //Integral	  = Proportion/I_value = (0.6*KC)/(0.5*TC)
  //Derivative  = Proportion*D_value = (0.6*KC)*(0.125*TC)  
  
  //KC = 21.4;//test 
  //TC = 471;//test 
void pid_pro(void)//pid ������� ppppppppppppppppppppppppppppp
{
  int temp_pid;
  Error = SV_value - PV_value;                 // ƫ��
  if(( Error < max_value_error  ) && ( Error > (min_value_error)  ))//ֻ����һ�����²Χ�ڲ�pid����
  {    
    SumError += Error;
    dError    = LastError - PrevError;   // ��ǰ΢��
    PrevError = LastError;
    LastError = Error;
    temp_pid  =  (int)((Proportion * Error) + (Integral * SumError) + (Derivative * dError));    
    //temp_pid  =  (int)(temp_pid * 0.5) ;//�����������
  }
  else//ֻ�п�������
  {
    if( Error >= max_value_error )//Զ���ڵ�ǰ�¶ȣ�����
    {
      temp_pid = 100;
      //temp_pid = 80;
    }
    else if( Error <= (min_value_error) )//ԶС�ڵ�ǰ�¶ȣ�������
    {
      temp_pid = 0;
    }
  }
  if( temp_pid < 0 )
  {
    temp_pid = 0;
  }
  else if( temp_pid > 100 )
  {
    temp_pid = 100;
  } 
  Dac_Out(40 + (uint8_t)(temp_pid * 1.97) );//������һ���Ľ��������� 
}


//PID������
void Pid_Sinceset(void)
{
    pid_self_time_sec++;
    if(pid_self_time_sec > (3600*3)) // ����ܵ�������ʱ�������3/5=0.6��Сʱ����˵������ʧ��
    {
      pid_self_time_sec = 0;			
//      pid_tune_flag = 0;//��ô���Զ��˳����������� ͬʱ����Ĭ��ֵ  ����pid�׶�
      KC = 1.0;//�ٽ����ϵ��  ��ʼĬ�ϵ�ֵ
      TC = 40; //������    ��ʼĬ�ϵ�ֵ 
      memory[PIDSET_ADR]=0;     //�ر�PID������
      Pid_Setok=0;   //PID������ʧ��
    }
    if(( pid_self_first_status_flag == 1) || ( pid_self_first_status_flag == 0))//0 �趨�¶� ���� ��ǰ�¶�  //1�趨�¶� ���� ���� ����  ��ǰ�¶�  ��������
    {
      //����on/off����
      if( SV_value >= PV_value )//��������
      {
        cool_ack_counter = 0;
        hot_ack_counter++;
        if(hot_ack_counter > 3)//����3�ζ���һ���Ľ�� ˵��ȷ�� SV_value >= PV_value
        {
          Dac_Out(237);//ȫ�ټ���
          if(once_add_1_flag == 0)
          {
            once_add_1_flag = 1;
            zero_across_counter++; 
            if(zero_across_counter == 3 )
            {
              TIME_LOW = pid_self_time_sec - 3;//��ʱ��ʱ�䲻������¶ȶ�Ӧ��ʱ��
            }
          }
        }
      }
      else//��ǰ�¶� ���� �趨�¶� ֹͣ����
      { 
        hot_ack_counter = 0;
        cool_ack_counter++;
        if(cool_ack_counter > 3)
        {
          Dac_Out(40);//������
          if(once_add_1_flag == 1)
          {
            once_add_1_flag = 0;
            zero_across_counter++;
            if(zero_across_counter == 3 )
            {
              TIME_LOW = pid_self_time_sec - 3;//��ʱ��ʱ�䲻������¶ȶ�Ӧ��ʱ��
            }
          }
        }
      }
      
      //����¶� ������ zero_across_counter = 3 �Ľ׶�
      //����¶� ������ zero_across_counter = 4 �Ľ׶�
      if((zero_across_counter == 3 ) || (zero_across_counter == 4 ))
      {				
        pid_self_calc_buffer[k_pid_self_counter] = PV_value;
        k_pid_self_counter++;
        if(k_pid_self_counter > 3)//0--3 ��4��Ԫ��
        {
          k_pid_self_counter = 0;
          enable_calc_min_max_flag = 1;
        }
        if(enable_calc_min_max_flag == 1)//ֻҪ��4��ֵ���Ϳ��Լ����� ��������ֵ������ǰ���ֵ 
        {
          //ȥ����Сֵ ���ֵ ȡʣ��2��ֵ��ƽ��ֵ 
          sum_temp = 0.0;  //����0
          min_temp = 1024.0;
          max_temp = 0.0;
          
          for(uint8_t k_max_min = 0; k_max_min < 4; k_max_min++ )
          {						
            if(pid_self_calc_buffer[k_max_min] <= min_temp)
            {
              min_temp = pid_self_calc_buffer[k_max_min];
            }
            if(pid_self_calc_buffer[k_max_min] >= max_temp)
            {
              max_temp = pid_self_calc_buffer[k_max_min];
            }						
            sum_temp = (sum_temp + pid_self_calc_buffer[k_max_min]);
          }
          sum_temp =  sum_temp - min_temp - max_temp ;
          
          
          //pid_self_first_status_flag = 1 ʱ ����¶ȳ�����3�׶�
          //pid_self_first_status_flag = 0 ʱ ����¶ȳ�����4�׶�
          if(pid_self_first_status_flag == 1)
          {
            if(zero_across_counter == 3 )//����¶�
            {
              aver_temp = (sum_temp/2.0);					
              if( aver_temp <= T_LOW )
              {
                T_LOW = aver_temp;
              }				
            }
            else if(zero_across_counter == 4 )//����¶�
            {
              aver_temp = (sum_temp/2.0);
              if( aver_temp >= T_Hight )
              {
                T_Hight = aver_temp;
              }
            }
          }
          else if(pid_self_first_status_flag == 0)
          {
            if(zero_across_counter == 4 )//����¶�
            {
              aver_temp = (sum_temp/2.0);					
              if( aver_temp <= T_LOW )
              {
                T_LOW = aver_temp;
              }				
            }
            else if(zero_across_counter == 3 )//����¶�
            {
              aver_temp = (sum_temp/2.0);
              if( aver_temp >= T_Hight )
              {
                T_Hight = aver_temp;
              }
            }
          }
        }
      }
      else if(zero_across_counter == 5 )//4�ι�0 ��˵���������� �����ɹ�
      {
        zero_across_counter = 0;				
        //        pid_tune_flag = 0;//����pid�׶�
        //pid_tune_flag = 1;//test
        TIME_Hight = pid_self_time_sec - 3;//��ʱ��ʱ�䲻������¶ȶ�Ӧ��ʱ��
        //���� T_Hight T_LOW TIME_Hight TIME_LOW ��4��ֵ 
        //��������4��ֵ  KC �� TC ��ֵ���������    
        KC = 12.7/(T_Hight - T_LOW);
        KC = 5.0 * KC;//��Ϊ��0.2sһ�� ��������5��
        TC = 1 * (TIME_Hight - TIME_LOW);//�����¼�� ����¶� �� ����¶ȶ�Ӧ��ʱ�� ��ô���������ʽ��TC = 2 * (TIME_Hight - TIME_LOW);
       
        memory[PIDSET_ADR]=0;     //�ر�PID������
        Pid_Setok=1;   //PID�������ɹ�
      }
    }
}

void PidParameter_Sinceset(void)//PID����������
{
  //��¼�˿̵�״̬ ���趨�¶��Ƿ� ���ڻ���� ��ǰ�¶� 
  PV_value = read_max6675_temper();
  if( SV_value >= PV_value )//�趨�¶� ���� ���� ����  ��ǰ�¶�  ��������
  {
    pid_self_first_status_flag = 1;
    once_add_1_flag = 0;
  }
  else//�趨�¶� ���� ��ǰ�¶�
  {
    pid_self_first_status_flag = 0;
    once_add_1_flag = 1;
  } 
  zero_across_counter = 0;
  pid_self_time_sec = 0;      
  k_pid_self_counter = 0;
  enable_calc_min_max_flag = 0;
  max_temp = 0.0 ;  //��ʼ�¶ȵ���0
  min_temp = 1024.0 ;//��ʼ�¶ȵ���1024
  sum_temp = 0.0 ;  //��ʼ�¶ȵ���0
  aver_temp = 0.0 ;
  T_Hight  = 0.0;
  T_LOW    = 1024.0; //�¶�
  TIME_Hight = 0;
  TIME_LOW   = 0;	//�����0.2s 
}

void PidParameter_pro(void)//PID����
{
  
 KC=*((float *)(&memory[PIDKC_ADR]));
 TC=*((int *)(&memory[PIDTC_ADR]));
  if(KC > 1666.0 )
  {
    KC = 1666.0;//��Ӧ ������Ϊ 0.1%
  }
  else if(KC < 0.5 )
  {
    KC = 0.5;//��Ӧ ������Ϊ 200.0%
  }
  if(TC > 7200 )
  {
    TC = 7200;
  }
  else if(TC < 8 )
  {
    TC = 8;
  }   
  Proportion  = 0.6*KC;
  Integral	= (0.6*KC)/(0.5*TC);
  Derivative  = (0.6*KC)*(0.125*TC); 
}


void Preheat(void)// Ԥ��
{
  uint16_t preheat_cnt=0;
  uint8_t  preheat_out=40;
  while(preheat_cnt<240) //2��
  {
    preheat_cnt++;
    if(preheat_out<237)
      preheat_out++;
    else
      preheat_out=237;
     Dac_Out(preheat_out);
    if(PV_value>=(SV_value-50))
      break;
    HAL_Delay(500);
  }
}

uint16_t PID_Algorithm(float setValue, float feedbackValue)
{
	int16_t temp_pid;
	float errorValue;
	uint16_t duty_cycle = 0;	
	errorValue = setValue - feedbackValue;                 // ƫ��
	
  if(errorValue > MIN_ERROR_VALUE && errorValue < MAX_ERROR_VALUE )//ֻ����һ�����²Χ�ڲ�pid����
  {    
    SumError += Error;
    dError    = LastError - PrevError;   // ��ǰ΢��
    PrevError = LastError;
    LastError = Error;
    temp_pid  =  (int)((Proportion * Error) + (Integral * SumError) + (Derivative * dError));    
    //temp_pid  =  (int)(temp_pid * 0.5) ;//�����������
  }
  else if(errorValue >= MAX_ERROR_VALUE)//ֻ�п�������
  {
		
  }
	else
	{
		
	}
  if( temp_pid < 0 )
  {
    temp_pid = 0;
  }
  else if( temp_pid > 100 )
  {
    temp_pid = 100;
  } 
  Dac_Out(40 + (uint8_t)(temp_pid * 1.97) );//������һ���Ľ��������
	
	
	return duty_cycle;
}
