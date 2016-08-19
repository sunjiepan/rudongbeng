#ifndef _PID_H
#define _PID_H

#include "stm32f1xx_hal.h"

#define     max_value_error               50.0
#define     min_value_error              -50.0//��������

//extern volatile uint8_t  get_now_temp_flag;
//extern volatile uint8_t  pwm_con_time_flag;
//extern volatile uint8_t  pid_tune_flag;//��ʼΪ0 ��pid�׶� ����Ĭ�ϵ�ֵ    1 Ϊ����������
//extern volatile uint8_t  enable_pid_sec_flag;
//extern volatile uint8_t  pid_self_sec_flag;
extern volatile uint8_t  Pid_Setok;
extern volatile uint8_t  pid_self_first_status_flag;
extern volatile uint8_t  once_add_1_flag;
extern volatile uint8_t  enable_calc_min_max_flag;
extern unsigned short time0_temp2;
extern float  KV_value;  //�¶Ȳ�ֵ
extern int zero_across_counter;
extern long pid_self_time_sec;

extern int cool_ack_counter;
extern int hot_ack_counter;
extern int k_pid_self_counter;


extern float  Proportion;           //  �������� Proportional Const
extern float  Integral;           //  ���ֳ��� Integral Const        
extern float  Derivative;           //  ΢�ֳ��� Derivative Const
extern float  LastError;           //  Error[-1]
extern float  PrevError;           //  Error[-2]
extern float  SumError;           //  Sums of Errors
extern float  dError;
extern float  Error;

//pid
extern float           SV_value; //�趨�¶�ֵ
extern float           PV_value;  //���ڲ������ĵ�ǰ�¶�ֵ
//extern float           PV_oldvalue;  //���ڲ������ĵ�ǰ�¶�ֵ
//extern volatile float  P_value;  //������ ����56.3����56.3%  0.0--200.0
//extern int             I_value;  //����ʱ��  ��  0-3600
//extern int             D_value;   //΢��ʱ��  ��  0-900

extern int  pid_result;                //DAC���

extern float max_temp;  //��ʼ�¶ȵ���0
extern float min_temp;//��ʼ�¶ȵ���100
extern float sum_temp;  //��ʼ�¶ȵ���0
extern float aver_temp ;

extern float  KC;  //�ٽ����ϵ��  ��ʼĬ�ϵ�ֵ
extern int    TC;   //������      ��ʼĬ�ϵ�ֵ

extern float T_Hight;
extern float T_LOW; //�¶�
extern long TIME_Hight;
extern long TIME_LOW;	//�������
extern float pid_self_calc_buffer[4];

void pid_pro(void);
void Pid_Sinceset(void);
void PidParameter_Sinceset(void);//PID����������
void PidParameter_pro(void);//PID����
void Preheat(void);// Ԥ��
#endif /* _PID_H */