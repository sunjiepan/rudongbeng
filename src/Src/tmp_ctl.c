#include <stdio.h>
#include "tmp_ctl.h"
#include "wenyu_tmp_ctl_bsp.h"
#include "stm32f1xx_hal.h"
#include "Queue.h"
#include "eeprom.h"

static uint16_t buzzer_interval = 50;
static float set_temp_value = 40;
static float current_temp = 0;
static uint8_t switchOnOff = 1;

static QUEUE8_t m_QueueComRx         = {0};
static QUEUE8_t m_QueueComTx         = {0};
static uint8_t  m_ComRxBuf[COM_RX_BUF_SIZE]      = {0};     
static uint8_t  m_ComTxBuf[COM_TX_BUF_SIZE]      = {0}; 

	/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777};
uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0};
uint16_t VarValue = 0;

static PID_TypeDef pid={
	0,0,0,0,0,500,0.2,0.1,0,400
};



void Init(void)
{
	QUEUE_PacketCreate(&m_QueueComRx, m_ComRxBuf, sizeof(m_ComRxBuf));
  QUEUE_PacketCreate(&m_QueueComTx, m_ComTxBuf, sizeof(m_ComTxBuf));
	
	/* Unlock the Flash Program Erase controller */
  HAL_FLASH_Unlock();
	
	/* EEPROM Init */
  EE_Init();

  /* read the last stored variables data*/
  EE_ReadVariable(VirtAddVarTab[0], &VarDataTab[0]);
	
	HAL_FLASH_Lock();
	
	set_temp_value = (float)VarDataTab[0] / 100;
	

}

void SetBuzzer(uint16_t interval)
{
	buzzer_interval = interval;
}
void BuzzerON(void)
{
	BSP_BUZZER_On(Buzzer);
	//HAL_GPIO_WritePin(BUZZER_GPIO_PORT,BUZZER_PIN, GPIO_PIN_SET);
}
void BuzzerOFF(void)
{
	BSP_BUZZER_Off(Buzzer);
	//HAL_GPIO_WritePin(BUZZER_GPIO_PORT,BUZZER_PIN, GPIO_PIN_RESET);	
}

	
void BuzzerOutPut(void)
{
	if(buzzer_interval)
	{
		//BuzzerON();
		/* Insert delay n ms */
		HAL_Delay(buzzer_interval);		
	}
	//BuzzerOFF();
}

void SetTemp(uint16_t temp)
{
	/* Unlock the Flash Program Erase controller */
  HAL_FLASH_Unlock();
  /* Store 0x1000 values of Variable1 in EEPROM */
	EE_WriteVariable(VirtAddVarTab[0], temp);
	HAL_FLASH_Lock();
	set_temp_value = (float)temp / 100;
}

float GetTemp(void)
{
	return set_temp_value;
}

float GetCurrentTemp(void)
{
	//current_temp = BSP_DS18B20_Get_Temp();
	return current_temp;
}

void InsertSort(float a[], int n)  
{  
  for(int i= 1; i<n; i++)
	{  
		if(a[i] < a[i-1]){              
    int j= i-1;   
    float x = a[i];        
    a[i] = a[i-1];       
		while(x < a[j]){   
				a[j+1] = a[j];  
				j--;         
		}  
		a[j+1] = x;    
        }  
    }  
      
}

float InserAver(float a[], int n)
{
	float sum = 0;
	int i = 0;
	for(i= 0; i<n; i++)
	{
		sum += a[i];
	}
	sum /= 5;
	return sum;
}

float ReadCurrentTemp(void)
{
	float temp[7];
	
	for(int i = 0; i < 7; i++)
	{
		temp[i] = BSP_DS18B20_Get_Temp();
	}
	
	InsertSort(temp,7); 
	current_temp = InserAver(&temp[1],5);
	
	return current_temp;
}

void SwitchOnOff(uint8_t onOff)
{
	TIM_HandleTypeDef htim3;
	htim3.Instance = TIM3;
	switchOnOff = onOff;
	
	if(switchOnOff == 0)
		HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
	else
		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
}

void PWMOutPut(uint16_t pulse_num,uint16_t period)
{
	TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
	TIM_HandleTypeDef htim3;
	TIM_HandleTypeDef htim2;
	uint32_t time2_period = 0;
	
	time2_period = pulse_num * period;
	
	htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = time2_period / 100 + 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  HAL_TIM_OC_Init(&htim2);

  HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 720 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = period / 10 ;
	//htim3.Init.Period = period - 1; //F = SYSCLK/(( Prescaler + 1)*( Period + 1)); 72000K/(( 71  +  1) * ( 999 + 1))
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = period / 20;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
	
	if(switchOnOff == 0)
		HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
	else
		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	
	HAL_TIM_OnePulse_Start(&htim2, TIM_CHANNEL_1);
	
}

uint8_t GetChannel(void)
{
	return 0;	
}
void SetTimer(uint16_t timer)
{
	
}
uint8_t GetTimer(void)
{
	return 0;
}

/*******************************************************************************
* Function Name : uint32_t RxRead(uint8_t *buffter, uint32_t buffterSize)
* Description   : �ӽ��ջ����ж�����
* Input         : 
* Output        : 
* Other         : 
* Date          : 2014.11.28
*******************************************************************************/
uint32_t RxRead(uint8_t *buffter, uint32_t buffterSize)
{
	uint32_t tempLen;

	tempLen = QUEUE_PacketStartEndCharSplit(&m_QueueComRx, START_CHAR, buffter, buffterSize);
		
	return tempLen;
}
/*******************************************************************************
* Function Name : uint32_t RxWrite(uint8_t *buffter, uint32_t writeLen)
* Description   : д���ݵ����ջ�����
* Input         : 
* Output        : 
* Other         : 
* Date          : 2014.11.28
*******************************************************************************/
uint32_t RxWrite(uint8_t *buffter, uint32_t writeLen)
{
    return QUEUE_PacketIn(&m_QueueComRx, buffter, writeLen);
}
/*******************************************************************************
* Function Name : uint32_t TxRead(uint8_t *buffter, uint32_t buffterSize)
* Description   : �ӷ��ͻ����ж�����
* Input         : 
* Output        : 
* Other         : 
* Date          : 2014.11.28
*******************************************************************************/
uint32_t TxRead(uint8_t *buffter, uint32_t buffterSize)
{
    return QUEUE_PacketOut(&m_QueueComTx, buffter, buffterSize);;
}
/*******************************************************************************
* Function Name : uint32_t TxWrite(uint8_t *buffter, uint32_t writeLen)
* Description   : д���ݵ����ͻ�����
* Input         : 
* Output        : 
* Other         : 
* Date          : 2014.11.28
*******************************************************************************/
uint32_t TxWrite(uint8_t *buffter, uint32_t writeLen)
{
    return QUEUE_PacketIn(&m_QueueComTx, buffter, writeLen);
}



HAL_StatusTypeDef UartTransmit(uint8_t *aTxBuffer)
{
	return HAL_UART_Transmit_DMA(&huart1, (uint8_t*)aTxBuffer, sizeof(CustomProtocol_TypeDef));	
}
HAL_StatusTypeDef UartReceive(uint8_t *aRxBuffer)
{
	return HAL_UART_Receive_DMA(&huart1, (uint8_t*)aRxBuffer, sizeof(CustomProtocol_TypeDef));
}

uint16_t PID_Algorithm(float setValue, float feedbackValue)
{	
	pid.setValue = setValue; 
	pid.feedbackValue = feedbackValue;
	
	if(setValue < 10)
	{
		setValue = 10;
	}
	else if(setValue > 100)
	{
		setValue = 100;
	}
	
	if(feedbackValue < 0)
	{
		feedbackValue = 0;
	}
	else if(feedbackValue > 100)
	{
		feedbackValue = 100;
	}
	pid.err = pid.setValue - pid.feedbackValue;
	if(pid.err > MAX_ERROR_VALUE)
	{
		pid.out = MAX_DUTY_CYCLE;
	}
	else if(pid.err > MIN_ERROR_VALUE && pid.err < MAX_ERROR_VALUE)
	{
		pid.out = pid.outNext + pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last);         
		pid.err_last = pid.err_next;     
		pid.err_next = pid.err;
		pid.outNext = pid.out;
		if(pid.out < MIN_DUTY_CYCLE)
		{
			pid.out = MIN_DUTY_CYCLE;
		}
		else if(pid.out > MAX_DUTY_CYCLE)
		{
			pid.out = MAX_DUTY_CYCLE;
		}
	}
	else if(pid.err < 0)
	{
		pid.out = MIN_DUTY_CYCLE;
	}
	
	return pid.out;
}




uint8_t getCrc8(uint8_t *data)
{
   uint8_t crc = 0;
   uint8_t length = 3;
   uint8_t i, j;
   for(i=0; i<length; i++) {
      crc ^= data[i];
      for(j=0; j<8; j++) {
         if(crc & 1) 
					 crc = (uint8_t)((crc >> 1) ^ ((1 << 7) + (1 << 6) + (1 << 5)));
         else crc >>= 1;
      }
   }
   return crc;
}



TmpCtl_TypeDef weiyu_tmp_ctr = {
	Init,
	SetBuzzer,
	BuzzerON,
	BuzzerOFF,
	BuzzerOutPut,
	SetTemp,
	GetTemp,
	GetCurrentTemp,
	ReadCurrentTemp,
	SwitchOnOff,
	GetChannel,
  SetTimer,
  GetTimer,
	PWMOutPut,
	UartTransmit,
	UartReceive,
	PID_Algorithm,
	getCrc8,
	{0,0,0,0,0},
	{0,0,0,0,0},
};

