/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "wenyu_tmp_ctl_bsp.h"
#include "tmp_ctl.h"
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
float temp_value;
uint16_t duty_cycle;
	
uint8_t aTxBuffer[TXBUFFERSIZE];
uint8_t aRxBuffer[RXBUFFERSIZE];

uint8_t Uart1ReadyRead = RESET;
uint8_t Uart1ReadyWrite = SET;

uint8_t Rxbuff[RXBUFFERSIZE * 5];
extern uint8_t TIM3_Pulse_TIM2_Counter_OK; 
uint16_t pulse = 0;
uint16_t rev_pulse = 0;
uint16_t frequence = 0;
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void Foreward_Rollback(GPIO_PinState PinState);
static void MX_USART1_UART_Init(void);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void RxTxProcessing(void);
void Output_Pulse(uint16_t Num);
void Frequence_Setting(uint32_t PulseFrequency);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
	uint32_t rxtxtickstart = 0;
  rxtxtickstart = HAL_GetTick();
	
//	uint32_t readtickstart = 0;
//  readtickstart = HAL_GetTick();
	
//	uint32_t pidtickstart = 0;
//  pidtickstart = HAL_GetTick();

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_IWDG_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	
	weiyu_tmp_ctr.Init();
	
	HAL_UART_Receive_DMA(&huart1, (uint8_t*)aRxBuffer, RXBUFFERSIZE);
	HAL_IWDG_Start(&hiwdg);
	


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		if((HAL_GetTick() - rxtxtickstart) > DELAY_200MS)
		{
			rxtxtickstart = HAL_GetTick();
			RxTxProcessing();
		}
		if(TIM3_Pulse_TIM2_Counter_OK == CMD_FINISH)
		{
			TIM3_Pulse_TIM2_Counter_OK = FORWARD_START;
			Frequence_Setting(frequence);
			Foreward_Rollback(GPIO_PIN_SET);
			Output_Pulse(pulse);
		}else if(TIM3_Pulse_TIM2_Counter_OK == FORWARD_FINISH)
		{
			TIM3_Pulse_TIM2_Counter_OK = ROLLBACK_START;
			Foreward_Rollback(GPIO_PIN_RESET);
			Output_Pulse(rev_pulse);
		}
		
		HAL_IWDG_Refresh(&hiwdg);


  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/8000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* IWDG init function */
void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  HAL_IWDG_Init(&hiwdg);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  HAL_TIM_OC_Init(&htim2);

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0xFFFF;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 71;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim3);

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 36;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOD_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void Frequence_Setting(uint32_t PulseFrequency) 
{ 
	uint32_t nPDTemp ;
  TIM_OC_InitTypeDef sConfigOC;
	
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
	nPDTemp = 72000UL/PulseFrequency; 
	
	htim3.Instance = TIM3;
  htim3.Init.Prescaler = 999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = nPDTemp;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);
	
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = nPDTemp/2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
}

void Output_Pulse(uint16_t Num)
{
		TIM2->CCR1 = Num; 
		TIM2->CNT = 0; 
		HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
}

void Foreward_Rollback(GPIO_PinState PinState)
{
	PinState?HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

void RxTxProcessing(void)
{
	uint8_t result_crc;
	
	if(Uart1ReadyRead == SET)
	{
		Uart1ReadyWrite = RESET;
		RxRead((uint8_t*)&weiyu_tmp_ctr.aRxBuffer, sizeof(CustomProtocol_TypeDef));
		Uart1ReadyWrite = SET;
	}
		
	if(weiyu_tmp_ctr.aRxBuffer.start_flag == 0xff && weiyu_tmp_ctr.aRxBuffer.end_flag == 0xff)
	{
		result_crc = weiyu_tmp_ctr.getCrc8((uint8_t *)&weiyu_tmp_ctr.aRxBuffer.cmd);
		if(weiyu_tmp_ctr.aRxBuffer.crc == result_crc)
		{
			switch(weiyu_tmp_ctr.aRxBuffer.cmd)
			{
				case CMD_SET_PULSE:
					if(TIM3_Pulse_TIM2_Counter_OK == ROLLBACK_FINISH || TIM3_Pulse_TIM2_Counter_OK == STOP_STATUE)
					{
							pulse = weiyu_tmp_ctr.aRxBuffer.data;
							//TIM3_Pulse_TIM2_Counter_OK = CMD_PULSE_FINISH;
					}
					memcpy((uint8_t*)&weiyu_tmp_ctr.aTxBuffer,(uint8_t*)&weiyu_tmp_ctr.aRxBuffer,sizeof(CustomProtocol_TypeDef));
					HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&weiyu_tmp_ctr.aTxBuffer, sizeof(CustomProtocol_TypeDef));
					break;
				case CMD_SET_PERIOD:
					if(TIM3_Pulse_TIM2_Counter_OK == ROLLBACK_FINISH || TIM3_Pulse_TIM2_Counter_OK == STOP_STATUE)
					{
							frequence = weiyu_tmp_ctr.aRxBuffer.data;
							//TIM3_Pulse_TIM2_Counter_OK = CMD_PERIOD_FINISH;
					}
					memcpy((uint8_t*)&weiyu_tmp_ctr.aTxBuffer,(uint8_t*)&weiyu_tmp_ctr.aRxBuffer,sizeof(CustomProtocol_TypeDef));
					HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&weiyu_tmp_ctr.aTxBuffer, sizeof(CustomProtocol_TypeDef));
					break;
				case CMD_SET_REV_PULSE:
					if(TIM3_Pulse_TIM2_Counter_OK == ROLLBACK_FINISH || TIM3_Pulse_TIM2_Counter_OK == STOP_STATUE)
					{
							
							rev_pulse = weiyu_tmp_ctr.aRxBuffer.data;
							//TIM3_Pulse_TIM2_Counter_OK = CMD_REV_PULSE_FINISH;
							TIM3_Pulse_TIM2_Counter_OK = CMD_FINISH;
					}
					memcpy((uint8_t*)&weiyu_tmp_ctr.aTxBuffer,(uint8_t*)&weiyu_tmp_ctr.aRxBuffer,sizeof(CustomProtocol_TypeDef));
					HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&weiyu_tmp_ctr.aTxBuffer, sizeof(CustomProtocol_TypeDef));
					break;
				case CMD_SWITCH_HART:
					break;
				case CMD_GET_CHANNEL:
					break;
				case CMD_SET_TIMER:
					break;
				case CMD_GET_TIMER:
					break;
				default:
					break;
			}
		}
	}
	memset((uint8_t*)&weiyu_tmp_ctr.aRxBuffer,0, sizeof(CustomProtocol_TypeDef));
}	

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
