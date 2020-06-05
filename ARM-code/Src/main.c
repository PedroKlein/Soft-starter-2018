
/**
  ******************************************************************************
  * @file           : main.c
  * @author         : Pedro Klein
  * @brief          : Programa para o controle do softstarter e da comunicacao com a interface
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <math.h>
//!Resolucao do ADC do Vrms
#define ADC_RES_Vrms 0.16748
//!Definicao do Vpico de Vrms
#define Vpike 342
//!Numero de aquisicoes durante a subida e descida para o grafico
#define Aquisitions 26
//!Valor minimo do angulo (varia de 1-100)
#define Increment_minimum 1
#define Arr_period 102
//!Erro compensado na medida Vrms
#define Vrms_ERROR 1.3
//!Erro compensado na medida Irms
#define Irms_ERROR 1

/**
  * @brief enum dos comandos recebidos pela interface
  * @note START:Comando recebido para iniciar
  * @note STOP:Comando recebido para parar
  * @note EMERGENCY:Comando recebido para parada de emergencia
  * @note VRMSREAD:Comando exigindo medida Vrms como retorno
  * @note IRMSREAD:Comando exigindo medida Irms como retorno
  * @note CURRENTSET:Comando que define a corrente nominal do motor
  */
enum calls
{
  START = 0x72,
  STOP = 0x66,
  EMERGENCY = 0x61,
  VRMSREAD = 0x76,
  IRMSREAD = 0x69,
  CURRENTSET = 0x63
};

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
//!ADC1 usado para medidas de tensao
ADC_HandleTypeDef hadc1;
//!ADC2 usado para medidas de corrente
ADC_HandleTypeDef hadc2;
//!ADC1 usado com DMA
DMA_HandleTypeDef hdma_adc1;
//!ADC2 usado com DMA
DMA_HandleTypeDef hdma_adc2;

//!Timer 1 usado para atualizar o deslocamento do pulso de controle
TIM_HandleTypeDef htim1;
//!Timer 2 usado para triggar a conversao dos ADC's por DMA
TIM_HandleTypeDef htim2;
//!Timer 3 usado para gerar o pulso de controle (PWM)
TIM_HandleTypeDef htim3;
//!Timer 4 usado para contar RPM (IC)
TIM_HandleTypeDef htim4;

//!Comunicacao por cabo
UART_HandleTypeDef huart2;
//!Comunicacao por bluetooth
UART_HandleTypeDef huart6;

osThreadId Measure_TaskHandle;
osThreadId Serial_TaskHandle;
osThreadId RPM_TaskHandle;
osThreadId Alarm_TaskHandle;
osThreadId Initialization_Handle;
osThreadId SoftStart_TaskHandle;
osThreadId Chart_TaskHandle;

/* USER CODE BEGIN PV */

//!Semaforo para sincronizacao do AD com a thread de calculo rms.
xSemaphoreHandle ADC_Conv_Semaphore = 0;
//!Semaforo para sincronizacao do deslocamento de pulso com a thread de atualizacao do pulso
xSemaphoreHandle Increment_Semaphore = 0;
//!Semaforo para envio de notificacao de emergencia a interface.
xSemaphoreHandle Emergency_Semaphore = 0;
//!Semaforo para envio de notificacao de corrente > 150% a interface
xSemaphoreHandle E150_Semaphore = 0;

/* Private variables ---------------------------------------------------------*/

//!Vetor que recebe conversao do ADC1
uint16_t ADC_Values_Vrms[16];
//!Vetor que recebe conversao do ADC2
uint16_t ADC_Values_Irms[16];
//!Variavel que recebe corrente nominal do motor da interface
uint16_t Nominal_Current = 500;
//!Variavel que recebe o Ipico da medida Irms
uint16_t Ipike = 1000;
//!Vetor das medidas Irms do grafico de subida
uint16_t Irms_data_rise[Aquisitions];
//!Vetor das medidas Irms do grafico de descida
uint16_t Irms_data_fall[Aquisitions];
//!Variavel que armazena valor de 200% de Irms
uint16_t i200 = 10000;
//!Variavel que armazena valor de 150% de Irms
uint16_t i150 = 10000;
//!Variavel que recebe valor final de Vrms
uint16_t Vrms;
//!Variavel que recebe valor final de Irms
uint16_t Irms;

//!Variavel que recebe tempo de subida
uint8_t rising_time = 5;
//!Variavel que recebe tempo de descida
uint8_t falling_time = 5;
//!Variavel que desloca os pulsos de controle
uint8_t increment = 102;
//!Flag que indica se o motor esta acelerando
uint8_t rise = 0;
//!Flag que indica se o motor esta desacelerando
uint8_t fall = 0;
//!Definicao da largura do pulso de controle
uint8_t Pulse_Width = 2;
//!Flag que indica o fim da aceleracao
uint8_t rise_ok = 0;
//!Flag que indica o fim da desaceleracao
uint8_t fall_ok = 0;
//!Flag que indica atualizacao da posicao do pulso de controle
uint8_t check = 1;
//!Flag que indica se a corrente nominal foi definida na interface
uint8_t Nominal_Current_SET = 0;

//!Variavel que recebe a resolucao do ADC de Irms
float ADC_RES_Irms = 0.48828;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
void Measure_Thread(void const *argument);
void Serial_Thread(void const *argument);
void RPM_Thread(void const *argument);
void Alarm_Thread(void const *argument);
void Initialization_Thread(void const *argument);
void SoftStart_Thread(void const *argument);
void Chart_Thread(void const *argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief Funcao que executa todas medidas necessarias para emergencia
  * @note Zera as flags que indicam aceleracao ou desaceleracao
  * @note Desliga os dois reles, o de alimentacao e o de bypass
  * @note Para o timer responsavel por gerar o pulso de controle
  * @note Seta a variavel 'increment' para seu valor inicial
  */
void Emergency(void)
{
  fall = 0;
  rise = 0;
  HAL_GPIO_WritePin(GPIOA, Bypass_relay_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, Power_relay_Pin, GPIO_PIN_RESET);
  HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
  increment = (Arr_period - Pulse_Width);
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();

  /* Create the thread(s) */
  /* definition and creation of Measure_Task */
  osThreadDef(Measure_Task, Measure_Thread, osPriorityNormal, 0, 128);
  Measure_TaskHandle = osThreadCreate(osThread(Measure_Task), NULL);

  /* definition and creation of Serial_Task */
  osThreadDef(Serial_Task, Serial_Thread, osPriorityNormal, 0, 128);
  Serial_TaskHandle = osThreadCreate(osThread(Serial_Task), NULL);

  /* definition and creation of RPM_Task */
  osThreadDef(RPM_Task, RPM_Thread, osPriorityNormal, 0, 128);
  RPM_TaskHandle = osThreadCreate(osThread(RPM_Task), NULL);

  /* definition and creation of Alarm_Task */
  osThreadDef(Alarm_Task, Alarm_Thread, osPriorityNormal, 0, 128);
  Alarm_TaskHandle = osThreadCreate(osThread(Alarm_Task), NULL);

  /* definition and creation of Initialization_ */
  osThreadDef(Initialization_, Initialization_Thread, osPriorityRealtime, 0, 128);
  Initialization_Handle = osThreadCreate(osThread(Initialization_), NULL);

  /* definition and creation of SoftStart_Task */
  osThreadDef(SoftStart_Task, SoftStart_Thread, osPriorityNormal, 0, 128);
  SoftStart_TaskHandle = osThreadCreate(osThread(SoftStart_Task), NULL);

  /* definition and creation of Chart_Task */
  osThreadDef(Chart_Task, Chart_Thread, osPriorityNormal, 0, 128);
  Chart_TaskHandle = osThreadCreate(osThread(Chart_Task), NULL);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8499;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 174;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 6295;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 102;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_FALLING;
  sSlaveConfig.TriggerFilter = 15;
  if (HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 150;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);
}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 419;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 10;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Bypass_relay_Pin | Power_relay_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0 
                           PC1 PC2 PC3 PC4 
                           PC5 PC8 PC9 PC10 
                           PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : Bypass_relay_Pin Power_relay_Pin */
  GPIO_InitStruct.Pin = Bypass_relay_Pin | Power_relay_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB12 PB13 PB14 PB15 
                           PB3 PB4 PB5 PB7 
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11 
                           PA12 PA13 PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/**
  * @brief Callback executado sempre que o DMA termina todas conversoes dos ADC�s
  * @note Disponibiliza o semaforo 'ADC_Conv_Semaphore', permitindo o calculo rms das conversoes
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
    xSemaphoreGiveFromISR(ADC_Conv_Semaphore, &Measure_TaskHandle);
}
/* USER CODE END 4 */

/* Measure_Thread function */
/**
  * @brief Thread para o calculo rms das medidas dos ADC's
  * @note Somente e executada se o semaforo 'ADC_Conv_Semaphore' esta disponivel
  * @note Para a medida Vrms e setado um offset em 2048, portando -Vpico = 0V e +Vpico = 3v3
  * @note Para a medida Irms e utilizado um retificador de precisao, nao sendo necessario o offset
  * @note Nos dois casos e realizado a media dos 16 valores convertidos, que e multiplicado pela
  * resolucao correspondente do seu ADC, esse valor e elevado ao quadrado
  * @note Esse processo e realizado 16 vezes, e entao e realizada outra media do valor quadratico
  * @note Por fim e feita a raiz quadrada dessa media quadratica, retornando o valor final rms
  */
void Measure_Thread(void const *argument)
{

  /* USER CODE BEGIN 5 */
  //!Vetor que recebe os valores convertidos de tensao de cada ciclo
  float ADC_Cycles_Vrms[16];
  //!Vetor que recebe os valores convertidos de corrente de cada ciclo
  float ADC_Cycles_Irms[16];
  //!Variavel de contagem de ciclos
  uint8_t count = 0;
  /* Infinite loop */
  while (1)
  {
    if (xSemaphoreTake(ADC_Conv_Semaphore, 10))
    {
      ADC_Cycles_Vrms[count] = pow(
          ((ADC_RES_Vrms * (ADC_Values_Vrms[0] + ADC_Values_Vrms[1] + ADC_Values_Vrms[2] + ADC_Values_Vrms[3] + 
          ADC_Values_Vrms[4] + ADC_Values_Vrms[5] + ADC_Values_Vrms[6] + ADC_Values_Vrms[7] + ADC_Values_Vrms[8] + 
          ADC_Values_Vrms[9] + ADC_Values_Vrms[10] + ADC_Values_Vrms[11] + ADC_Values_Vrms[12] + ADC_Values_Vrms[13] + 
          ADC_Values_Vrms[14] + ADC_Values_Vrms[15]) / 16)) - Vpike, 2);
      //!So realiza o calculo de Irms caso a corrente nominal tenha sido definida
      if (Nominal_Current_SET)
      {
        ADC_Cycles_Irms[count] = pow(
            ((ADC_RES_Irms * (ADC_Values_Irms[0] + ADC_Values_Irms[1] + ADC_Values_Irms[2] + ADC_Values_Irms[3] + 
            ADC_Values_Irms[4] + ADC_Values_Irms[5] + ADC_Values_Irms[6] + ADC_Values_Irms[7] + ADC_Values_Irms[8] + 
            ADC_Values_Irms[9] + ADC_Values_Irms[10] + ADC_Values_Irms[11] + ADC_Values_Irms[12] + ADC_Values_Irms[13] + 
            ADC_Values_Irms[14] + ADC_Values_Irms[15]) / 16)), 2);
      }
      count++;
      if (count == 16)
      {
        count = 0;
        Vrms = round(
            sqrt(
                (ADC_Cycles_Vrms[0] + ADC_Cycles_Vrms[1] + ADC_Cycles_Vrms[2] + ADC_Cycles_Vrms[3] + ADC_Cycles_Vrms[4] + 
                ADC_Cycles_Vrms[5] + ADC_Cycles_Vrms[6] + ADC_Cycles_Vrms[7] + ADC_Cycles_Vrms[8] + ADC_Cycles_Vrms[9] + 
                ADC_Cycles_Vrms[10] + ADC_Cycles_Vrms[11] + ADC_Cycles_Vrms[12] + ADC_Cycles_Vrms[13] + ADC_Cycles_Vrms[14] + 
                ADC_Cycles_Vrms[15]) / 16) *
            Vrms_ERROR);
        //!So realiza o calculo de Irms caso a corrente nominal tenha sido definida
        if (Nominal_Current_SET)
        {
          Irms = round(
              sqrt(
                  (ADC_Cycles_Irms[0] + ADC_Cycles_Irms[1] + ADC_Cycles_Irms[2] + ADC_Cycles_Irms[3] + ADC_Cycles_Irms[4] + 
                  ADC_Cycles_Irms[5] + ADC_Cycles_Irms[6] + ADC_Cycles_Irms[7] + ADC_Cycles_Irms[8] + ADC_Cycles_Irms[9] + 
                  ADC_Cycles_Irms[10] + ADC_Cycles_Irms[11] + ADC_Cycles_Irms[12] + ADC_Cycles_Irms[13] + ADC_Cycles_Irms[14] + 
                  ADC_Cycles_Irms[15]) / 16) *
              Irms_ERROR);
        }
      }
    }
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* Serial_Thread function */
/**
  * @brief Thread que controla o envio e o recebimento da serial
  * @note Possui um caso para cada chamada da interface
  * @note Verifica as flags e semaforos que indicam a necessidade de envio de confirmacao a interface
  */
void Serial_Thread(void const *argument)
{
  /* USER CODE BEGIN Serial_Thread */
  /* Infinite loop */
  //!Caracter que indica a ordem da interface
  uint8_t rx;
  //!Vetor que contem o tempo de subida ou descida definido na interface
  uint8_t rxtime[5];
  //!Variavel que recebe o tamanho da string a ser enviada
  uint8_t size;
  //!String para envio de dados solicitados a interface
  char tx[5];
  //!Stirng para envio de mensagens a interface
  char msg[20];
  while (1)
  {
    if (HAL_UART_Receive(&huart6, &rx, 1, 1) == HAL_OK)
    {
      switch (rx)
      {

      //!Procedimentos realizados quando a corrente nominal e definida na interface
      case CURRENTSET:
        if (HAL_UART_Receive(&huart6, rxtime, 4, 100) == HAL_OK)
        {
          taskENTER_CRITICAL();
          Nominal_Current = (uint16_t)(1000 * (rxtime[0] - '0') + 100 * (rxtime[1] - '0') + 10 * (rxtime[2] - '0') + (rxtime[3] - '0'));
          Ipike = Nominal_Current * 2;
          ADC_RES_Irms = (Nominal_Current * 6) / 4095.;
          Nominal_Current_SET = 1;
          i150 = round(Nominal_Current * 1.5);
          i200 = Nominal_Current * 2;
          taskEXIT_CRITICAL();
        }
        else
        {
          Nominal_Current_SET = 0;
          //!Sinaliza erro na definicao da corrente nominal para interface
          size = sprintf(msg, "Z\n");
          HAL_UART_Transmit(&huart6, (uint8_t *)msg, size, 10);
        }
        break;

      //!Retorna a medida Vrms para a interface
      case VRMSREAD:
        size = sprintf(tx, "P%d\n", Vrms);
        HAL_UART_Transmit(&huart6, (uint8_t *)tx, size, 10);
        break;

      //!Retorna a medida Irms para a interface
      case IRMSREAD:
        size = sprintf(tx, "U%d\n", Irms);
        HAL_UART_Transmit(&huart6, (uint8_t *)tx, size, 10);
        break;

      //!Procedimentos realizados quando e ordenado a aceleracao
      case START:
        if ((!fall && !rise) && increment > Increment_minimum)
        {
          if (HAL_UART_Receive(&huart6, rxtime, 2, 100) == HAL_OK)
          {
            taskENTER_CRITICAL();
            rising_time = (uint8_t)(10 * (rxtime[0] - '0') + rxtime[1] - '0');
            //!Retorna o sucesso da comunicacao a interface
            size = sprintf(msg, "A\n");
            HAL_UART_Transmit(&huart6, (uint8_t *)msg, size, 10);
            HAL_GPIO_WritePin(GPIOA, Power_relay_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, Bypass_relay_Pin, GPIO_PIN_RESET);
            rise = 1;
            fall = 0;
            HAL_TIM_Base_Start_IT(&htim1);
            HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
            __HAL_TIM_SET_AUTORELOAD(&htim1, 0);
            __HAL_TIM_SET_AUTORELOAD(&htim1, (rising_time * 100) - 1);
            taskEXIT_CRITICAL();
          }
          else
          {
            //!Indica que o comando nao foi realizado com sucesso
            size = sprintf(msg, "ITimeout\n");
            HAL_UART_Transmit(&huart6, (uint8_t *)msg, size, 10);
          }
        }
        else if (increment != Increment_minimum)
        {
          //!Indica para a interface que o motor esta em transicao
          size = sprintf(msg, "K\n");
          HAL_UART_Transmit(&huart6, (uint8_t *)msg, size, 10);
        }
        else
        {
          //!Indica para interface que o motor ja esta ligado
          size = sprintf(msg, "J\n");
          HAL_UART_Transmit(&huart6, (uint8_t *)msg, size, 10);
        }
        break;

      //!Procedimentos realizados quando e ordenado a desaceleracao
      case STOP:
        if ((!fall && !rise) && increment < (Arr_period - Pulse_Width))
        {
          if (HAL_UART_Receive(&huart6, rxtime, 2, 100) == HAL_OK)
          {
            taskENTER_CRITICAL();
            HAL_GPIO_WritePin(GPIOA, Power_relay_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, Bypass_relay_Pin, GPIO_PIN_RESET);
            falling_time = (uint8_t)(10 * (rxtime[0] - '0') + rxtime[1] - '0');
            //!Retorna o sucesso da comunicacao a interface
            size = sprintf(msg, "B\n");
            HAL_UART_Transmit(&huart6, (uint8_t *)msg, size, 10);
            fall = 1;
            rise = 0;
            HAL_TIM_Base_Start_IT(&htim1);
            HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
            __HAL_TIM_SET_AUTORELOAD(&htim1, 0);
            __HAL_TIM_SET_AUTORELOAD(&htim1, (falling_time * 100) - 1);
            taskEXIT_CRITICAL();
          }
          else
          {
            //!Indica que o comando nao foi realizado com sucesso
            size = sprintf(msg, "ITimeout\n");
            HAL_UART_Transmit(&huart6, (uint8_t *)msg, size, 10);
          }
        }
        else if (increment != (Arr_period - Pulse_Width))
        {
          //!Indica para a interface que o motor esta em transicao
          size = sprintf(msg, "K\n");
          HAL_UART_Transmit(&huart6, (uint8_t *)msg, size, 10);
        }
        else
        {
          //!Indica para interface que o motor ja esta desligado
          size = sprintf(msg, "L\n");
          HAL_UART_Transmit(&huart6, (uint8_t *)msg, size, 10);
        }
        break;

      //!Procedimentos realizados quando e ordenado a parada de emergencia
      case EMERGENCY:
        size = sprintf(msg, "IEmergency...\n");
        HAL_UART_Transmit(&huart6, (uint8_t *)msg, size, 10);
        Emergency();
        break;
      }
    }

    //!Indica o fim da aceleracao a interface e envia os valores para o grafico de subida
    if (rise_ok)
    {
      rise_ok = 0;
      size = sprintf(msg, "r");
      HAL_UART_Transmit(&huart6, (uint8_t *)msg, size, 10);
      for (uint8_t count = 0; count < Aquisitions; count++)
      {
        size = sprintf(msg, "%d#", Irms_data_rise[count]);
        HAL_UART_Transmit(&huart6, (uint8_t *)msg, size, 10);
      }
      size = sprintf(msg, "\n");
      HAL_UART_Transmit(&huart6, (uint8_t *)msg, size, 10);
    }

    //!Indica o fim da desaceleracao a interface e envia os valores para o grafico de descida
    if (fall_ok)
    {
      fall_ok = 0;
      size = sprintf(msg, "f");
      HAL_UART_Transmit(&huart6, (uint8_t *)msg, size, 10);
      for (uint8_t count = 0; count < Aquisitions; count++)
      {
        size = sprintf(msg, "%d#", Irms_data_fall[count]);
        HAL_UART_Transmit(&huart6, (uint8_t *)msg, size, 10);
      }
      size = sprintf(msg, "\n");
      HAL_UART_Transmit(&huart6, (uint8_t *)msg, size, 10);
    }

    //!Indica uma emergencia a interface
    if (xSemaphoreTake(Emergency_Semaphore, 1))
    {
      size = sprintf(msg, "e\n");
      HAL_UART_Transmit(&huart6, (uint8_t *)msg, size, 10);
    }
    //!Indica Irms > 150% a interface
    if (xSemaphoreTake(E150_Semaphore, 1))
    {
      size = sprintf(msg, "c\n");
      HAL_UART_Transmit(&huart6, (uint8_t *)msg, size, 10);
    }
    osDelay(1);
  }
  /* USER CODE END Serial_Thread */
}

/* RPM_Thread function */
void RPM_Thread(void const *argument)
{
  /* USER CODE BEGIN RPM_Thread */
  /* Infinite loop */
  while (1)
  {
    osDelay(500);
  }
  /* USER CODE END RPM_Thread */
}

/* Alarm_Thread function */
/**
  * @brief Thread que checa  Irms>150%, Irms>200% e desenergizacao do motor
  * @note Caso Irms>150% o pulso retorna 3 unidades (1-100)
  * @note Caso Irms>200% e acionado a emergencia
  * @note Caso haja desenergizacao do motor e acionado a emergencia
  */
void Alarm_Thread(void const *argument)
{
  /* USER CODE BEGIN Alarm_Thread */
  /* Infinite loop */
  while (1)
  {
    taskENTER_CRITICAL();
    //!Checa se Irms>150% durante a aceleracao
    if (Irms >= i150 && rise && increment < 80)
    {
      xSemaphoreGive(E150_Semaphore);
      increment = increment + 3;
    }
    //!Checa se Irms>200%
    if (Irms > i200)
    {
      xSemaphoreGive(Emergency_Semaphore);
      Emergency();
    }
    //!Checa se o motor foi desenergizado durante a aceleracao ou enquanto estava em funcionamento
    if (Vrms < 13 && HAL_GPIO_ReadPin(GPIOA, Power_relay_Pin) && increment < 70)
    {
      xSemaphoreGive(Emergency_Semaphore);
      Emergency();
    }
    taskEXIT_CRITICAL();
    osDelay(100);
  }
  /* USER CODE END Alarm_Thread */
}

/* Initialization_Thread function */
/**
  * @brief Thread executada sempre que o programa e inicializado
  * @note Possui prioridade RealTime e e suspendida apos executada uma vez
  * @note Inicializa os perifericos a serem utilizados e cria os semaforos
  */
void Initialization_Thread(void const *argument)
{
  /* USER CODE BEGIN Initialization_Thread */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Values_Vrms, 16);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADC_Values_Irms, 16);
  HAL_TIM_Base_Start(&htim2);
  vSemaphoreCreateBinary(ADC_Conv_Semaphore);
  vSemaphoreCreateBinary(Increment_Semaphore);
  vSemaphoreCreateBinary(Emergency_Semaphore);
  vSemaphoreCreateBinary(E150_Semaphore);
  vTaskSuspend(Initialization_Handle);
  /* Infinite loop */
  while (1)
  {
    osDelay(1);
  }
  /* USER CODE END Initialization_Thread */
}

/* SoftStart_Thread function */
/**
  * @brief Thread responsavel por atualizar o puslo de controle e monitora-lo
  * @note Realiza o deslocamento do puslo de controle
  * @note Indica com flags quando o motor foi acelerado ou desacelerado
  */
void SoftStart_Thread(void const *argument)
{
  /* USER CODE BEGIN SoftStart_Thread */
  /* Infinite loop */
  while (1)
  {
    //!A Thread so e executada quando ha atualizacao na variavel 'increment'
    if (xSemaphoreTake(Increment_Semaphore, 1))
    {
      if (fall)
      {
        //!Seta o inicio do pulso para 'increment' e seu final para 'increment' + 'Pulse_Width'
        if (__HAL_TIM_GET_COUNTER(&htim3) <= increment)
        {
          __HAL_TIM_SET_AUTORELOAD(&htim3, increment + Pulse_Width);
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, increment);
        }
        //!Procedimentos realizados quando o motor e desacelerado
        if (increment >= (Arr_period - Pulse_Width))
        {
          taskENTER_CRITICAL();
          HAL_GPIO_WritePin(GPIOA, Power_relay_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOA, Bypass_relay_Pin, GPIO_PIN_RESET);
          HAL_TIM_Base_Stop_IT(&htim1);
          fall = 0;
          fall_ok = 1;
          HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
          taskEXIT_CRITICAL();
        }
      }
      if (rise)
      {
        if (increment > Arr_period)
          increment = Increment_minimum;
        //!Seta o inicio do pulso para 'increment' e seu final para 'increment' + 'Pulse_Width'
        if (__HAL_TIM_GET_COUNTER(&htim3) <= increment)
        {
          __HAL_TIM_SET_AUTORELOAD(&htim3, increment + Pulse_Width);
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, increment);
        }
        //!Procedimentos realizados quando o motor e acelerado
        if (increment == Increment_minimum)
        {
          taskENTER_CRITICAL();
          HAL_GPIO_WritePin(GPIOA, Bypass_relay_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOA, Power_relay_Pin, GPIO_PIN_SET);
          rise_ok = 1;
          HAL_TIM_Base_Stop_IT(&htim1);
          rise = 0;
          HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
          taskEXIT_CRITICAL();
        }
      }
    }
    osDelay(10);
  }
  /* USER CODE END SoftStart_Thread */
}

/* Chart_Thread function */
/**
  * @brief Thread responsavel por fazer a aquisicao dos graficos de subida e descida
  * @note Checa se o motor esta acelerando ou desacelerando e preenche o respectivo vetor do grafico
  * @note Realiza 25 medidas durante a transicao ('increment % 4', onde 'increment' varia de 1-100)
  * @note Realiza a medida 26 ao final de cada transicao
  */
void Chart_Thread(void const *argument)
{
  /* USER CODE BEGIN Chart_Thread */
  //!Variavel de contagem das aquisicoes
  uint8_t count = 0;
  /* Infinite loop */
  while (1)
  {
    if (rise)
    {
      if ((!(increment % 4)) && check)
      {
        check = 0;
        Irms_data_rise[count] = Irms;
        if (count < Aquisitions - 1)
          count++;
      }
      else if (increment == 1)
      {
        check = 1;
        Irms_data_rise[Aquisitions - 1] = Irms;
      }
    }
    if (fall)
    {
      if (increment == 1 && check)
      {
        check = 0;
        Irms_data_fall[0] = Irms;
      }
      else if ((!(increment % 4)) && check)
      {
        if (count < Aquisitions - 1)
          count++;
        check = 0;
        Irms_data_fall[count] = Irms;
      }
      if (count == Aquisitions)
      {
        check = 1;
      }
    }
    if (!rise && !fall)
      count = 0;
    osDelay(1);
  }
  /* USER CODE END Chart_Thread */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  //!Checa se a interrupcao foi feita por TIM1
  if (htim->Instance == TIM1)
  {
    //!Disponibiliza o semaforo 'Increment_Semaphore' e seta a flag 'check' indicando atualizacao de 'increment'
    xSemaphoreGiveFromISR(Increment_Semaphore, &SoftStart_TaskHandle);
    check = 1;
    //!Checa se o motor esta acelerando e se Irms<150% para deslocar o pulso
    if (rise && Irms < i150)
      increment--;
    //!Checa se o motor esta desacelerando para deslocar o pulso
    if (fall)
      increment++;
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
