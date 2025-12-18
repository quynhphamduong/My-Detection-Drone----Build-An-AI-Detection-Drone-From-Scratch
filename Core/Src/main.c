/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nRF24L01.h"
#include "PID.h"
#include "Drone_control.h"
#include "Oneshot125_Esc_Control.h"
#include "bmp180.h"
#include "bno055_stm32.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define ADC_MAX 4095
#define PWM_MIN 1000 // 1000 us
#define PWM_MAX 2000 // 2000 us
#define DEADZONE 50
#define DEBUG 1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
uint8_t uart2_rx[16];
uint32_t high[4];

uint8_t buff_esc[128];

#ifdef SIMULATION
int pwm;
#endif

uint32_t adc_value;
uint8_t buff_esc[128];

float value[7];
uint8_t TCP_Mess[128];
Motor_speed_Typedef speed;
PIDControllers_Typedef pitch;
PIDControllers_Typedef pitch_rate;
PIDControllers_Typedef roll;
PIDControllers_Typedef roll_rate;
PIDControllers_Typedef yaw;
PIDControllers_Typedef yaw_rate;

Drone_Calculation_Typedef calculation;
Drone_Control_Typedef control;


xTaskHandle ESC_Handle;
xTaskHandle NRF_Handle;
xTaskHandle BNO_Handle;
xTaskHandle BMP_Handle;
xTaskHandle ADC_Handle;
xTaskHandle USB_Handle;
xTaskHandle ZIGBEE_Handle;
xTaskHandle NEO7M_Handle;
xTaskHandle GPS_Handle;
xTaskHandle TCP_Handle;

xQueueHandle xReceivedADC;
xQueueHandle xBNOQueue;
xQueueHandle xBMPQueue;
xQueueHandle xGPSQueue;
xQueueHandle xTCPMessage;

xSemaphoreHandle xAdcSem;
xSemaphoreHandle xUSART1Sem;
xSemaphoreHandle xUSART2Sem;
xSemaphoreHandle xUSART6Sem;

xSemaphoreHandle xTCPSem;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xAdcSem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xUSART1Sem, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
  if (huart->Instance == USART2)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xUSART2Sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  if (huart->Instance==USART6)
  {
	  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	  xSemaphoreGiveFromISR(xUSART6Sem, &xHigherPriorityTaskWoken);
	  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void ESC_Task(void *argument);
void NRF_Task(void *argument);
void BNO_Task(void *argument);
void BMP_Task(void *argument);
void ADC_Task(void *argument);
void USB_UART_Task(void *argument);
void ZIGBEE_Task(void *argument);
void NEO7M_Task(void *argument);
void GPS_DATA_Task(void *argument);
void TCP_Message_Handling_Task(void *argument);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
  float sample = powf(10, -3);
  pidControllersInit(&yaw, 0, 0, 0, 0.1, sample, 360,-360);
  pidControllersInit(&yaw_rate, 0, 0, 0, 0.1, sample, 1500, -1500);
  pidControllersInit(&pitch, 0, 0, 0, 0.1, sample, 10, -10);
  pidControllersInit(&pitch_rate, 0, 0, 0, 0.1, sample, 1500, -1500);
  pidControllersInit(&roll, 0, 0, 0, 0.1, sample, 10, -10);
  pidControllersInit(&roll_rate, 0,0, 0, 0.1, sample, 1500, -1500);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  xAdcSem = xSemaphoreCreateBinary();
  xUSART1Sem = xSemaphoreCreateBinary();
  xUSART2Sem = xSemaphoreCreateBinary();
  xUSART6Sem = xSemaphoreCreateBinary();
  xTCPSem = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  xReceivedADC = xQueueCreate(4, sizeof(uint32_t));
  xBNOQueue = xQueueCreate(2, sizeof(bno055_vector_t));
  xBMPQueue = xQueueCreate(1, sizeof(float));
  xGPSQueue = xQueueCreate(128,sizeof(uint8_t));
  xTCPMessage = xQueueCreate(128,sizeof(uint8_t));

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(ESC_Task, "ESC", 512, NULL, 2, &ESC_Handle);
  xTaskCreate(TCP_Message_Handling_Task,"TCP",512,NULL,1,&TCP_Handle);
  xTaskCreate(BNO_Task, "BNO", 256, NULL, 1, &BNO_Handle);
  xTaskCreate(BMP_Task, "BMP", 256, NULL, 1, &BMP_Handle);
  xTaskCreate(ADC_Task, "ADC", 256, NULL, 1, &ADC_Handle);
  xTaskCreate(USB_UART_Task, "USB", 512, NULL, 2, &USB_Handle);
  xTaskCreate(ZIGBEE_Task, "ZIGBEE", 256, NULL, 2, &ZIGBEE_Handle);
  xTaskCreate(NEO7M_Task, "NEO7M", 256, NULL, 2, &NEO7M_Handle);
  xTaskCreate(GPS_DATA_Task, "GPS", 256, NULL, 0, &GPS_Handle);

  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 42000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS2_Pin|CE2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CS2_Pin CE2_Pin */
  GPIO_InitStruct.Pin = CS2_Pin|CE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ2_Pin */
  GPIO_InitStruct.Pin = IRQ2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IRQ2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void ESC_Task(void *argument)
{
bno055_vector_t euler, gyro;
  float altitude;


  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  Calibration_RTOS(&htim3);
  while (1)
  {

    if (xQueueReceive(xBNOQueue, &euler, portMAX_DELAY) == pdTRUE)
    {
    }
    if (xQueueReceive(xBNOQueue, &gyro, portMAX_DELAY) == pdTRUE)
    {
    }
    if (xQueueReceive(xBMPQueue, &altitude, portMAX_DELAY) == pdTRUE)
    {
    }

    if(TCP_Mess[0]=='W')
   		{
   			roll.expected=roll.expected+0.1f;
   		}
   		else if(TCP_Mess[0]=='S')
   		{
   			yaw.expected=yaw.expected-0.1f;
   		}
   		else if(TCP_Mess[0]=='A')
   		{
   			yaw.expected=yaw.expected+0.1f;
   		}
   		else if(TCP_Mess[0]=='D')
   		{
   			roll.expected=roll.expected-0.1f;
   		}
    xSemaphoreTake(xTCPSem,portMAX_DELAY);
    pidUpdate(&pitch, euler.z, pitch.expected);
    calculation.picth_rate_reference = pitch.u;
    pidUpdate(&pitch_rate, gyro.z, calculation.picth_rate_reference);
    pidUpdate(&roll, euler.y,  roll.expected);
    calculation.roll_rate_reference=roll.u;
    pidUpdate(&roll_rate, gyro.y, calculation.roll_rate_reference);
    pidUpdate(&yaw, euler.x, yaw.expected);
    calculation.yaw_rate_reference = yaw.u;
    pidUpdate(&yaw_rate, gyro.x, calculation.yaw_rate_reference);

#if DEBUG ==1
    speed.speed1 = high[0] - (uint32_t)pitch_rate.u + (uint32_t)roll_rate.u - (uint32_t)yaw_rate.u;
    speed.speed2 = high[1] + (uint32_t)pitch_rate.u + (uint32_t)roll_rate.u + (uint32_t)yaw_rate.u;
    speed.speed3 = high[2] + (uint32_t)pitch_rate.u - (uint32_t)roll_rate.u - (uint32_t)yaw_rate.u;
    speed.speed4 = high[3] - (uint32_t)pitch_rate.u - (uint32_t)roll_rate.u + (uint32_t)yaw_rate.u;
#else
    speed.speed1 = 2000;
	speed.speed2 = 2000;
	speed.speed3 = 2000;
	speed.speed4 = 2000;
#endif
    Control4Motor(&htim3, &speed);
    sprintf((char*)buff_esc,"%f/%f/%f\n",euler.y,euler.z,euler.x);
    HAL_UART_Transmit(&huart1, buff_esc, 128,10);
	xSemaphoreGive(xTCPSem);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void BNO_Task(void *argument)
{
  bno055_vector_t euler_t, gyro_t;
  bno055_assignI2C(&hi2c1);
  bno055_setup();
  bno055_setOperationModeNDOF();
  while (1)
  {
    gyro_t = bno055_getVectorGyroscope();
    euler_t = bno055_getVectorEuler();
    xQueueSend(xBNOQueue, &euler_t, portMAX_DELAY);
    xQueueSend(xBNOQueue, &gyro_t, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void BMP_Task(void *argument)
{
  BMP180_Handle_t bmp180;
  float altitude_t;
  if (BMP180_Init(&bmp180, &hi2c2, 3) != HAL_OK) // OSS = 3 (highest resolution)
  {
  }
  while (1)
  {
    altitude_t = BMP180_ReadAltitude(&bmp180, BMP180_STD_ATM_PRESS); // 101325 Pa
    xQueueSend(xBMPQueue, &altitude_t, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void ADC_Task(void *argument)
{
  HAL_ADC_Start_DMA(&hadc1, &adc_value, 1);
  while (1)
  {
    if (xSemaphoreTake(xAdcSem, portMAX_DELAY) == pdTRUE)
    {
      HAL_ADC_Start_DMA(&hadc1, &adc_value, 1);
    }
  }
}

void ZIGBEE_Task(void *argument)
{


  HAL_UART_Receive_IT(&huart2, uart2_rx, 16);
  while (1)
  {
    if (xSemaphoreTake(xUSART2Sem, portMAX_DELAY) == pdTRUE)
    {
      HAL_UART_Receive_IT(&huart2, uart2_rx, 16);
    }
  }
}

void USB_UART_Task(void *argument)
{
	uint8_t uart1_rx[128];
	uint8_t buffer;
	HAL_UART_Receive_IT(&huart1, &buffer, 1);
	static int i=0;
	while(1)
	{
		if (xSemaphoreTake(xUSART1Sem, portMAX_DELAY) == pdTRUE)
		{

			if(buffer=='\n' || i==127)
			{
				i=0;
				buffer=0;
				int a=strlen((char*)uart1_rx);
				for(int u=0;u<128;u++)
				{
					xQueueSend(xTCPMessage,uart1_rx+u,portMAX_DELAY);
				}
				for(int u=0;u<a;u++)
				{
					uart1_rx[u]=0;
				}
				goto reset;
			}
			memcpy(uart1_rx+i,&buffer,1);
			i++;
			reset:
			HAL_UART_Receive_IT(&huart1, &buffer, 1);
		}
	}
}

void NEO7M_Task(void *argument)
{

	uint8_t i=0;
	uint8_t a;
	uint8_t uart6_rx[128];
	uint8_t buffer;
	uint8_t MessageID[6];
	uint8_t NeededID[]="$GPRMC";
	HAL_UART_Receive_IT(&huart6, &buffer, 1);
	while (1)
	{
		if (xSemaphoreTake(xUSART6Sem, portMAX_DELAY) == pdTRUE)
		{
			if(buffer=='\n')
			{
				i=0;
				a=strlen((char*)uart6_rx);
				memcpy(MessageID,uart6_rx,6);
				if(strcmp((char*)MessageID,(char*)NeededID))
				{
					for(int u=0;u<128;u++)
					{
						xQueueSend(xGPSQueue,MessageID+u,portMAX_DELAY);
					}
				}
				for(int u=0;u<a;u++)
				{
					uart6_rx[u]=0;
				}
				goto Init_Interupt;
			}
			memcpy(uart6_rx+i,&buffer,1);
			i++;
Init_Interupt:
			HAL_UART_Receive_IT(&huart6, &buffer, 1);

		}
	}

}

void GPS_DATA_Task(void *argument)
{
	while(1)
	{

	}
}


void TCP_Message_Handling_Task(void *argument)
{

	uint8_t *string=malloc(sizeof(char)*30);
	while(1)
	{
		for(int i=0;i<128;i++)
		{
			if (xQueueReceive(xTCPMessage, TCP_Mess+i, portMAX_DELAY) == pdTRUE)
			{

			}
		}
		xSemaphoreTake(xTCPSem,portMAX_DELAY);
		int count_string=0;
		int track_string=0;
		for(int i=1;i<128;i++)
		{
			if(TCP_Mess[i]!='/'&&TCP_Mess[i]!=0&&count_string<=7&&track_string<30)
			{
				string[track_string]=TCP_Mess[i];
				track_string++;
			}
			else if((TCP_Mess[i]=='/'||TCP_Mess[i]==0)&&count_string<7&&track_string<30)
			{
				value[count_string]=atof((char*)string);
				count_string++;
				track_string=0;
				memset(string,0,30);
			}
			else if(track_string>=30)
			{
				goto escape_function;
			}
			if (count_string>=7||TCP_Mess[i]==0)
			{
				break;
			}
		}
		if(TCP_Mess[0]=='R')
		{
			AdjustPIDParams(&roll, value+1,value+2, value+3);
			calculation.roll_reference=value[0];
			AdjustPIDParams(&roll_rate, value+4, value+5, value+6);
		}
		else if(TCP_Mess[0]=='P')
		{
			AdjustPIDParams(&pitch, value+1,value+2, value+3);
			calculation.picth_reference=value[0];
			AdjustPIDParams(&pitch_rate, value+4, value+5, value+6);
		}
		else if(TCP_Mess[0]=='Y')
		{
			AdjustPIDParams(&yaw , value+1,value+2, value+3);
			calculation.yaw_reference=value[0];
			AdjustPIDParams(&yaw_rate, value+4, value+5, value+6);
		}
		else if(TCP_Mess[0]=='H')
		{
			for(int i=0;i<4;i++)
			{
				high[i]=value[i];
			}
		}
		else if(TCP_Mess[0]=='C')
		{
			HAL_UART_Transmit(&huart1,(uint8_t*) "A", 1, 1);
		}

		escape_function:
		xSemaphoreGive(xTCPSem);

	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    osDelay(500);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
