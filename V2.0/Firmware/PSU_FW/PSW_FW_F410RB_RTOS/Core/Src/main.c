/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "display.h"
#include "leds.h"
#include "stats.h"
#include "uart.h"
#include "analog.h"
#include "buttons.h"
#include "mcp4018.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for uartTxTask */
osThreadId_t uartTxTaskHandle;
const osThreadAttr_t uartTxTask_attributes = { .name = "uartTxTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for uartRxTask */
osThreadId_t uartRxTaskHandle;
const osThreadAttr_t uartRxTask_attributes = { .name = "uartRxTask",
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityNormal1, };
/* Definitions for oledTask */
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = { .name = "oledTask", .stack_size =
		256 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for initPsuTask */
osThreadId_t initPsuTaskHandle;
const osThreadAttr_t initPsuTask_attributes = { .name = "initPsuTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityHigh7, };
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = { .name = "ledTask", .stack_size = 128
		* 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for regulatorCtrlTa */
osThreadId_t regulatorCtrlTaHandle;
const osThreadAttr_t regulatorCtrlTa_attributes = { .name = "regulatorCtrlTa",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for btnsTask */
osThreadId_t btnsTaskHandle;
const osThreadAttr_t btnsTask_attributes = { .name = "btnsTask", .stack_size =
		512 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for mcp4018Task */
osThreadId_t mcp4018TaskHandle;
const osThreadAttr_t mcp4018Task_attributes = { .name = "mcp4018Task",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* USER CODE BEGIN PV */
Stats psuStats;
uartRxData uartRx;
uint8_t interruptFlags = INT_FLAG_CLEAR;

extern uint16_t analogBuffDMA[ANALOG_DMA_BUFF_SIZE];
extern uint16_t analogBuffFinal[ANALOG_DMA_BUFF_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
void startUartTxTask(void *argument);
void startUartRxTask(void *argument);
void startOledTask(void *argument);
void startInitPsuTask(void *argument);
void startLedTask(void *argument);
void startRegulatorCtrlTask(void *argument);
void startBtnsTask(void *argument);
void startMcp4018Task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_TIM11_Init();
	MX_USART2_UART_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
//	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) analogBuffDMA, ANALOG_DMA_BUFF_SIZE);
	HAL_TIM_Base_Start(&htim1);
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of uartTxTask */
	uartTxTaskHandle = osThreadNew(startUartTxTask, NULL,
			&uartTxTask_attributes);

	/* creation of uartRxTask */
	uartRxTaskHandle = osThreadNew(startUartRxTask, NULL,
			&uartRxTask_attributes);

	/* creation of oledTask */
	oledTaskHandle = osThreadNew(startOledTask, NULL, &oledTask_attributes);

	/* creation of initPsuTask */
	initPsuTaskHandle = osThreadNew(startInitPsuTask, NULL,
			&initPsuTask_attributes);

	/* creation of ledTask */
	ledTaskHandle = osThreadNew(startLedTask, NULL, &ledTask_attributes);

	/* creation of regulatorCtrlTa */
	regulatorCtrlTaHandle = osThreadNew(startRegulatorCtrlTask, NULL,
			&regulatorCtrlTa_attributes);

	/* creation of btnsTask */
	btnsTaskHandle = osThreadNew(startBtnsTask, NULL, &btnsTask_attributes);

	/* creation of mcp4018Task */
	mcp4018TaskHandle = osThreadNew(startMcp4018Task, NULL,
			&mcp4018Task_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 3;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
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
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 9999;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void) {

	/* USER CODE BEGIN TIM11_Init 0 */

	/* USER CODE END TIM11_Init 0 */

	/* USER CODE BEGIN TIM11_Init 1 */

	/* USER CODE END TIM11_Init 1 */
	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 8400 - 1;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 65535;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM11_Init 2 */

	/* USER CODE END TIM11_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(REG_EN_GPIO_Port, REG_EN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, VI_LED_Pin | OE_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CC_LED_GPIO_Port, CC_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : REG_EN_Pin CC_LED_Pin */
	GPIO_InitStruct.Pin = REG_EN_Pin | CC_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : VI_LED_Pin OE_LED_Pin */
	GPIO_InitStruct.Pin = VI_LED_Pin | OE_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : nSW_INT_Pin */
	GPIO_InitStruct.Pin = nSW_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(nSW_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : nSW_VI_Pin */
	GPIO_InitStruct.Pin = nSW_VI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(nSW_VI_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : nSW_DW_Pin nSW_UP_Pin nSW_OE_Pin */
	GPIO_InitStruct.Pin = nSW_DW_Pin | nSW_UP_Pin | nSW_OE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : OLED_RST_Pin */
	GPIO_InitStruct.Pin = OLED_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(OLED_RST_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	uartRxIntHandler(&uartRx);
}

double potToVoltage(uint8_t potVal) {
	double potCalc = (((double) potVal / 128.0)) * 5000.0;
	double temp = 800.0 * ((4690.0 / potCalc) + 1.0);
	return temp;

}

uint8_t voltageToPot(double vVal) {
	double potCalc = 4960.0 / (((double) vVal / 800.0) - 1.0);
	potCalc = (potCalc / 5000.0) * 128.0;
	return (uint8_t) potCalc;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//	uartTxString("ADC INT FOR\n");
//	for (int i = 0; i < ANALOG_DMA_BUFF_SIZE; i++) {
//	char buff[90];
//	sprintf(buff, "a %d\n | ", analogBuffDMA[0]);
//	uartTxString(buff);
//	sprintf(buff, "b %d\n | ", analogBuffDMA[1]);
//	uartTxString(buff);
//	sprintf(buff, "c %d\n | ", analogBuffDMA[2]);
//	uartTxString(buff);
//	}
	memcpy(analogBuffFinal, analogBuffDMA, sizeof(analogBuffDMA));

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == nSW_INT_Pin) // If The INT Source Is button interrupt
	{
		interruptFlags |= INT_FLAG_BTN;
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_startUartTxTask */
/**
 * @brief  Function implementing the uartTxTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_startUartTxTask */
void startUartTxTask(void *argument) {
	/* USER CODE BEGIN 5 */
//	HAL_UART_Transmit_DMA(&huart2, uartTxBuff, UART_DMA_BUFFER_SIZE);
	/* Infinite loop */
	osThreadTerminate(osThreadGetId());
	for (;;) {
		osDelay(10);
	}
	osThreadTerminate(osThreadGetId());
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startUartRxTask */
/**
 * @brief Function implementing the uartRxTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startUartRxTask */
void startUartRxTask(void *argument) {
	/* USER CODE BEGIN startUartRxTask */
	/* Infinite loop */
	for (;;) {
		HAL_UART_Receive_DMA(&huart2, (uint8_t*) &uartRx.uartRxChar,
		UART_RX_CHAR_SIZE);
		uartRxStringParser(&uartRx, &psuStats);
		osDelay(25);
	}
	osThreadTerminate(osThreadGetId());
	/* USER CODE END startUartRxTask */
}

/* USER CODE BEGIN Header_startOledTask */
/**
 * @brief Function implementing the oledTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startOledTask */
void startOledTask(void *argument) {
	/* USER CODE BEGIN startOledTask */
	double Vin = 0;
	double Vout = 0;
	double Iout = 0;
	displayInit();

	/* Infinite loop */
	for (;;) {
		Vin = analogReadVin(analogBuffFinal[ANALOG_BUFF_V_IN_POS]);
		Vout = analogReadVout(analogBuffFinal[ANALOG_BUFF_V_OUT_POS]);
		Iout = analogReadIOut(analogBuffFinal[ANALOG_BUFF_I_SENSE_POS]);
		displayVoltageCurrent(&psuStats, Vin, Vout, Iout);
		osDelay(500);
		Vin++;
	}
	osThreadTerminate(osThreadGetId());
	/* USER CODE END startOledTask */
}

/* USER CODE BEGIN Header_startInitPsuTask */
/**
 * @brief Function implementing the initPsuTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startInitPsuTask */
void startInitPsuTask(void *argument) {
	/* USER CODE BEGIN startInitPsuTask */
	ledsInit();
	psuStats.vSet = V_DEFAULT;
	psuStats.iSet = I_DEFAULT;
	psuStats.iLim = I_LIM_DEFAULT;
	psuStats.VI = VI_DEFAULT;
	psuStats.OE = OE_DEFAULT;

	ledSet(&psuStats);
	osThreadTerminate(osThreadGetId());	// do not need the task after init
//should never get here!
	ledOn(LED_NUM_CC);
	ledOff(LED_NUM_OE);
	ledOn(LED_NUM_VI);
	/* Infinite loop */
	for (;;) {
		ledToggle(LED_NUM_CC);
		ledToggle(LED_NUM_OE);
		ledToggle(LED_NUM_VI);
		osDelay(50);
	}
	/* USER CODE END startInitPsuTask */
}

/* USER CODE BEGIN Header_startLedTask */
/**
 * @brief Function implementing the ledTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startLedTask */
void startLedTask(void *argument) {
	/* USER CODE BEGIN startLedTask */
	osDelay(2000);	//wait for init sequence to complete
	if (psuStats.OE == OE_ENABLED) {
		ledOn(LED_NUM_OE);
	} else {
		ledOff(LED_NUM_OE);
	}
	if (psuStats.VI == VI_V_SEL) {
		ledOn(LED_NUM_VI);
	} else {
		ledOff(LED_NUM_VI);
	}
	if (psuStats.iLim == I_LIM_SET) {
		ledOn(LED_NUM_CC);
	} else {
		ledOff(LED_NUM_CC);
	}
	/* Infinite loop */
	for (;;) {
		if (psuStats.OE == OE_ENABLED) {
			ledOn(LED_NUM_OE);
		} else {
			ledOff(LED_NUM_OE);
		}
		if (psuStats.VI == VI_V_SEL) {
			ledOn(LED_NUM_VI);
		} else {
			ledOff(LED_NUM_VI);
		}
		if (psuStats.iLim == I_LIM_SET) {
			ledOn(LED_NUM_CC);
		} else {
			ledOff(LED_NUM_CC);
		}
		osDelay(25);
	}
	/* USER CODE END startLedTask */
}

/* USER CODE BEGIN Header_startRegulatorCtrlTask */
/**
 * @brief Function implementing the regulatorCtrlTa thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startRegulatorCtrlTask */
void startRegulatorCtrlTask(void *argument) {
	/* USER CODE BEGIN startRegulatorCtrlTask */
	//Regulator enable pin is made active low
	HAL_GPIO_WritePin(REG_EN_GPIO_Port, REG_EN_Pin, GPIO_PIN_SET);
	/* Infinite loop */
	for (;;) {
		if (psuStats.OE == OE_ENABLED) {
			HAL_GPIO_WritePin(REG_EN_GPIO_Port, REG_EN_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(REG_EN_GPIO_Port, REG_EN_Pin, GPIO_PIN_SET);
		}
		osDelay(50);
	}
	/* USER CODE END startRegulatorCtrlTask */
}

/* USER CODE BEGIN Header_startBtnsTask */
/**
 * @brief Function implementing the btnsTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startBtnsTask */
void startBtnsTask(void *argument) {
	/* USER CODE BEGIN startBtnsTask */
	/* Infinite loop */
	for (;;) {
		if (interruptFlags & INT_FLAG_BTN) {
			interruptFlags &= ~INT_FLAG_BTN;
			uint8_t btns = btnsRead();
			if (btns == BTN_NUM_VI) {
				if (psuStats.VI == VI_V_SEL) {
					psuStats.VI = VI_I_SEL;
				} else if (psuStats.VI == VI_I_SEL) {
					psuStats.VI = VI_V_SEL;
				} else { //something went wrong, reinit psu
//					fatalErrorScreen();
//					initPSU();
				}
			} else if (btns == BTN_NUM_UP) {
				if (psuStats.VI == VI_V_SEL) {
					psuStats.vSet--;	//reduce R to increase V
//					MCP4018_WriteVal(psuStats.vSet);
				} else if (psuStats.VI == VI_I_SEL) {
					psuStats.iSet += 50;
				} else { //something went wrong, reinit psu
//					fatalErrorScreen();
//					initPSU();
				}
			} else if (btns == BTN_NUM_DWN) {
				if (psuStats.VI == VI_V_SEL) {
					psuStats.vSet++;	//increase R to decrease V
//					MCP4018_WriteVal(psuStats.vSet);
				} else if (psuStats.VI == VI_I_SEL) {
					psuStats.iSet -= 50;
				} else { //something went wrong, reinit psu
//					fatalErrorScreen();
//					initPSU();
				}

			} else if (btns == BTN_NUM_OE) {
				if (psuStats.OE == OE_ENABLED) {
					psuStats.OE = OE_DISABLED;
				} else if (psuStats.OE == OE_DISABLED) {
					psuStats.OE = OE_ENABLED;
				} else { //something went wrong, reinit psu
//					fatalErrorScreen();
//					initPSU();
				}

			}
		}
		osDelay(10);
	}
	/* USER CODE END startBtnsTask */
}

/* USER CODE BEGIN Header_startMcp4018Task */
/**
 * @brief Function implementing the mcp4018Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startMcp4018Task */
void startMcp4018Task(void *argument) {
	/* USER CODE BEGIN startMcp4018Task */
	/* Infinite loop */
	for (;;) {
		mcp4018WriteVal(psuStats.vSet);
		osDelay(10);
	}
	/* USER CODE END startMcp4018Task */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
