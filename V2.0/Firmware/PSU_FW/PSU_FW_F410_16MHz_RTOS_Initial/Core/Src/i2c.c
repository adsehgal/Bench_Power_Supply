/**
 ******************************************************************************
 * @file    i2c.c
 * @brief   This file provides code for the configuration
 *          of the I2C instances.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
#include "cmsis_os.h"

osMutexId i2cRdMutexId;
osMutexDef(i2cRdMutex);

osMutexId i2cWrMutexId;
osMutexDef(i2cWrMutex);

osMutexId i2cRdMemMutexId;
osMutexDef(i2cRdMemMutex);

osMutexId i2cWrMemMutexId;
osMutexDef(i2cWrMemMutex);

osMutexId i2cWrMemByteMutexId;
osMutexDef(i2cWrMemByteMutex);
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

/* I2C1 init function */
void MX_I2C1_Init(void) {

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

void HAL_I2C_MspInit(I2C_HandleTypeDef *i2cHandle) {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (i2cHandle->Instance == I2C1) {
		/* USER CODE BEGIN I2C1_MspInit 0 */

		/* USER CODE END I2C1_MspInit 0 */

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**I2C1 GPIO Configuration
		 PB6     ------> I2C1_SCL
		 PB7     ------> I2C1_SDA
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* I2C1 clock enable */
		__HAL_RCC_I2C1_CLK_ENABLE();

		/* I2C1 DMA Init */
		/* I2C1_RX Init */
		hdma_i2c1_rx.Instance = DMA1_Stream0;
		hdma_i2c1_rx.Init.Channel = DMA_CHANNEL_1;
		hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
		hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
		hdma_i2c1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK) {
			Error_Handler();
		}

		__HAL_LINKDMA(i2cHandle, hdmarx, hdma_i2c1_rx);

		/* I2C1_TX Init */
		hdma_i2c1_tx.Instance = DMA1_Stream1;
		hdma_i2c1_tx.Init.Channel = DMA_CHANNEL_0;
		hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
		hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
		hdma_i2c1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK) {
			Error_Handler();
		}

		__HAL_LINKDMA(i2cHandle, hdmatx, hdma_i2c1_tx);

		/* I2C1 interrupt Init */
		HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
		/* USER CODE BEGIN I2C1_MspInit 1 */

		/* USER CODE END I2C1_MspInit 1 */
	}
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *i2cHandle) {

	if (i2cHandle->Instance == I2C1) {
		/* USER CODE BEGIN I2C1_MspDeInit 0 */

		/* USER CODE END I2C1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_I2C1_CLK_DISABLE();

		/**I2C1 GPIO Configuration
		 PB6     ------> I2C1_SCL
		 PB7     ------> I2C1_SDA
		 */
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

		/* I2C1 DMA DeInit */
		HAL_DMA_DeInit(i2cHandle->hdmarx);
		HAL_DMA_DeInit(i2cHandle->hdmatx);

		/* I2C1 interrupt Deinit */
		HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
		/* USER CODE BEGIN I2C1_MspDeInit 1 */

		/* USER CODE END I2C1_MspDeInit 1 */
	}
}

/* USER CODE BEGIN 1 */
void I2C_safeInit(void) {

	i2cRdMutexId = osMutexCreate(osMutex(i2cRdMutex));
	osMutexRelease(i2cRdMutexId);

	i2cWrMutexId = osMutexCreate(osMutex(i2cWrMutex));
	osMutexRelease(i2cWrMutexId);

	i2cRdMemMutexId = osMutexCreate(osMutex(i2cRdMemMutex));
	osMutexRelease(i2cRdMemMutexId);

	i2cWrMemMutexId = osMutexCreate(osMutex(i2cWrMemMutex));
	osMutexRelease(i2cWrMemMutexId);

	i2cWrMemByteMutexId = osMutexCreate(osMutex(i2cWrMemByteMutex));
	osMutexRelease(i2cWrMemByteMutexId);

}

//#define USE_DMA

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {

}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {

}

HAL_StatusTypeDef I2C_safeMemWriteByte(I2C_HandleTypeDef *hi2c,
		uint16_t DevAddress, uint16_t MemAddress, uint8_t Data) {

	osMutexWait(i2cWrMemMutexId, osWaitForever);

	HAL_StatusTypeDef err = HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, 1,
			&Data, 1, 1000);

	osMutexRelease(i2cWrMemMutexId);

	return err;
}

HAL_StatusTypeDef I2C_safeMemWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
		uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) {

	osMutexWait(i2cWrMemMutexId, osWaitForever);
#ifdef USE_DMA
	HAL_StatusTypeDef err = HAL_I2C_Mem_Write_DMA(hi2c, DevAddress, MemAddress,
			MemAddSize, pData, Size);
#else
	HAL_StatusTypeDef err = HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress,
			MemAddSize, pData, Size, 1000);
#endif

	osMutexRelease(i2cWrMemMutexId);

	return err;
}

HAL_StatusTypeDef I2C_safeMemRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
		uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) {

	osMutexWait(i2cRdMemMutexId, osWaitForever);
#ifdef USE_DMA
	HAL_StatusTypeDef err = HAL_I2C_Mem_Read_DMA(hi2c, DevAddress, MemAddress,
			MemAddSize, pData, Size);
#else
	HAL_StatusTypeDef err = HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress,
			MemAddSize, pData, Size, 1000);
#endif
	osMutexRelease(i2cRdMemMutexId);

	return err;
}

HAL_StatusTypeDef I2C_safeRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
		uint8_t *pData, uint16_t Size) {

	osMutexWait(i2cRdMutexId, osWaitForever);
	HAL_StatusTypeDef err = HAL_I2C_Master_Receive(hi2c, DevAddress, pData,
			Size, 1000);
	osMutexRelease(i2cRdMutexId);

	return err;
}

HAL_StatusTypeDef I2C_safeWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
		uint8_t *pData, uint16_t Size) {

	osMutexWait(i2cWrMutexId, osWaitForever);
	HAL_StatusTypeDef err = HAL_I2C_Master_Transmit(hi2c, DevAddress, pData,
			Size, 1000);
	osMutexRelease(i2cWrMutexId);

	return err;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
