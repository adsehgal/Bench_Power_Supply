/*
 * i2c.c
 *
 *  Created on: 1 Mar 2022
 *      Author: adityasehgal
 */

#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_i2c.h"

#include "i2c.h"

#define I2C_BASE I2C1

#define I2C_SCL_PIN		LL_GPIO_PIN_6
#define I2C_SDA_PIN		LL_GPIO_PIN_7
#define I2C_PORT	GPIOB

#define CHECK_BUFF_SIZE(x) \
	if(x >= 255) {	x = 255; }

#define USE_TIMEOUT 1

#if USE_TIMEOUT
// define timeout for tx/rx of each byte. Do not increase more than 100
#define TIMEOUT_MS 100
#else
#define SEND_TIMEOUT_MS 0xFFFFFFFF
#define RECV_TIMEOUT_MS 0xFFFFFFFF
#endif

#if USE_TIMEOUT
#define TIMEOUT_DECLARE(timeout) uint32_t timeoutCount = timeout
#define TIMEOUT_RESET(timeout) timeoutCount = timeout
#define TIMEOUT_OCCURRED()	((--timeoutCount == 0) ? 1 : 0)
#else
#define TIMEOUT_DECLARE(timeout) do{}while(0)
#define TIMEOUT_RESET(timeout) do{}while(0)
#define TIMEOUT_DECREMENT(counter) do{}while(0)
#define TIMEOUT_OCCURRED()	0	// dont let it time out
#endif

err_t i2c_read(uint32_t devAddr, uint8_t *buf, uint32_t size) {
	err_t err = osOK;

	if (!size) {
		err = osErrorValue;
	} else {
		devAddr = (devAddr << 1) | 1;
		TIMEOUT_DECLARE(TIMEOUT_MS);
		// Enable ACK
		LL_I2C_AcknowledgeNextData(I2C_BASE, LL_I2C_ACK);

		// start comms
		LL_I2C_GenerateStartCondition(I2C_BASE);
		while (!LL_I2C_IsActiveFlag_SB(I2C_BASE))
			;	// wait until start bit is set

		// index the device
		LL_I2C_TransmitData8(I2C_BASE, (uint8_t) devAddr);
		while (!LL_I2C_IsActiveFlag_ADDR(I2C_BASE))
			;
		LL_I2C_ClearFlag_ADDR(I2C_BASE);
		LL_I2C_ClearFlag_AF(I2C_BASE);

		for (uint32_t i = 0; i < size;) {
			if (LL_I2C_IsActiveFlag_RXNE(I2C_BASE)) {
				buf[i] = LL_I2C_ReceiveData8(I2C_BASE);
				i++;
				TIMEOUT_RESET(TIMEOUT_MS);
			}
			if (LL_SYSTICK_IsActiveCounterFlag()) {
				if (TIMEOUT_OCCURRED()) {
					LL_I2C_GenerateStopCondition(I2C_BASE);
					err = osErrorTimeoutResource;
					break;
				}
			}
		}
		LL_I2C_GenerateStopCondition(I2C_BASE);
		LL_I2C_ClearFlag_STOP(I2C_BASE);
	}

	return err;
}

err_t i2c_write(uint32_t devAddr, uint8_t *buf, uint32_t size) {
	err_t err = osOK;

	if (!size) {
		err = osErrorValue;
	} else {
		devAddr = (devAddr << 1) | 1;
		TIMEOUT_DECLARE(TIMEOUT_MS);
		// Enable ACK
		LL_I2C_AcknowledgeNextData(I2C_BASE, LL_I2C_ACK);

		// start comms
		LL_I2C_GenerateStartCondition(I2C_BASE);
		while (!LL_I2C_IsActiveFlag_SB(I2C_BASE))
			;	// wait until start bit is set

		// index the device
		LL_I2C_TransmitData8(I2C_BASE, (uint8_t) devAddr);
		while (!LL_I2C_IsActiveFlag_ADDR(I2C_BASE))
			;
		LL_I2C_ClearFlag_ADDR(I2C_BASE);

		for (uint32_t i = 0; i < size;) {
			if (LL_I2C_IsActiveFlag_TXE(I2C_BASE)) {
				LL_I2C_TransmitData8(I2C_BASE, buf[i]);
				while (!LL_I2C_IsActiveFlag_BTF(I2C_BASE))
					;
				i++;
				TIMEOUT_RESET(TIMEOUT_MS);
			}
			if (LL_SYSTICK_IsActiveCounterFlag()) {
				if (TIMEOUT_OCCURRED()) {
					LL_I2C_GenerateStopCondition(I2C_BASE);
					err = osErrorTimeoutResource;
					break;
				}
			}
		}
		LL_I2C_GenerateStopCondition(I2C_BASE);
		LL_I2C_ClearFlag_STOP(I2C_BASE);
	}

	return err;
}

err_t i2c_readReg(uint32_t devAddr, uint8_t reg, uint8_t *buf, uint32_t size) {
	err_t err = osOK;

	return err;
}

err_t i2c_writeReg(uint32_t devAddr, uint8_t reg, uint8_t *buf, uint32_t size) {
	err_t err = osOK;

	return err;
}

err_t i2c_readWrite(uint32_t devAddr, uint8_t *wBuf, uint32_t wSize,
		uint8_t *rBuf, uint32_t rSize) {
	err_t err = osOK;

	return err;
}

void i2c_init(void) {
	LL_I2C_InitTypeDef i2c = { 0 };
	LL_GPIO_InitTypeDef gpio = { 0 };

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

	gpio.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
	gpio.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	gpio.Pull = LL_GPIO_PULL_NO;
	gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.Alternate = LL_GPIO_AF_4;
	LL_GPIO_Init(I2C_PORT, &gpio);

	LL_I2C_DisableOwnAddress2(I2C_BASE);
	LL_I2C_DisableGeneralCall(I2C_BASE);
	LL_I2C_EnableClockStretching(I2C_BASE);

	i2c.PeripheralMode = LL_I2C_MODE_I2C;
	i2c.ClockSpeed = 400000;
	i2c.DutyCycle = LL_I2C_DUTYCYCLE_2;
	i2c.TypeAcknowledge = LL_I2C_ACK;
	i2c.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
	i2c.OwnAddress1 = 0;
	i2c.DigitalFilter = 0;
	LL_I2C_Init(I2C_BASE, &i2c);
	LL_I2C_Enable(I2C_BASE);
}
