/*
 * i2c.h
 *
 *  Created on: 1 Mar 2022
 *      Author: adityasehgal
 */

#ifndef I2C_H_
#define I2C_H_

#include "os.h"

err_t i2c_read(uint32_t devAddr, uint8_t *buf, uint32_t size);

err_t i2c_write(uint32_t devAddr, uint8_t *buf, uint32_t size);

err_t i2c_readReg(uint32_t devAddr, uint8_t reg, uint8_t *buf, uint32_t size);

err_t i2c_writeReg(uint32_t devAddr, uint8_t reg, uint8_t *buf, uint32_t size);

err_t i2c_readWrite(uint32_t devAddr, uint8_t *wBuf, uint32_t wSize,
		uint8_t *rBuf, uint32_t rSize);

void i2c_init(void);

#endif /* I2C_H_ */
