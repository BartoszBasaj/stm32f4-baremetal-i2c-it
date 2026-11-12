/*
 * i2c.h
 *
 *  Created on: Nov 11, 2026
 *      Author: bhbas
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>

// Synchronization flags shared with upper layers
extern volatile uint8_t data_ready;
extern volatile uint8_t data_sent;
extern volatile uint8_t i2c_error;

// Public non-blocking I2C1 API
void I2C_Init(void);
void I2C1_Read_IT(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t len);
void I2C1_Write_IT(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t len);

#endif /* I2C_H_ */