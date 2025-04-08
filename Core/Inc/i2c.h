/*
 * i2c.h
 *
 *  Created on: Mar 28, 2025
 *      Author: user
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "stm32g0xx_hal.h"

void ResetBus();
void Reset_sgpio_fan_bus();
void Read_Register(uint8_t register_pointer, uint8_t* receive_buffer, uint16_t adr_rep);
HAL_StatusTypeDef i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);

#endif /* INC_I2C_H_ */
