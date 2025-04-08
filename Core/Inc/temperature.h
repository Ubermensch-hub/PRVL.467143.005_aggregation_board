/*
 * temperature.h
 *
 *  Created on: Mar 28, 2025
 *      Author: user
 */

#ifndef INC_TEMPERATURE_H_
#define INC_TEMPERATURE_H_
#include "stm32g0xx_hal.h"

int8_t readTemperature(uint8_t address);
int8_t getMaxTemperature();
void TransmitTemperature();

#endif /* INC_TEMPERATURE_H_ */
