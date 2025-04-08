/*
 * led.h
 *
 *  Created on: Mar 28, 2025
 *      Author: user
 */

#ifndef INC_LED_H_
#define INC_LED_H_
#include "stm32g0xx_hal.h"
#include "config.h"

void LED_Init(void);
void UpdateLEDs();
void UpdateLEDStates();
void SetLED(uint8_t adapter_number, uint8_t state);
void StartBlinking(uint8_t adapter_number);
void StopBlinking(uint8_t adapter_number);


#endif /* INC_LED_H_ */
