/*
 * adapter.h
 *
 *  Created on: Mar 28, 2025
 *      Author: user
 */

#ifndef INC_ADAPTER_H_
#define INC_ADAPTER_H_
#include "stm32g0xx_hal.h"
#include "config.h"



void Adapter_Init(void); //Init Adapter
void PowerOnAdapter(uint8_t adapter_number); // отправка посылки на включение адаптера по I2C
void PowerOffAdapter(uint8_t adapter_number); // отправка посылки на выключение адаптера по I2C
void HardResetAdapter(uint8_t adapter_number);// отправка посылки на hard reset  адаптера по I2C
void UpdateCPU_PSON(void); // отслеживание и обновление состояния блока питания



#endif /* INC_ADAPTER_H_ */
