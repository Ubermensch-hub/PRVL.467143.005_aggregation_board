/*
 * adapter.c
 *
 *  Created on: Mar 28, 2025
 *      Author: user
 */


#include "adapter.h"
#include "i2c.h"
#include "led.h"
#include "main.h" // Для доступа к пинам
#include "config.h"
#include "disk.h"
// Переменные модуля


void Adapter_Init(void) {
	adapter1_state = 0;
	adapter2_state = 0;
}

void PowerOnAdapter(uint8_t adapter_number)
{
	HAL_GPIO_WritePin(CPU_PSON_GPIO_Port, CPU_PSON_Pin, SET);
	ResetBus();
	Set_devslp();
	is_launching = 1;
	uint8_t i2c_buffer[1]; // Буфер для передачи данных по I2C
	StartBlinking(adapter_number);
	if (adapter_number == 1 && adapter1_state != 1 && MB1_attach ==  0) {
		HAL_Delay(300);
		LED_Init();
		// Подготовка команды для передачи по I2C
		i2c_buffer[0] = PWR_ON; // �?спользуем значение из enum
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_off, 1, 100);
		HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_2, 1, 100);
		HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_STM_FAN_adapter << 1), i2c_buffer, 1, 100); // Передаём буфер
		HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_off, 1, 100);
		UpdateLEDs();
		if(HAL_GPIO_ReadPin(MB1_BITCH_GPIO_Port, MB1_BITCH_Pin)!= 1){ adapter1_state = 1;
		}else adapter1_state = 0;


	} else if (adapter_number == 2 && adapter2_state != 1 && MB2_attach ==  0 ) {
		HAL_Delay(300);
		LED_Init();
		// Подготовка команды для передачи по I2C
		i2c_buffer[0] = PWR_ON; // �?спользуем значение из enum
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_off, 1, 1000);
		HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_3, 1, 500);
		HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_STM_FAN_adapter << 1), i2c_buffer, 1, 1000); // Передаём буфер
		HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_off, 1, 1000);
		UpdateLEDs();
		if(HAL_GPIO_ReadPin(MB2_BITCH_GPIO_Port, MB2_BITCH_Pin)!= 1){ adapter1_state = 1;
		}else adapter1_state = 0;

	}

	cold_start = 0;
	is_launching = 0;
}
void PowerOffAdapter(uint8_t adapter_number) // отправка посылки на выключение адаптера по I2C
{

	uint8_t i2c_buffer[1]; // Буфер для передачи данных по I2C

	is_launching = 0;
	if (adapter_number == 1 && adapter1_state == 1) {
		// Подготовка команды для передачи по I2C
		i2c_buffer[0] = PWR_OFF; // �?спользуем значение из enum
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_off, 1, 1000);
		HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_2, 1, 1000);
		HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_STM_FAN_adapter << 1), i2c_buffer, 1, 1000); // Передаём буфер
		HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_off, 1, 1000);

		if(HAL_GPIO_ReadPin(MB1_BITCH_GPIO_Port, MB1_BITCH_Pin)!= 1) {
			adapter1_state = 1;
		} else adapter1_state = 0;
	} else if (adapter_number == 2 && adapter2_state == 1) {
		// Подготовка команды для передачи по I2C
		i2c_buffer[0] = PWR_OFF; // �?спользуем значение из enum
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_off, 1, 1000);
		HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_3, 1, 1000);
		HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_STM_FAN_adapter << 1), i2c_buffer, 1, 1000); // Передаём буфер
		HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_off, 1, 1000);
		if(HAL_GPIO_ReadPin(MB2_BITCH_GPIO_Port, MB2_BITCH_Pin)!= 1) {
			adapter2_state = 1;
		} else adapter2_state = 0;
	}
}

void HardResetAdapter(uint8_t adapter_number)// отправка посылки на hard reset  адаптера по I2C
{

	uint8_t i2c_buffer[1]; // Буфер для передачи данных по I2C

	if (adapter_number == 1 && adapter1_state == 1) {
		// Подготовка команды для передачи по I2C
		i2c_buffer[0] = HARD_RESET; // �?спользуем значение из enum
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_off, 1, 1000);
		HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_2, 1, HAL_MAX_DELAY);
		HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_STM_FAN_adapter << 1), i2c_buffer, 1, 1000); // Передаём буфер
		HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_off, 1, 1000);
		HAL_Delay(6000);
	} else if (adapter_number == 2 && adapter2_state == 1) {
		// Подготовка команды для передачи по I2C
		i2c_buffer[0] = HARD_RESET; // �?спользуем значение из enum
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_off, 1, 1000);
		HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_3, 1, HAL_MAX_DELAY);
		HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_STM_FAN_adapter << 1), i2c_buffer, 1, 1000); // Передаём буфер
		HAL_Delay(10);
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_off, 1, 1000);
		HAL_Delay(6000);
	}

}

void UpdateCPU_PSON()
{
	if( MB1_attach == 0 && MB2_attach == 0)
	{
		if (adapter1_state == 0 && adapter2_state == 1 && MB2_attach == 0 && cold_start == 1)
		{
			HAL_GPIO_WritePin(CPU_PSON_GPIO_Port, CPU_PSON_Pin, SET);
			is_launching = 1;
			LED_Init();
			PowerOnAdapter(2);
			HAL_TIM_Base_Start_IT(&htim1);
			HAL_TIM_Base_Start_IT(&htim2);

			cold_start = 0;
		}else
			if (adapter2_state == 0 && adapter1_state == 1 && MB1_attach == 0 && cold_start == 1)
			{
				HAL_GPIO_WritePin(CPU_PSON_GPIO_Port, CPU_PSON_Pin, SET);
				is_launching = 1;
				LED_Init();
				PowerOnAdapter(1);
				HAL_TIM_Base_Start_IT(&htim1);
				HAL_TIM_Base_Start_IT(&htim2);
				cold_start = 0;
			}else if (adapter1_state == 1 && adapter2_state == 1 && cold_start == 1)
			{
				HAL_GPIO_WritePin(CPU_PSON_GPIO_Port, CPU_PSON_Pin, SET);
				is_launching = 1;
				LED_Init();
				PowerOnAdapter(1);
				PowerOnAdapter(2);
				HAL_Delay(200);
				HAL_TIM_Base_Start_IT(&htim1);
				HAL_TIM_Base_Start_IT(&htim2);
				cold_start = 0;
			}

	}else if ((adapter1_state == 1 && MB1_attach == 0) || (adapter2_state == 1 && MB2_attach == 0) )
	{

		HAL_GPIO_WritePin(CPU_PSON_GPIO_Port, CPU_PSON_Pin, SET);

		if (cold_start == 1)
		{
			HAL_Delay(500);
			LED_Init();
			HAL_Delay(500);
		}
		cold_start =0;
		HAL_TIM_Base_Start_IT(&htim1);
		HAL_TIM_Base_Start_IT(&htim2);

	}else if (is_launching == 1)
	{
		HAL_GPIO_WritePin(CPU_PSON_GPIO_Port, CPU_PSON_Pin, SET);
		HAL_Delay(500);
		LED_Init();
		HAL_TIM_Base_Start_IT(&htim1);
		HAL_TIM_Base_Start_IT(&htim2);
	}else
	{
		HAL_TIM_Base_Stop_IT(&htim1);
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_GPIO_WritePin(CPU_PSON_GPIO_Port, CPU_PSON_Pin, RESET);
	}
}

