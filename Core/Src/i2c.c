/*
 * i2c.c
 *
 *  Created on: Mar 28, 2025
 *      Author: user
 */
#include "i2c.h"
#include "config.h"
#include "main.h"

void ResetBus()
{

	HAL_GPIO_WritePin(TEMP_I2C1_RES_GPIO_Port, TEMP_I2C1_RES_Pin, RESET);
	HAL_GPIO_WritePin(SGPIO_I2C1_RES_G_GPIO_Port, SGPIO_I2C1_RES_G_Pin, RESET);
	HAL_GPIO_WritePin(SGPIO_I2C1_RES_GPIO_Port, SGPIO_I2C1_RES_Pin, RESET);
	HAL_GPIO_WritePin(SGPIO_I2C3RES_G_GPIO_Port, SGPIO_I2C3RES_G_Pin, RESET);
	HAL_GPIO_WritePin(SGPIO_I2C3_RES_GPIO_Port, SGPIO_I2C3_RES_Pin, RESET);
	HAL_GPIO_WritePin(SGPIO_I2C2_RES_GPIO_Port, SGPIO_I2C2_RES_Pin, RESET);
	HAL_GPIO_WritePin(SGPIO_I2C2_RES_G_GPIO_Port, SGPIO_I2C2_RES_G_Pin, RESET);
	HAL_GPIO_WritePin(TEMP_I2C2_RES_GPIO_Port, TEMP_I2C2_RES_Pin, RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(TEMP_I2C1_RES_GPIO_Port, TEMP_I2C1_RES_Pin, SET);
	HAL_GPIO_WritePin(SGPIO_I2C1_RES_G_GPIO_Port, SGPIO_I2C1_RES_G_Pin, SET);
	HAL_GPIO_WritePin(SGPIO_I2C1_RES_GPIO_Port, SGPIO_I2C1_RES_Pin, SET);
	HAL_GPIO_WritePin(SGPIO_I2C3RES_G_GPIO_Port, SGPIO_I2C3RES_G_Pin, SET);
	HAL_GPIO_WritePin(SGPIO_I2C3_RES_GPIO_Port, SGPIO_I2C3_RES_Pin, SET);
	HAL_GPIO_WritePin(SGPIO_I2C2_RES_GPIO_Port, SGPIO_I2C2_RES_Pin, SET);
	HAL_GPIO_WritePin(SGPIO_I2C2_RES_G_GPIO_Port, SGPIO_I2C2_RES_G_Pin, SET);
	HAL_GPIO_WritePin(TEMP_I2C2_RES_GPIO_Port, TEMP_I2C2_RES_Pin, SET);

}
void Reset_sgpio_fan_bus()
{
	HAL_GPIO_WritePin(TEMP_I2C2_RES_GPIO_Port, TEMP_I2C2_RES_Pin, RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(TEMP_I2C2_RES_GPIO_Port, TEMP_I2C2_RES_Pin, SET);
	HAL_Delay(100);
}
void Read_Register(uint8_t register_pointer, uint8_t* receive_buffer, uint16_t adr_rep)
{
	//set pointer to register

	HAL_I2C_Master_Transmit(&hi2c2, (adr_rep << 1), &register_pointer, 1, 1000);
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c2, (adr_rep << 1), receive_buffer, 1, 1000);


}

HAL_StatusTypeDef i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {
	// Чтение данных из регистра reg_addr устройства с адресом dev_addr
	return HAL_I2C_Mem_Read(&hi2c2, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}
