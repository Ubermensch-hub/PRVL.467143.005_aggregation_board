/*
 * temperature.c
 *
 *  Created on: Mar 28, 2025
 *      Author: user
 */
#include "config.h"
#include "main.h"
#include "i2c.h"
#include "temperature.h"
// Функция для чтения температуры с датчика (возвращает температуру в десятых долях градуса)
int8_t readTemperature(uint8_t address) {
	uint8_t data[2];
	// Чтение двух байтов из регистра температуры
	i2c_read(address, TEMP_REGISTER, data, 2);

	// Объединение двух байтов в 16-битное значение
	int16_t tempData = (data[0] << 8) | data[1];

	// �?звлечение знакового бита
	uint8_t isNegative = (tempData & 0x8000) != 0;

	// �?звлечение целой части температуры
	int8_t integerPart = (tempData >> 8) & 0x7F;

	// Учет отрицательной температуры
	if (isNegative) {
		integerPart = -integerPart;
	}

	return integerPart;
}
// Функция для получения максимальной температуры
int8_t getMaxTemperature() {
	int8_t temp1 = readTemperature(MAX7500_ADDR_1);
	int8_t temp2 = readTemperature(MAX7500_ADDR_2);

	return (temp1 > temp2) ? temp1 : temp2;
}

void TransmitTemperature()
{

	// Преобразуем int8_t в uint8_t для передачи
	uint8_t data = (uint8_t)temperature;

	if (MB1_attach == 0 && adapter1_state == 1) {
		HAL_GPIO_WritePin(MB1_STATUS_LED_GPIO_Port, MB1_STATUS_LED_Pin, RESET);
		HAL_Delay(50);
		// Выбираем канал 2 мультиплексора
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_2, 1, HAL_MAX_DELAY);
		HAL_Delay(50);

		// Передаем данные (1 байт)
		HAL_I2C_Master_Transmit(&hi2c2, (0x25 << 1), &data, 1, HAL_MAX_DELAY);
	}

	if (MB2_attach == 0 && adapter2_state == 1) {
		HAL_GPIO_WritePin(MB2_STATUS_LED_GPIO_Port, MB2_STATUS_LED_Pin, RESET);
		HAL_Delay(50);
		// Выбираем канал 3 мультиплексора
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_3, 1, HAL_MAX_DELAY);
		HAL_Delay(50);

		// Передаем данные (1 байт)
		HAL_I2C_Master_Transmit(&hi2c2, (0x25 << 1), &data, 1, HAL_MAX_DELAY);
	}

}
