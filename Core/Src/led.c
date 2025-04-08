/*
 * led.c
 *
 *  Created on: Mar 28, 2025
 *      Author: user
 */

#include "led.h"
#include "i2c.h"
#include "adapter.h"
#include "main.h"
#include "disk.h"

// Буферы для LED
static uint8_t channel_one[3] = {0x02, 0xFF, 0xFF};
static uint8_t channel_two[3] = {0x02, 0xFF, 0xFF};
static uint8_t channel_three[3] = {0x02, 0xFF, 0xFF};




void Set_Led_On(void)
{
	while (HAL_I2C_IsDeviceReady(&hi2c2, LED_adr << 1, 3, 100)!= HAL_OK){
	}
	HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), i2cbuff_OUT, 3, 10);
	HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), ledbufON, 3, 10);
}

void Set_Led_Off()
{
	while (HAL_I2C_IsDeviceReady(&hi2c2, LED_adr << 1, 3, 100)!= HAL_OK){
	}
	HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), i2cbuff_OUT, 3, 10);
	HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), ledbufOFF, 3, 10);

}

void LED_Init(void) {
	// Инициализация LED
	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_0, 1, 10);
	Set_Led_On();
	HAL_Delay(400);
	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_1, 1, 10);
	Set_Led_On();
	HAL_Delay(400);
	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_2, 1, 10);
	Set_Led_On();
	HAL_Delay(400);
	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_0, 1, 10);
	Set_Led_Off();
	HAL_Delay(400);
	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_1, 1, 10);
	Set_Led_Off();
	HAL_Delay(400);
	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_2, 1, 10);
	Set_Led_Off();
	HAL_Delay(400);
}

void Set_Led()
{
	while (HAL_I2C_IsDeviceReady(&hi2c2, I2C_EXPAND_adr << 1, 3, 100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_0, 1, 100);
	HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), i2cbuff_OUT, 3, 100);
	HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), channel_one, 3, 100);
	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_1, 1, 100);
	HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), i2cbuff_OUT, 3, 100);
	HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), channel_two, 3, 100);
	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_2, 1, 100);
	HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), i2cbuff_OUT, 3, 100);
	HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), channel_three, 3, 100);
}

void UpdateLEDStates()
{
	static uint8_t blinkState = 0; // Состояние мигания (0 или 1)
	blinkState = !blinkState;     // �?нвертируем состояние каждые 125 мс (4 Гц)

	// Очищаем буферы каналов


	// Обновляем состояние светодиодов для каждого диска
	for (int i = 0; i < MAX_DISKS; ++i) {

		// Определяем, какой канал использовать
		uint8_t *channel;
		uint8_t greenBit;
		uint8_t redBit;

		if (i < 8) {
			channel = channel_one;
			if (i < 4) {
				// Диски 0-3: первый байт
				greenBit = (i == 0) ? 6 : (i == 1) ? 4 : (i == 2) ? 2 : 1;
				redBit = (i == 0) ? 7 : (i == 1) ? 5 : (i == 2) ? 3 : 0;
			} else {
				// Диски 4-7: второй байт
				greenBit = (i == 4) ? 7 : (i == 5) ? 4 : (i == 6) ? 3 : 1;
				redBit = (i == 4) ? 6 : (i == 5) ? 5 : (i == 6) ? 2 : 0;
			}
		} else if (i < 16) {
			channel = channel_two;
			if (i < 12) {
				// Диски 8-11: первый байт
				greenBit = (i == 8) ? 6 : (i == 9) ? 5 : (i == 10) ? 2 : 1;
				redBit = (i == 8) ? 7 : (i == 9) ? 4 : (i == 10) ? 3 : 0;
			} else {
				// Диски 12-15: второй байт
				greenBit = (i == 12) ? 6 : (i == 13) ? 4 : (i == 14) ? 2 : 1;
				redBit = (i == 12) ? 7 : (i == 13) ? 5 : (i == 14) ? 3 : 0;
			}
		} else {
			channel = channel_three;
			if (i < 20) {
				// Диски 16-19: первый байт
				greenBit = (i == 16) ? 7 : (i == 17) ? 5 : (i == 18) ? 3 : 1;
				redBit = (i == 16) ? 6 : (i == 17) ? 4 : (i == 18) ? 2 : 0;
			} else {
				// Диски 20-23: второй байт
				greenBit = (i == 20) ? 6 : (i == 21) ? 4 : (i == 22) ? 3 : 1;
				redBit = (i == 20) ? 7 : (i == 21) ? 5 : (i == 22) ? 2 : 0;
			}
		}
		uint8_t byteIndex = (i < 4 || (i >= 8 && i < 12) || (i >= 16 && i < 20)) ? 2 : 1;

		// Управление светодиодами
		if (disks[i].isConnected) {
			if (disks[i].error) {
				// Ошибка: красный светодиод горит постоянно
				channel[byteIndex] |= (1 << (greenBit % 8));  // Выключаем зеленый светодиод
				channel[byteIndex] &= ~(1 << (redBit % 8)); // Включаем красный светодиод
			} else if (disks[i].locate) {
				// Локация: зеленый и красный светодиоды мигают
				if (blinkState) {
					channel[byteIndex] &= ~(1 << (greenBit % 8)); // Включаем зеленый светодиод
					channel[byteIndex] &= ~(1 << (redBit % 8)); // Включаем красный светодиод
				} else {
					channel[byteIndex] |= (1 << (greenBit % 8));  // Выключаем зеленый светодиод
					channel[byteIndex] |= (1 << (redBit % 8));  // Выключаем красный светодиод
				}
			} else if (disks[i].activity) {
				// Активность: зеленый светодиод мигает
				if (blinkState) {
					channel[byteIndex] &= ~(1 << (greenBit % 8)); // Включаем зеленый светодиод
				} else {
					channel[byteIndex] |= (1 << (greenBit % 8));  // Выключаем зеленый светодиод
				}
			} else if (disks[i].error == 0 && disks[i].locate == 0 && disks[i].activity == 0)
			{
				channel[byteIndex] &= ~ (1 << (greenBit % 8));  // Включаем зеленый светодиод
				channel[byteIndex] |= (1 << (redBit % 8));  // Выключаем красный светодиод
			}
		} else
		{
			channel[byteIndex] |= (1 << (greenBit % 8));  // Выключаем зеленый светодиод
			channel[byteIndex] |= (1 << (redBit % 8));  // Выключаем красный светодиод
		}
	}

	// Обновляем светодиоды на расширителе
	Set_Led();
}

void UpdateLEDs() {
	uint32_t current_time = HAL_GetTick();

	for (int i = 0; i < 2; i++) {
		uint8_t adapter_state = (i == 0) ? adapter1_state : adapter2_state;
		uint8_t adapter_attach = (i == 0) ? MB1_attach : MB2_attach;
		LEDState* led = &leds[i];

		if (adapter_attach) {
			// Адаптер не подключен - выключаем светодиод
			HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_RESET);
			led->is_blinking = 0;
		}
		else if (led->is_blinking) {
			// Мигание в процессе
			if (current_time - led->blink_start_time < 5000) { // Мигаем 5 секунд
				// Состояние мигания управляется в прерывании таймера
			}
			else {
				// Завершаем мигание
				led->is_blinking = 0;
				if (adapter_state == 1) {
					HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_SET); // Включаем
				} else {
					HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_RESET); // Выключаем
				}
			}
		}
		else {
			// Обычный режим
			HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin,
					adapter_state == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
		}
	}
}
void SetLED(uint8_t adapter_number, uint8_t state) {

	if (adapter_number < 1 || adapter_number > 2) return;

	LEDState *led = &leds[adapter_number - 1];
	led->is_blinking = 0;  // Выключаем мигание при явном установлении состояния
	HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);


}
// Функция для запуска мигания светодиода
void StartBlinking(uint8_t adapter_num) {
	LEDState *led = &leds[adapter_num - 1];
	led->is_blinking = 1;
	led->blink_start_time = HAL_GetTick();
	HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_SET); // Начинаем с включенного состояния

}


// Функция для выключения светодиода
void StopBlinking(uint8_t adapter_number) {
	if (adapter_number < 1 || adapter_number > 2) return;

	LEDState *led = &leds[adapter_number - 1];
	led->is_blinking = 0;  // Останавливаем мигание
	HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_RESET);

}
