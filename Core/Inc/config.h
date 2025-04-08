/*
 * config.h
 *
 *  Created on: Mar 28, 2025
 *      Author: user
 *      Description: Конфигурация прошивки
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "stm32g0xx_hal.h"
// Общие настройки
#define MAX_DISKS 24
#define DEBOUNCE_TIME 50    // 50 мс
#define SHORT_PRESS 2000    // 2 сек
#define LONG_PRESS 5000     // 5 сек

#define TEMP_REGISTER 0x00 //регистр температуры MAX7500

// Адреса I2C устройств
#define MAX7500_ADDR_1 0x94 // адрес первого температурного датчика
#define MAX7500_ADDR_2 0x92 // адрес второго температурного датчика
#define LED_adr 0x75 // адрес расширителя индикации
#define Dev_SLP_adr 0x77 //адрес расширителя наличия дисков+DevSLP
#define I2C_EXPAND_adr 0x74 // адрес i2C переключателя с индикацией и prstn+devslp
#define I2C_adapter_adr 0x73 // адрес I2c переключателя с микроконтроллерами адаптера
#define I2C_STM_FAN_adapter 0x25 // адрес микроконтроллера fan на адаптере
#define I2C_STM_SGPIO_adapter 0x24 // адрес микроконтроллера sgpio на адаптере



typedef enum {
	BUTTON_IDLE,        // Кнопка отпущена
	BUTTON_DEBOUNCE,    // Дребезг
	BUTTON_PRESSED,     // Нажата (ожидание действия)
	BUTTON_HOLD         // Удержание
} ButtonState;

typedef struct {
	GPIO_TypeDef* port;
	uint16_t pin;
	ButtonState state;
	uint32_t press_time;
	uint8_t adapter_num;
} Button;

typedef enum AdapterState { // adapter commands
    PWR_OFF = 0b00,
    PWR_ON = 0b01,
    HARD_RESET = 0b11
} AdapterState;

typedef struct { //disks structure
    uint8_t isConnected;    // Флаг подключения
    uint8_t activity;       // Активность
    uint8_t error;          // Ошибка
    uint8_t locate;         // Локация
} DiskStatus;


typedef enum{
	LED_OFF,
	LED_BLINKING,
	LED_ON
}LEDMode;

typedef struct {
	GPIO_TypeDef *GPIOx;       // Порт светодиода
	uint16_t GPIO_Pin;         // Пин светодиода
	uint8_t adapter_number;    // Номер адаптера (1 или 2)
	uint32_t blink_start_time; // Время начала мигания
	uint8_t is_blinking;       // Флаг мигания
} LEDState;

 extern LEDState leds[2];
 extern Button buttons[2];
extern ButtonState button1_state;
extern ButtonState button2_state;

extern uint8_t adapter1_state; //состояние адаптеров
extern uint8_t adapter2_state;

extern uint8_t BP_ON; // состояние БП

extern uint8_t I2CInit_off[2]; //Буфер отключения каналов расширителя I2C
extern uint8_t I2CInit_0[2]; //Буфер выбора нулевого канала расширителя I2C
extern uint8_t I2CInit_1[2] ; //Буфер выбора первого канала расширителя I2C
extern uint8_t I2CInit_2[2] ; //Буфер выбора второго канала расширителя I2C
extern uint8_t I2CInit_3[2] ; //Буфер выбора третьего канала расширителя I2C
extern uint8_t I2CInit_2_3[2]; //Буфер выбора второго и третьего канала расширителя I2C

extern uint8_t ledbufON[3]; // Все 1 на выходе расширителя
extern uint8_t ledbufOFF[3]; // Все 0 на выходе расширителя

extern uint8_t i2cbuff_OUT[3]; // Буфер инициализации каналов расширителя на output
extern uint8_t i2cbuff_IN[3]; // Буфер инициализации каналов расширителя на input (10101010,10101010)
extern uint8_t Dev_SLP_ON[3];

extern uint8_t Buf_PRSTN[2];
extern uint32_t connectActivityTimer[MAX_DISKS] ; // Таймеры для сброса активности при подключении
extern uint8_t MB1_attach;// переменные для управления адаптерами
extern uint8_t MB2_attach;

extern int8_t temperature;
extern uint8_t btn_handler;
extern uint8_t is_launching;

extern I2C_HandleTypeDef hi2c2; // Для HAL
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern uint8_t flag;
extern uint8_t cold_start;
#endif /* INC_CONFIG_H_ */
