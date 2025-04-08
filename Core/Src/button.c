/*
 * button.c
 *
 *  Created on: Mar 28, 2025
 *      Author: user
 */
#include "config.h"
#include "main.h"
#include "adapter.h"
#include "button.h"

void HandleButtons() {
	for (int i = 0; i < 2; i++) {
		ProcessButton(&buttons[i]);
	}
}

void ProcessButton(Button* btn) {
    uint8_t pin_state = HAL_GPIO_ReadPin(btn->port, btn->pin);
    uint32_t current_time = HAL_GetTick();

    switch (btn->state) {
        case BUTTON_IDLE:
            if (pin_state == GPIO_PIN_RESET) {  // Кнопка нажата (активный 0)
                btn->state = BUTTON_DEBOUNCE;
                btn->press_time = current_time;
            }
            break;

        case BUTTON_DEBOUNCE:
            if (current_time - btn->press_time >= DEBOUNCE_TIME) {
                if (pin_state == GPIO_PIN_RESET) {  // Кнопка всё ещё нажата
                    btn->state = BUTTON_PRESSED;
                } else {  // Кнопка отпущена во время дребезга
                    btn->state = BUTTON_IDLE;
                }
            }
            break;

        case BUTTON_PRESSED:
        	btn_handler = 1;
            // Проверяем, не истекло ли время LONG_PRESS
            if (current_time - btn->press_time >= LONG_PRESS) {
                ButtonAction(btn->adapter_num, 2);  // Долгое нажатие (>5 сек)
                btn->state = BUTTON_HOLD;
            }
            // Если кнопка отпущена ДО LONG_PRESS — обрабатываем короткое/среднее нажатие
            else if (pin_state == GPIO_PIN_SET) {
                if (current_time - btn->press_time < SHORT_PRESS) {
                    ButtonAction(btn->adapter_num, 0);  // Короткое нажатие (<2 сек)
                }
                else if (current_time - btn->press_time < LONG_PRESS) {
                    ButtonAction(btn->adapter_num, 1);  // Среднее нажатие (2-5 сек)
                }
                btn->state = BUTTON_IDLE;
            }
            break;

        case BUTTON_HOLD:
            // Ждём, пока кнопку отпустят
            if (pin_state == GPIO_PIN_SET) {
                btn->state = BUTTON_IDLE;
            }
            break;
	default:
		break;
	}
}

void ButtonAction(uint8_t adapter_num, uint8_t action_type) {
	HAL_GPIO_WritePin(MB1_STATUS_LED_GPIO_Port, MB1_STATUS_LED_Pin, SET);
	HAL_GPIO_WritePin(MB2_STATUS_LED_GPIO_Port, MB2_STATUS_LED_Pin, SET);
	HAL_Delay(50);
	is_launching = 1;
	switch (action_type) {
	case 0: // Короткое нажатие (0-2 сек)
		PowerOnAdapter(adapter_num);
		break;
	case 1: // Среднее нажатие (2-5 сек)
		HardResetAdapter(adapter_num);
		break;
	case 2: // Долгое нажатие (>5 сек)
		PowerOffAdapter(adapter_num);
		break;
	default: break;
	}

	btn_handler = 0;

}

