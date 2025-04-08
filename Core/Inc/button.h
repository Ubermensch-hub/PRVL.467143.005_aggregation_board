/*
 * button.h
 *
 *  Created on: Mar 28, 2025
 *      Author: user
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_


void HandleButtons();
void ProcessButton(Button* btn);
void ButtonAction(uint8_t adapter_num, uint8_t action_type);

#endif /* INC_BUTTON_H_ */
