/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FP_MB2_PWRLED_Pin GPIO_PIN_11
#define FP_MB2_PWRLED_GPIO_Port GPIOC
#define SGPIO_I2C2_RES_G_Pin GPIO_PIN_13
#define SGPIO_I2C2_RES_G_GPIO_Port GPIOC
#define SGPIO_I2C2_RES_Pin GPIO_PIN_14
#define SGPIO_I2C2_RES_GPIO_Port GPIOC
#define D_DRIVE4_ACTIVITY_Pin GPIO_PIN_15
#define D_DRIVE4_ACTIVITY_GPIO_Port GPIOC
#define FP_MB1_PWRLED_Pin GPIO_PIN_0
#define FP_MB1_PWRLED_GPIO_Port GPIOC
#define B_DRIVE4_ACTIVITY_Pin GPIO_PIN_1
#define B_DRIVE4_ACTIVITY_GPIO_Port GPIOC
#define B_DRIVE3_ACTIVITY_Pin GPIO_PIN_2
#define B_DRIVE3_ACTIVITY_GPIO_Port GPIOC
#define B_DRIVE2_ACTIVITY_Pin GPIO_PIN_3
#define B_DRIVE2_ACTIVITY_GPIO_Port GPIOC
#define MB1_BITCH_Pin GPIO_PIN_3
#define MB1_BITCH_GPIO_Port GPIOA
#define MB1_STATUS_LED_Pin GPIO_PIN_4
#define MB1_STATUS_LED_GPIO_Port GPIOA
#define MB2_BITCH_Pin GPIO_PIN_5
#define MB2_BITCH_GPIO_Port GPIOA
#define MB2_STATUS_LED_Pin GPIO_PIN_6
#define MB2_STATUS_LED_GPIO_Port GPIOA
#define CPU_PWROK_Pin GPIO_PIN_7
#define CPU_PWROK_GPIO_Port GPIOA
#define CPU_PSON_Pin GPIO_PIN_4
#define CPU_PSON_GPIO_Port GPIOC
#define MB1_PWR_SW_Pin GPIO_PIN_5
#define MB1_PWR_SW_GPIO_Port GPIOC
#define MB1_ATTACH_Pin GPIO_PIN_0
#define MB1_ATTACH_GPIO_Port GPIOB
#define SGPIO_I2C3_RES_Pin GPIO_PIN_1
#define SGPIO_I2C3_RES_GPIO_Port GPIOB
#define SGPIO_I2C3RES_G_Pin GPIO_PIN_2
#define SGPIO_I2C3RES_G_GPIO_Port GPIOB
#define TEMP_I2C1_SCL_Pin GPIO_PIN_10
#define TEMP_I2C1_SCL_GPIO_Port GPIOB
#define TEMP_I2C1_SDA_Pin GPIO_PIN_11
#define TEMP_I2C1_SDA_GPIO_Port GPIOB
#define A_DRIVE1_ACTIVITY_Pin GPIO_PIN_13
#define A_DRIVE1_ACTIVITY_GPIO_Port GPIOB
#define A_DRIVE2_ACTIVITY_Pin GPIO_PIN_14
#define A_DRIVE2_ACTIVITY_GPIO_Port GPIOB
#define A_DRIVE3_ACTIVITY_Pin GPIO_PIN_15
#define A_DRIVE3_ACTIVITY_GPIO_Port GPIOB
#define A_DRIVE4_ACTIVITY_Pin GPIO_PIN_8
#define A_DRIVE4_ACTIVITY_GPIO_Port GPIOA
#define B_DRIVE1_ACTIVITY_Pin GPIO_PIN_9
#define B_DRIVE1_ACTIVITY_GPIO_Port GPIOA
#define C_DRIVE1_ACTIVITY_Pin GPIO_PIN_6
#define C_DRIVE1_ACTIVITY_GPIO_Port GPIOC
#define C_DRIVE2_ACTIVITY_Pin GPIO_PIN_7
#define C_DRIVE2_ACTIVITY_GPIO_Port GPIOC
#define C_DRIVE3_ACTIVITY_Pin GPIO_PIN_8
#define C_DRIVE3_ACTIVITY_GPIO_Port GPIOD
#define C_DRIVE4_ACTIVITY_Pin GPIO_PIN_9
#define C_DRIVE4_ACTIVITY_GPIO_Port GPIOD
#define D_DRIVE1_ACTIVITY_Pin GPIO_PIN_10
#define D_DRIVE1_ACTIVITY_GPIO_Port GPIOA
#define D_DRIVE2_ACTIVITY_Pin GPIO_PIN_11
#define D_DRIVE2_ACTIVITY_GPIO_Port GPIOA
#define D_DRIVE3_ACTIVITY_Pin GPIO_PIN_12
#define D_DRIVE3_ACTIVITY_GPIO_Port GPIOA
#define FP_MB2_PWR_SW_Pin GPIO_PIN_15
#define FP_MB2_PWR_SW_GPIO_Port GPIOA
#define FP_MB1_PWR_SW_Pin GPIO_PIN_8
#define FP_MB1_PWR_SW_GPIO_Port GPIOC
#define SGPIO_I2C1_RES_Pin GPIO_PIN_9
#define SGPIO_I2C1_RES_GPIO_Port GPIOC
#define SGPIO_I2C1_RES_G_Pin GPIO_PIN_0
#define SGPIO_I2C1_RES_G_GPIO_Port GPIOD
#define TEMP_I2C2_RES_Pin GPIO_PIN_1
#define TEMP_I2C2_RES_GPIO_Port GPIOD
#define TEMP_I2C1_RES_Pin GPIO_PIN_2
#define TEMP_I2C1_RES_GPIO_Port GPIOD
#define MB2_ATTACH_Pin GPIO_PIN_5
#define MB2_ATTACH_GPIO_Port GPIOD
#define MB2_PWR_SW_Pin GPIO_PIN_6
#define MB2_PWR_SW_GPIO_Port GPIOD
#define F_DRIVE4_ACTIVITY_Pin GPIO_PIN_3
#define F_DRIVE4_ACTIVITY_GPIO_Port GPIOB
#define F_DRIVE3_ACTIVITY_Pin GPIO_PIN_4
#define F_DRIVE3_ACTIVITY_GPIO_Port GPIOB
#define F_DRIVE2_ACTIVITY_Pin GPIO_PIN_5
#define F_DRIVE2_ACTIVITY_GPIO_Port GPIOB
#define F_DRIVE1_ACTIVITY_Pin GPIO_PIN_6
#define F_DRIVE1_ACTIVITY_GPIO_Port GPIOB
#define E_DRIVE4_ACTIVITY_Pin GPIO_PIN_7
#define E_DRIVE4_ACTIVITY_GPIO_Port GPIOB
#define E_DRIVE3_ACTIVITY_Pin GPIO_PIN_8
#define E_DRIVE3_ACTIVITY_GPIO_Port GPIOB
#define E_DRIVE2_ACTIVITY_Pin GPIO_PIN_9
#define E_DRIVE2_ACTIVITY_GPIO_Port GPIOB
#define E_DRIVE1_ACTIVITY_Pin GPIO_PIN_10
#define E_DRIVE1_ACTIVITY_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
