/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "adapter.h"
#include "led.h"
#include "disk.h"
#include "button.h"
#include "temperature.h"
#include "i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*typedef struct
{
	GPIO_TypeDef *activPort;  // Порт для пина ACTIV
	uint16_t activPin;        // Пин ACTIV
} DiskPins;*/
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
Button buttons[2] = {
		{GPIOC, FP_MB1_PWR_SW_Pin, BUTTON_IDLE, 0, 1}, // Кнопка 1
		{GPIOA, FP_MB2_PWR_SW_Pin, BUTTON_IDLE, 0, 2}  // Кнопка 2
};

LEDState leds[2] = {
		{FP_MB1_PWRLED_GPIO_Port, FP_MB1_PWRLED_Pin, 1, 0, 0},
		{FP_MB2_PWRLED_GPIO_Port, FP_MB2_PWRLED_Pin, 2, 0, 0}
};

int8_t temperature = 25;

uint8_t disk_status[6] = {0}; // Буфер для приёма данных

uint8_t Buf_PRSTN[2] = {0 , 0};

uint8_t i2cbuff_OUT[3] = {0x06, 0x00, 0x00}; // Буфер инициализации каналов расширителя на output
uint8_t i2cbuff_IN[3] = {0x06, 0xAA, 0xAA}; // Буфер инициализации каналов расширителя на input (10101010,10101010)

uint8_t I2CInit_off[2] = {0x0, 0x0}; //Буфер отключения каналов расширителя I2C
uint8_t I2CInit_0[2] = {0x1, 0x0}; //Буфер выбора нулевого канала расширителя I2C
uint8_t I2CInit_1[2] = {0x2, 0x0}; //Буфер выбора первого канала расширителя I2C
uint8_t I2CInit_2[2] = {0x4, 0x0}; //Буфер выбора второго канала расширителя I2C
uint8_t I2CInit_3[2] = {0x8, 0x0}; //Буфер выбора третьего канала расширителя I2C
uint8_t I2CInit_2_3[2] = {0xC, 0x0}; //Буфер выбора второго и третьего канала расширителя I2C

uint8_t ledbufON[3] = {0x02, 0x00, 0x00}; // Все 1 на выходе расширителя
uint8_t ledbufOFF[3] = {0x02, 0xFF, 0xFF}; // Все 0 на выходе расширителя

uint8_t Dev_SLP_ON[3] = {0x02, 0xAA, 0xAA}; // "10101010", "10101010" (dev_slp_position)

uint8_t channel_one[3] = {0x02, 0xFF, 0xFF};//буферы каналов sgpio
uint8_t channel_two[3] = {0x02, 0xFF, 0xFF};
uint8_t channel_three[3] = {0x02, 0xFF, 0xFF};

DiskStatus disks[MAX_DISKS] = {0}; // Массив для хранения статусов всех дисков
//DiskPins diskPins[MAX_DISKS]; // Массив для хранения пинов всех дисков

uint8_t MB1_attach = 1;// переменные для управления адаптерами
uint8_t MB2_attach = 1;

uint8_t ledbuf_AB[3] = {0x02, 0xFF, 0xFF}; // Все 1 на выходе расширителя
uint8_t ledbuf_CD[3] = {0x02, 0xFF, 0xFF}; // Все 2 на выходе расширителя
uint8_t ledbuf_EF[3] = {0x02, 0xFF, 0xFF}; // Все 3 на выходе расширителя

uint8_t adapter1_state = 0; // Состояние адаптера 1 (0 - выключен, 1 - включен)
uint8_t adapter2_state = 0; // Состояние адаптера 2 (0 - выключен, 1 - включен)

uint8_t BP_ON = 0;       //Статус блока питания
uint8_t state_fp1 = 0;
uint8_t state_fp2 = 0;
uint8_t hotswap_mb1 = 0;
uint8_t hotswap_mb2 = 0;

uint8_t MB_ON_1 = 0;
uint8_t MB_ON_2 = 0;

uint8_t btn_handler = 0;
uint8_t previousActivity[MAX_DISKS] = {0}; // Массив для хранения предыдущего состояния активности
uint32_t activityTimer[MAX_DISKS] = {0}; // Массив для хранения времени последнего изменения активности

uint32_t connectActivityTimer[MAX_DISKS] = {0}; // Таймеры для сброса активности при подключении
uint8_t cold_start = 1;
uint8_t is_launching = 0;
volatile uint8_t button1_pressed = 0; // Флаг нажатия кнопки 1
volatile uint8_t button2_pressed = 0; // Флаг нажатия кнопки 2
volatile uint32_t button1_press_time = 0; // Время нажатия кнопки 1
volatile uint32_t button2_press_time = 0; // Время нажатия кнопки 2
volatile uint32_t button1_debounce_time = 0; // Время последнего срабатывания прерывания
volatile uint32_t button2_debounce_time = 0; // Время последнего срабатывания прерывания
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
//void InitializeDiskPins();
void ProcessCondition()
{
	if (MB1_attach == 0) {
		if (HAL_GPIO_ReadPin(MB1_BITCH_GPIO_Port, MB1_BITCH_Pin) == 1)
		{
			adapter1_state = 0;}
		else adapter1_state = 1;
	}

	if (MB2_attach == 0) {
		if (HAL_GPIO_ReadPin(MB2_BITCH_GPIO_Port, MB2_BITCH_Pin) == 1)
		{
			adapter2_state = 0;
		}else adapter2_state = 1;
	}
	MB1_attach = HAL_GPIO_ReadPin(MB1_ATTACH_GPIO_Port, MB1_ATTACH_Pin);
	MB2_attach = HAL_GPIO_ReadPin(MB2_ATTACH_GPIO_Port, MB2_ATTACH_Pin);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t     update_led_flag = 0;

uint8_t     read_disk_status_flag = 0;

uint8_t     transmit_temperature_flag = 0;

uint8_t     read_disks_connected_flag = 0;

uint8_t flag = 0 ;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */


	ResetBus();
	HAL_Delay(500);
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
	ResetBus();
	//InitializeDiskPins();

	Initialize_Disks();

	Set_devslp();
	HAL_Delay(250);
	Read_disks_connected(); // Чтение подключенных дисков
	HAL_Delay(250);
	HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HandleButtons();//buttons
		ProcessCondition();
		UpdateLEDs();





	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* I2C2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(I2C2_IRQn);
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00F12163;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 124;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FP_MB2_PWRLED_Pin|FP_MB1_PWRLED_Pin|CPU_PSON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SGPIO_I2C2_RES_G_Pin|SGPIO_I2C2_RES_Pin|SGPIO_I2C1_RES_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MB1_STATUS_LED_Pin|MB2_STATUS_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SGPIO_I2C3_RES_Pin|SGPIO_I2C3RES_G_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SGPIO_I2C1_RES_G_Pin|TEMP_I2C2_RES_Pin|TEMP_I2C1_RES_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : FP_MB2_PWRLED_Pin SGPIO_I2C2_RES_G_Pin SGPIO_I2C2_RES_Pin FP_MB1_PWRLED_Pin
                           CPU_PSON_Pin SGPIO_I2C1_RES_Pin */
  GPIO_InitStruct.Pin = FP_MB2_PWRLED_Pin|SGPIO_I2C2_RES_G_Pin|SGPIO_I2C2_RES_Pin|FP_MB1_PWRLED_Pin
                          |CPU_PSON_Pin|SGPIO_I2C1_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : D_DRIVE4_ACTIVITY_Pin B_DRIVE4_ACTIVITY_Pin B_DRIVE3_ACTIVITY_Pin B_DRIVE2_ACTIVITY_Pin
                           MB1_PWR_SW_Pin C_DRIVE1_ACTIVITY_Pin C_DRIVE2_ACTIVITY_Pin E_DRIVE1_ACTIVITY_Pin */
  GPIO_InitStruct.Pin = D_DRIVE4_ACTIVITY_Pin|B_DRIVE4_ACTIVITY_Pin|B_DRIVE3_ACTIVITY_Pin|B_DRIVE2_ACTIVITY_Pin
                          |MB1_PWR_SW_Pin|C_DRIVE1_ACTIVITY_Pin|C_DRIVE2_ACTIVITY_Pin|E_DRIVE1_ACTIVITY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MB1_BITCH_Pin MB2_BITCH_Pin CPU_PWROK_Pin A_DRIVE4_ACTIVITY_Pin
                           B_DRIVE1_ACTIVITY_Pin D_DRIVE1_ACTIVITY_Pin D_DRIVE2_ACTIVITY_Pin D_DRIVE3_ACTIVITY_Pin */
  GPIO_InitStruct.Pin = MB1_BITCH_Pin|MB2_BITCH_Pin|CPU_PWROK_Pin|A_DRIVE4_ACTIVITY_Pin
                          |B_DRIVE1_ACTIVITY_Pin|D_DRIVE1_ACTIVITY_Pin|D_DRIVE2_ACTIVITY_Pin|D_DRIVE3_ACTIVITY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MB1_STATUS_LED_Pin MB2_STATUS_LED_Pin */
  GPIO_InitStruct.Pin = MB1_STATUS_LED_Pin|MB2_STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MB1_ATTACH_Pin A_DRIVE1_ACTIVITY_Pin A_DRIVE2_ACTIVITY_Pin A_DRIVE3_ACTIVITY_Pin
                           F_DRIVE4_ACTIVITY_Pin F_DRIVE3_ACTIVITY_Pin F_DRIVE2_ACTIVITY_Pin F_DRIVE1_ACTIVITY_Pin
                           E_DRIVE4_ACTIVITY_Pin E_DRIVE3_ACTIVITY_Pin E_DRIVE2_ACTIVITY_Pin */
  GPIO_InitStruct.Pin = MB1_ATTACH_Pin|A_DRIVE1_ACTIVITY_Pin|A_DRIVE2_ACTIVITY_Pin|A_DRIVE3_ACTIVITY_Pin
                          |F_DRIVE4_ACTIVITY_Pin|F_DRIVE3_ACTIVITY_Pin|F_DRIVE2_ACTIVITY_Pin|F_DRIVE1_ACTIVITY_Pin
                          |E_DRIVE4_ACTIVITY_Pin|E_DRIVE3_ACTIVITY_Pin|E_DRIVE2_ACTIVITY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SGPIO_I2C3_RES_Pin SGPIO_I2C3RES_G_Pin */
  GPIO_InitStruct.Pin = SGPIO_I2C3_RES_Pin|SGPIO_I2C3RES_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : C_DRIVE3_ACTIVITY_Pin C_DRIVE4_ACTIVITY_Pin MB2_ATTACH_Pin MB2_PWR_SW_Pin */
  GPIO_InitStruct.Pin = C_DRIVE3_ACTIVITY_Pin|C_DRIVE4_ACTIVITY_Pin|MB2_ATTACH_Pin|MB2_PWR_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : FP_MB2_PWR_SW_Pin */
  GPIO_InitStruct.Pin = FP_MB2_PWR_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FP_MB2_PWR_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FP_MB1_PWR_SW_Pin */
  GPIO_InitStruct.Pin = FP_MB1_PWR_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FP_MB1_PWR_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SGPIO_I2C1_RES_G_Pin TEMP_I2C2_RES_Pin TEMP_I2C1_RES_Pin */
  GPIO_InitStruct.Pin = SGPIO_I2C1_RES_G_Pin|TEMP_I2C2_RES_Pin|TEMP_I2C1_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1 && btn_handler == 0) // 8 раз в секунду
	{
		HandleButtons();//buttons
		UpdateLEDStates(); // Обновление состояния светодиодов
		static uint8_t blink_phase = 0;
		blink_phase = !blink_phase; // �?нвертируем состояние
		for (int i = 0; i < 2; i++) {
			if (leds[i].is_blinking) {
				HAL_GPIO_WritePin(leds[i].GPIOx, leds[i].GPIO_Pin,
						blink_phase ? GPIO_PIN_SET : GPIO_PIN_RESET);
			}
		}
	} else
		if (htim->Instance == TIM2 && btn_handler == 0) // 4 раза в секунду
		{
			read_disk_status_flag = 1;

			Read_Disk_Status(0x24, disk_status, 6); // Чтение статуса дисков
			Decode_Disk_Status(disk_status); // Декодирование статуса
			Read_disks_connected(); // Чтение подключенных дисков

		} else
			if (htim->Instance == TIM3 && btn_handler == 0) // 1 раз в секунду
			{
				UpdateCPU_PSON();
				is_launching = 0;
				//temperature = getMaxTemperature();
				for (int i = 0; i < MAX_DISKS; ++i) {
					if (disks[i].activity == 1 && connectActivityTimer[i] != 0 &&
							(HAL_GetTick() - connectActivityTimer[i] >= 4000)) { // 4 секунды
						disks[i].activity = 0; // Сбрасываем активность
						connectActivityTimer[i] = 0; // Сбрасываем таймер
					}
				}
				transmit_temperature_flag = 1; // Устанавливаем флаг для передачи температуры

			}
}

/*void InitializeDiskPins()

{
	// Диск 0
	diskPins[0].activPort = F_DRIVE1_ACTIVITY_GPIO_Port;
	diskPins[0].activPin = F_DRIVE1_ACTIVITY_Pin;
	// Диск 1
	diskPins[1].activPort = F_DRIVE2_ACTIVITY_GPIO_Port;
	diskPins[1].activPin = F_DRIVE2_ACTIVITY_Pin;
	// Диск 2
	diskPins[2].activPort = F_DRIVE3_ACTIVITY_GPIO_Port;
	diskPins[2].activPin = F_DRIVE3_ACTIVITY_Pin;
	// �? так далее для остальных дисков...
	diskPins[3].activPort = F_DRIVE4_ACTIVITY_GPIO_Port;
	diskPins[3].activPin = F_DRIVE4_ACTIVITY_Pin;
	diskPins[4].activPort = E_DRIVE1_ACTIVITY_GPIO_Port;
	diskPins[4].activPin = E_DRIVE1_ACTIVITY_Pin;
	diskPins[5].activPort = E_DRIVE2_ACTIVITY_GPIO_Port;
	diskPins[5].activPin = E_DRIVE2_ACTIVITY_Pin;
	diskPins[6].activPort = E_DRIVE3_ACTIVITY_GPIO_Port;
	diskPins[6].activPin = E_DRIVE3_ACTIVITY_Pin;
	diskPins[7].activPort = E_DRIVE4_ACTIVITY_GPIO_Port;
	diskPins[7].activPin = E_DRIVE4_ACTIVITY_Pin;
	diskPins[8].activPort = D_DRIVE1_ACTIVITY_GPIO_Port;
	diskPins[8].activPin = D_DRIVE1_ACTIVITY_Pin;
	diskPins[9].activPort = D_DRIVE2_ACTIVITY_GPIO_Port;
	diskPins[9].activPin = D_DRIVE2_ACTIVITY_Pin;
	diskPins[10].activPort = D_DRIVE3_ACTIVITY_GPIO_Port;
	diskPins[10].activPin = D_DRIVE3_ACTIVITY_Pin;
	diskPins[11].activPort = D_DRIVE4_ACTIVITY_GPIO_Port;
	diskPins[11].activPin = D_DRIVE4_ACTIVITY_Pin;
	diskPins[12].activPort = C_DRIVE1_ACTIVITY_GPIO_Port;
	diskPins[12].activPin = C_DRIVE1_ACTIVITY_Pin;
	diskPins[13].activPort = C_DRIVE2_ACTIVITY_GPIO_Port;
	diskPins[13].activPin = C_DRIVE2_ACTIVITY_Pin;
	diskPins[14].activPort = C_DRIVE3_ACTIVITY_GPIO_Port;
	diskPins[14].activPin = C_DRIVE3_ACTIVITY_Pin;
	diskPins[15].activPort = C_DRIVE4_ACTIVITY_GPIO_Port;
	diskPins[15].activPin = C_DRIVE4_ACTIVITY_Pin;
	diskPins[16].activPort = B_DRIVE1_ACTIVITY_GPIO_Port;
	diskPins[16].activPin = B_DRIVE1_ACTIVITY_Pin;
	diskPins[17].activPort = B_DRIVE2_ACTIVITY_GPIO_Port;
	diskPins[17].activPin = B_DRIVE2_ACTIVITY_Pin;
	diskPins[18].activPort = B_DRIVE3_ACTIVITY_GPIO_Port;
	diskPins[18].activPin = B_DRIVE3_ACTIVITY_Pin;
	diskPins[19].activPort = B_DRIVE4_ACTIVITY_GPIO_Port;
	diskPins[19].activPin = B_DRIVE4_ACTIVITY_Pin;
	diskPins[20].activPort = A_DRIVE1_ACTIVITY_GPIO_Port;
	diskPins[20].activPin = A_DRIVE1_ACTIVITY_Pin;
	diskPins[21].activPort = A_DRIVE2_ACTIVITY_GPIO_Port;
	diskPins[21].activPin = A_DRIVE2_ACTIVITY_Pin;
	diskPins[22].activPort = A_DRIVE3_ACTIVITY_GPIO_Port;
	diskPins[22].activPin = A_DRIVE3_ACTIVITY_Pin;
	diskPins[23].activPort = A_DRIVE3_ACTIVITY_GPIO_Port;
	diskPins[23].activPin = A_DRIVE3_ACTIVITY_Pin;

}*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();

	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
