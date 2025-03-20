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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint8_t isConnected;    // Флаг подключения (1 - подключен, 0 - не подключен)
	uint8_t activity;       // Активность (1 - активен, 0 - не активен)
	uint8_t error;          // Ошибка (1 - ошибка, 0 - нет ошибки)
	uint8_t locate;         // Локация (1 - включена, 0 - выключена)
} DiskStatus;

typedef struct
{
	GPIO_TypeDef *activPort;  // Порт для пина ACTIV
	uint16_t activPin;        // Пин ACTIV
} DiskPins;


typedef enum {
	BUTTON_STATE_PRESSED,
	BUTTON_STATE_RELEASED,
} ButtonState;

ButtonState button1_state = BUTTON_STATE_RELEASED;
ButtonState button2_state = BUTTON_STATE_RELEASED;

typedef enum AdapterState
{
	PWR_OFF = 0b00,
	PWR_ON = 0b01,
	HARD_RESET = 0b11
}AdapterState;

typedef struct {
	GPIO_TypeDef *GPIOx;       // Порт светодиода
	uint16_t GPIO_Pin;         // Пин светодиода
	uint32_t blink_start_time; // Время начала мигания
	uint32_t blink_duration;   // Длительность мигания
	uint32_t blink_period;     // Период мигания (в мс)
	uint8_t is_blinking;       // Флаг мигания
} LEDState;

LEDState led1 = {FP_MB1_PWRLED_GPIO_Port, FP_MB1_PWRLED_Pin, 0, 0, 0, 0};
LEDState led2 = {FP_MB2_PWRLED_GPIO_Port, FP_MB2_PWRLED_Pin, 0, 0, 0, 0};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_DELAY 75 // Задержка для обработки дребезга (в мс)
#define TEMP_REGISTER 0x00
#define MAX7500_ADDR_1 0x94
#define MAX7500_ADDR_2 0x92
#define MAX_DISKS 24
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


//uint8_t flag_read = 0;
uint8_t flag_error = 0;
uint8_t flag_update = 0;
uint8_t TempRxBuf = 0;//Буферы датчиков температуры
uint8_t TempTxBuf = 0;
int8_t temperature = 25;
uint8_t disk_status[6] = {0}; // Буфер для приёма данных

uint8_t Buf_PRSTN[2] = {0 , 0};

uint8_t i2cbuff[3] = {0x06, 0x00, 0x00}; // Буфер инициализации каналов расширителя на output
uint8_t i2cbuff_IN[3] = {0x06, 0xAA, 0xAA}; // Буфер инициализации каналов расширителя на input (10101010,10101010)

uint8_t I2CInit_off[2] = {0x0, 0x0}; //Буфер отключения каналов расширителя I2C
uint8_t I2CInit_0[2] = {0x1, 0x0}; //Буфер выбора нулевого канала расширителя I2C
uint8_t I2CInit_1[2] = {0x2, 0x0}; //Буфер выбора первого канала расширителя I2C
uint8_t I2CInit_2[2] = {0x4, 0x0}; //Буфер выбора второго канала расширителя I2C
uint8_t I2CInit_3[2] = {0x8, 0x0}; //Буфер выбора третьего канала расширителя I2C
uint8_t I2CInit_2_3[2] = {0xC, 0x0}; //Буфер выбора второго и третьего канала расширителя I2C

uint8_t ledbufON[3] = {0x02, 0x00, 0x00}; // Все 1 на выходе расширителя
uint8_t ledbufOFF[3] = {0x02, 0xFF, 0xFF}; // Все 0 на выходе расширителя

uint8_t LED_adr = 0x75; // адрес расширителей отвечающих за индикацию
uint8_t Dev_SLP_adr = 0x77; // адрес расширителя отвечающего за devslp и prstn
uint8_t I2C_EXPAND_adr = 0x74; // адрес свитча с каналами sgpio_i2c (индикация, prsnt, devslp)
uint8_t I2C_adapter_adr = 0x73; // адрес свитча с каналами передачи данных на адаптер

uint8_t Dev_SLP_ON[3] = {0x02, 0xAA, 0xAA}; // "10101010", "10101010" (dev_slp_position)
const uint8_t prstn_masks[4] = {0x02, 0x08, 0x20, 0x80};

uint8_t channel_one[3] = {0x02, 0xFF, 0xFF};//буферы каналов sgpio
uint8_t channel_two[3] = {0x02, 0xFF, 0xFF};
uint8_t channel_three[3] = {0x02, 0xFF, 0xFF};

DiskStatus disks[MAX_DISKS] = {0}; // Массив для хранения статусов всех дисков
DiskPins diskPins[MAX_DISKS]; // Массив для хранения пинов всех дисков

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

AdapterState adapter1_command = PWR_OFF;
AdapterState adapter2_command = PWR_OFF;

uint8_t sgpio_started = 0; // Начало передачи sgpio
uint8_t Counter_sgpio_timeout = 0; // счётчик sgpio обнраружения sgpio
uint8_t sgpio_timeout = 0; // sgpio не обнаружен по времени
uint8_t previousActivity[MAX_DISKS] = {0}; // Массив для хранения предыдущего состояния активности
uint32_t activityTimer[MAX_DISKS] = {0}; // Массив для хранения времени последнего изменения активности



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
void Set_devslp(); // функция должна выставить на все регистры DevSLP = 0
void Read_Register(uint8_t register_pointer, uint8_t* receive_buffer, uint16_t adr_rep);
void Initialize_Disks();
void Update_Disk_Status(uint8_t diskIndex, uint8_t activity, uint8_t error, uint8_t locate);
void Process_SGPIO_Data(uint16_t sgpioData, uint8_t startIndex);
void Set_Led();
void Set_Led_On();
void Set_Led_Off();
int8_t readTemperature(uint8_t address);
int8_t getMaxTemperature();
void UpdateDriveStatus(uint8_t drive_index, uint8_t prstn_bit, uint8_t buf_value);
void UpdateLEDStates();
void ResetBus();
void Led_Init();
void InitializeDiskPins();

void PowerOnAdapter(uint8_t adapter_number);
void PowerOffAdapter(uint8_t adapter_number);
void UpdateCPU_PSON();
void RebootAdapter(uint8_t adapter_number);
void ProcessPins(uint8_t diskIndex);
void Read_disks_connected();
void HardResetAdapter(uint8_t adapter_number);
void TransmitTemperature();
HAL_StatusTypeDef i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {
	// Чтение данных из регистра reg_addr устройства с адресом dev_addr
	return HAL_I2C_Mem_Read(&hi2c2, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

void UpdateLED(LEDState *led)
{
	if (led->is_blinking) {
		uint32_t current_time = HAL_GetTick();
		uint32_t elapsed_time = current_time - led->blink_start_time;

		if (elapsed_time < led->blink_duration) {
			// Мигание
			if (elapsed_time % led->blink_period < led->blink_period / 2) {
				HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, SET);
			} else {
				HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, RESET);
			}
		} else {
			// Завершение мигания
			HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, RESET);
			led->is_blinking = 0;
		}
	} else if ((adapter1_state == 1 && MB1_attach == 0) || (adapter2_state == 1 && MB2_attach == 0))
	{
		if(adapter1_state == 1 && led->GPIO_Pin == FP_MB1_PWRLED_Pin && MB1_attach == 0) HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, SET);
		if(adapter2_state == 1 && led->GPIO_Pin == FP_MB2_PWRLED_Pin && MB2_attach == 0) HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, SET);
	}
	if (MB1_attach == 1 && led->GPIO_Pin == FP_MB1_PWRLED_Pin){
		HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, RESET);
	}else if (adapter1_state == 0 && led->GPIO_Pin == FP_MB1_PWRLED_Pin)
	{
		HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, RESET);
	}
	if (MB2_attach == 1 && led->GPIO_Pin == FP_MB2_PWRLED_Pin){
		HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, RESET);
	}else if (adapter2_state == 0 && led->GPIO_Pin == FP_MB2_PWRLED_Pin)
	{
		HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, RESET);
	}


}

void StartBlinking(LEDState *led, uint32_t frequency, uint32_t duration)
{
	led->blink_period = 1000 / frequency; // Период мигания в мс
	led->blink_duration = duration;       // Длительность мигания
	led->blink_start_time = HAL_GetTick(); // Время начала мигания
	led->is_blinking = 1;                 // Включить мигание
}

void HandleButtonAction(uint8_t button_number, uint32_t press_duration)
{
	HAL_GPIO_WritePin(MB1_STATUS_LED_GPIO_Port, MB1_STATUS_LED_Pin, SET);
	HAL_GPIO_WritePin(MB2_STATUS_LED_GPIO_Port, MB2_STATUS_LED_Pin, SET);
	if (button_number == 1) {
		if (press_duration <= 2000) {
			// Короткое нажатие (0-1 сек) - включение адаптера 1
			PowerOnAdapter(1);
		} else if (press_duration <= 5000) {
			// Долгое нажатие (3-6 сек) - жесткая перезагрузка адаптера 1
			HardResetAdapter(1);
			StartBlinking(&led1, 4, 5000); // Мигание 4 Гц, 5 сек
		} else if (press_duration > 5000) {
			// Очень долгое нажатие (6-10 сек) - выключение адаптера 1
			PowerOffAdapter(1);
		}
	} else if (button_number == 2) {
		if (press_duration <= 2000) {
			// Короткое нажатие (0-1 сек) - включение адаптера 1
			PowerOnAdapter(2);
		} else if (press_duration <= 5000) {
			// Долгое нажатие (3-6 сек) - жесткая перезагрузка адаптера 1
			HardResetAdapter(2);
			StartBlinking(&led2, 4, 5000); // Мигание 4 Гц, 5 сек
		} else if (press_duration > 5000) {
			// Очень долгое нажатие (6-10 сек) - выключение адаптера 1
			PowerOffAdapter(2);
		}
	}
	HAL_Delay(500);
	HAL_GPIO_WritePin(MB1_STATUS_LED_GPIO_Port, MB1_STATUS_LED_Pin, RESET);
	HAL_GPIO_WritePin(MB2_STATUS_LED_GPIO_Port, MB2_STATUS_LED_Pin, RESET);
	HAL_TIM_Base_Start_IT(&htim3);
}

void UpdateDiskStatus(uint8_t diskIndex, uint8_t activity, uint8_t error, uint8_t locate) //функция для обновления данных о дисках
{
	disks[diskIndex].activity = activity;
	disks[diskIndex].error = error;
	disks[diskIndex].locate = locate;
}

void Read_Disk_Status(uint16_t slave_address, uint8_t *data, uint16_t size) {
	HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_0, 1, HAL_MAX_DELAY);
	HAL_Delay(50);
	HAL_I2C_Master_Receive(&hi2c2, slave_address << 1, data, size, HAL_MAX_DELAY);
}

void Decode_Disk_Status(uint8_t *data) {
	for (uint8_t disk_id = 0; disk_id < 24; disk_id++) {
		uint8_t byte_index = disk_id / 4; // �?ндекс байта
		uint8_t bit_offset = (disk_id % 4) * 2; // Смещение в байте
		uint8_t status = (data[byte_index] >> bit_offset) & 0x03; // �?звлечение статуса

		// Декодирование статуса
		switch (status) {
		case 0x00:
			UpdateDiskStatus(disk_id, 0, 0, 0);
			break;
		case 0x01:
			UpdateDiskStatus(disk_id, 1, 0, 0);
			break;
		case 0x02:
			UpdateDiskStatus(disk_id, 0, 1, 0);
			break;
		case 0x03:
			UpdateDiskStatus(disk_id, 0, 0, 1);
			break;
		default:
			break;
		}
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

	InitializeDiskPins();

	Initialize_Disks();
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


		// Обновление состояния светодиодов
		UpdateLED(&led1);
		UpdateLED(&led2);


		button1_state = HAL_GPIO_ReadPin(FP_MB1_PWR_SW_GPIO_Port, FP_MB1_PWR_SW_Pin);
		button2_state = HAL_GPIO_ReadPin(FP_MB2_PWR_SW_GPIO_Port, FP_MB2_PWR_SW_Pin);

		if (button1_pressed && button1_state == BUTTON_STATE_RELEASED) {
			HandleButtonAction(1, HAL_GetTick() - button1_press_time);
			button1_pressed = 0;
		}
		if (button2_pressed && button2_state == BUTTON_STATE_RELEASED) {
			HandleButtonAction(2, HAL_GetTick() - button2_press_time);
			button2_pressed = 0;
		}

		temperature = getMaxTemperature();

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
		/*for (int i = 0; i < MAX_DISKS; ++i)
		{
			ProcessPins(i);  // Обновляем статус каждого диска
			if (disks[i].activity == 1 && (HAL_GetTick() - activityTimer[i] >= 5000) && sgpio_started != 1) {
				// Сбрасываем активность
				disks[i].activity = 0;
				Update_Disk_Status(i, disks[i].activity, disks[i].error, disks[i].locate);
			}
		}*/

		// Обновление состояния CPU_PSON
		//UpdateCPU_PSON();





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
  /* TIM1_BRK_UP_TRG_COM_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
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
  hi2c2.Init.Timing = 0x10801031;
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
  HAL_GPIO_WritePin(GPIOC, FP_MB1_PWRLED_Pin|FP_MB2_PWRLED_Pin|CPU_PSON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SGPIO_I2C2_RES_G_Pin|SGPIO_I2C2_RES_Pin|SGPIO_I2C1_RES_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MB1_STATUS_LED_Pin|MB2_STATUS_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SGPIO_I2C3_RES_Pin|SGPIO_I2C3RES_G_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SGPIO_I2C1_RES_G_Pin|TEMP_I2C2_RES_Pin|TEMP_I2C1_RES_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : FP_MB1_PWRLED_Pin SGPIO_I2C2_RES_G_Pin SGPIO_I2C2_RES_Pin FP_MB2_PWRLED_Pin
                           CPU_PSON_Pin SGPIO_I2C1_RES_Pin */
  GPIO_InitStruct.Pin = FP_MB1_PWRLED_Pin|SGPIO_I2C2_RES_G_Pin|SGPIO_I2C2_RES_Pin|FP_MB2_PWRLED_Pin
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

  /*Configure GPIO pin : FP_MB1_PWR_SW_Pin */
  GPIO_InitStruct.Pin = FP_MB1_PWR_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FP_MB1_PWR_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FP_MB2_PWR_SW_Pin */
  GPIO_InitStruct.Pin = FP_MB2_PWR_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FP_MB2_PWR_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SGPIO_I2C1_RES_G_Pin TEMP_I2C2_RES_Pin TEMP_I2C1_RES_Pin */
  GPIO_InitStruct.Pin = SGPIO_I2C1_RES_G_Pin|TEMP_I2C2_RES_Pin|TEMP_I2C1_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Set_devslp()
{
	while (HAL_I2C_IsDeviceReady(&hi2c2, I2C_EXPAND_adr << 1, 3, 100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_0, 1, 1); //DevSLP  0 канал (E/F)
	HAL_Delay(5);
	while (HAL_I2C_IsDeviceReady(&hi2c2, Dev_SLP_adr << 1, 3, 100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c2, (Dev_SLP_adr << 1), i2cbuff_IN, 3, 1); // init input
	HAL_Delay(5);
	while (HAL_I2C_IsDeviceReady(&hi2c2, Dev_SLP_adr << 1, 3, 100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c2, (Dev_SLP_adr << 1), Dev_SLP_ON, 3, 1); //write
	HAL_Delay(5);

	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_1, 1, 1); //DevSLP  1 канал (C/D)
	HAL_Delay(5);
	HAL_I2C_Master_Transmit(&hi2c2, (Dev_SLP_adr << 1), i2cbuff_IN, 3, 1);
	HAL_Delay(5);
	HAL_I2C_Master_Transmit(&hi2c2, (Dev_SLP_adr << 1), Dev_SLP_ON, 3, 1);
	HAL_Delay(5);

	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_2, 1, 1); //DevSLP  2 канал (A/B)
	HAL_Delay(5);
	HAL_I2C_Master_Transmit(&hi2c2, (Dev_SLP_adr << 1), i2cbuff_IN, 3, 1);
	HAL_Delay(5);
	HAL_I2C_Master_Transmit(&hi2c2, (Dev_SLP_adr << 1), Dev_SLP_ON, 3, 1);
	HAL_Delay(5);
}

void Initialize_Disks()
{
	for (int i = 0; i < MAX_DISKS; ++i) {
		disks[i].isConnected = 0;
		disks[i].activity = 0;
		disks[i].error = 0;
		disks[i].locate = 0;
	}
}

void Update_Disk_Status(uint8_t diskIndex, uint8_t activity, uint8_t error, uint8_t locate)
{
	disks[diskIndex].activity = activity;
	disks[diskIndex].error = error;
	disks[diskIndex].locate = locate;
}

void Process_SGPIO_Data(uint16_t sgpioData, uint8_t startIndex)
{

	for (int i = 0; i < 4; ++i)
	{
		uint8_t diskIndex = startIndex + i; // �?ндекс диска (0-3 для A, 4-7 для B...)
		uint8_t diskStatus = (sgpioData >> (3 * i)) & 0x07; // �?звлечение 3 бит для диска
		uint8_t activity = (diskStatus >> 0) & 0x01; // 1-й бит - активность
		uint8_t locate = (diskStatus >> 1) & 0x01;   // 2-й бит - локация
		uint8_t error = (diskStatus >> 2) & 0x01;    // 3-й бит - ошибка

		// Обновление статуса диска
		Update_Disk_Status(diskIndex, activity, error, locate);

	}
}

void ResetBus()
{
	flag_update = 10;
	HAL_GPIO_WritePin(TEMP_I2C1_RES_GPIO_Port, TEMP_I2C1_RES_Pin, RESET);
	HAL_GPIO_WritePin(SGPIO_I2C1_RES_G_GPIO_Port, SGPIO_I2C1_RES_G_Pin, RESET);
	HAL_GPIO_WritePin(SGPIO_I2C1_RES_GPIO_Port, SGPIO_I2C1_RES_Pin, RESET);
	HAL_GPIO_WritePin(SGPIO_I2C3RES_G_GPIO_Port, SGPIO_I2C3RES_G_Pin, RESET);
	HAL_GPIO_WritePin(SGPIO_I2C3_RES_GPIO_Port, SGPIO_I2C3_RES_Pin, RESET);
	HAL_GPIO_WritePin(SGPIO_I2C2_RES_GPIO_Port, SGPIO_I2C2_RES_Pin, RESET);
	HAL_GPIO_WritePin(SGPIO_I2C2_RES_G_GPIO_Port, SGPIO_I2C2_RES_G_Pin, RESET);
	HAL_GPIO_WritePin(TEMP_I2C2_RES_GPIO_Port, TEMP_I2C2_RES_Pin, RESET);

	flag_update = 11;
	HAL_GPIO_WritePin(TEMP_I2C1_RES_GPIO_Port, TEMP_I2C1_RES_Pin, SET);
	HAL_GPIO_WritePin(SGPIO_I2C1_RES_G_GPIO_Port, SGPIO_I2C1_RES_G_Pin, SET);
	HAL_GPIO_WritePin(SGPIO_I2C1_RES_GPIO_Port, SGPIO_I2C1_RES_Pin, SET);
	HAL_GPIO_WritePin(SGPIO_I2C3RES_G_GPIO_Port, SGPIO_I2C3RES_G_Pin, SET);
	HAL_GPIO_WritePin(SGPIO_I2C3_RES_GPIO_Port, SGPIO_I2C3_RES_Pin, SET);
	HAL_GPIO_WritePin(SGPIO_I2C2_RES_GPIO_Port, SGPIO_I2C2_RES_Pin, SET);
	HAL_GPIO_WritePin(SGPIO_I2C2_RES_G_GPIO_Port, SGPIO_I2C2_RES_G_Pin, SET);
	HAL_GPIO_WritePin(TEMP_I2C2_RES_GPIO_Port, TEMP_I2C2_RES_Pin, SET);

}

void Set_Led()
{
	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_0, 1, 10);
	HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), i2cbuff, 3, 10);
	HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), channel_one, 3, 10);
	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_1, 1, 10);
	HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), i2cbuff, 3, 10);
	HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), channel_two, 3, 10);
	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_2, 1, 10);
	HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), i2cbuff, 3, 10);
	HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), channel_three, 3, 10);
}

void Set_Led_On()
{
	flag_update = 5;

	if (HAL_I2C_IsDeviceReady(&hi2c2, LED_adr << 1, 3, 100) == HAL_OK) {
		HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), i2cbuff, 3, 10);
	} else {
		flag_update = 7;
	}


	HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), ledbufON, 3, 10);
}

void Set_Led_Off()
{

	HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), i2cbuff, 3, 10);

	HAL_I2C_Master_Transmit(&hi2c2, (LED_adr << 1), ledbufOFF, 3, 10);

}
void Led_Init()
{

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


void Read_disks_connected()
{

	ResetBus();
	HAL_Delay(10);
	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_0, 1, 10);
	Read_Register(0x01, Buf_PRSTN, Dev_SLP_adr);


	if (~Buf_PRSTN[0] & 0x02)		//PRSTN F1
	{
		disks[0].isConnected = 1;
	} else {
		disks[0].isConnected = 0;
	}

	if (~Buf_PRSTN[0] & 0x08)		//PRSTN F2
	{
		disks[1].isConnected = 1;
	} else {
		disks[1].isConnected = 0;
	}

	if (~Buf_PRSTN[0] & 0x20)		//PRSTN F3
	{
		disks[2].isConnected = 1;
	} else {
		disks[2].isConnected = 0;
	}
	if (~Buf_PRSTN[0] & 0x80)		//PRSTN F4
	{
		disks[3].isConnected = 1;
	} else {
		disks[3].isConnected = 0;
	}

	Read_Register(0x00, Buf_PRSTN, Dev_SLP_adr);

	if (~Buf_PRSTN[0] & 0x02)		//PRSTN E1
	{
		disks[4].isConnected = 1;
	} else {
		disks[4].isConnected = 0;
	}

	if (~Buf_PRSTN[0] & 0x08)		//PRSTN E2
	{
		disks[5].isConnected = 1;
	} else {
		disks[5].isConnected = 0;
	}

	if (~Buf_PRSTN[0] & 0x20)		//PRSTN E3
	{
		disks[6].isConnected = 1;
	} else {
		disks[6].isConnected = 0;
	}

	if (~Buf_PRSTN[0] & 0x80)		//PRSTN E4

	{
		disks[7].isConnected = 1;
	} else {
		disks[7].isConnected = 0;
	}

	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_1, 1, 10);
	Read_Register(0x01, Buf_PRSTN, Dev_SLP_adr);

	if (~Buf_PRSTN[0] & 0x02)		//PRSTN D1
	{
		disks[8].isConnected = 1;
	} else {
		disks[8].isConnected = 0;
	}
	if (~Buf_PRSTN[0] & 0x08)		//PRSTN D2
	{
		disks[9].isConnected = 1;
	} else {
		disks[9].isConnected = 0;
	}
	if (~Buf_PRSTN[0] & 0x20)		//PRSTN D3
	{
		disks[10].isConnected = 1;
	} else {
		disks[10].isConnected = 0;
	}
	if (~Buf_PRSTN[0] & 0x80)		//PRSTN D4
	{
		disks[11].isConnected = 1;
	} else {
		disks[11].isConnected = 0;
	}
	Read_Register(0x00, Buf_PRSTN, Dev_SLP_adr);

	if (~Buf_PRSTN[0] & 0x02)		//PRSTN C1
	{
		disks[12].isConnected = 1;
	} else {
		disks[12].isConnected = 0;
	}
	if (~Buf_PRSTN[0] & 0x08)		//PRSTN C2
	{
		disks[13].isConnected = 1;
	} else {
		disks[13].isConnected = 0;
	}
	if (~Buf_PRSTN[0] & 0x20)		//PRSTN C3
	{
		disks[14].isConnected = 1;
	} else {
		disks[14].isConnected = 0;
	}

	if (~Buf_PRSTN[0] & 0x80)		//PRSTN C4
	{
		disks[15].isConnected = 1;
	} else {
		disks[15].isConnected = 0;
	}

	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_2, 1, 10);
	Read_Register(0x01, Buf_PRSTN, Dev_SLP_adr);

	if (~Buf_PRSTN[0] & 0x02)		//PRSTN B1
	{
		disks[16].isConnected = 1;
	} else {
		disks[16].isConnected = 0;
	}

	if (~Buf_PRSTN[0] & 0x08)		//PRSTN B2
	{
		disks[17].isConnected = 1;
	} else {
		disks[17].isConnected = 0;
	}
	if (~Buf_PRSTN[0] & 0x20)		//PRSTN B3
	{
		disks[18].isConnected = 1;
	} else {
		disks[18].isConnected = 0;
	}
	if (~Buf_PRSTN[0] & 0x80)		//PRSTN B4
	{
		disks[19].isConnected = 1;
	} else {
		disks[19].isConnected = 0;
	}
	Read_Register(0x00, Buf_PRSTN, Dev_SLP_adr);

	if (~Buf_PRSTN[0] & 0x02)		//PRSTN A1
	{
		disks[20].isConnected = 1;
	} else {
		disks[20].isConnected = 0;
	}
	if (~Buf_PRSTN[0] & 0x08)		//PRSTN A2
	{
		disks[21].isConnected = 1;
	} else {
		disks[21].isConnected = 0;
	}
	if (~Buf_PRSTN[0] & 0x20)		//PRSTN A3
	{
		disks[22].isConnected = 1;
	} else {
		disks[22].isConnected = 0;
	}
	if (~Buf_PRSTN[0] & 0x80)		//PRSTN A4
	{
		disks[23].isConnected = 1;
	} else {
		disks[23].isConnected = 0;
	}
}

void Read_Register(uint8_t register_pointer, uint8_t* receive_buffer, uint16_t adr_rep)
{
	//set pointer to register

	HAL_I2C_Master_Transmit(&hi2c2, (adr_rep << 1), &register_pointer, 1, 10);
	//receive data to receive_buffer
	HAL_I2C_Master_Receive(&hi2c2, (adr_rep << 1), receive_buffer, 1, 10);

}

void UpdateLEDStates()
{
	static uint8_t blinkState = 0; // Состояние мигания (0 или 1)
	blinkState = !blinkState;     // �?нвертируем состояние каждые 125 мс (4 Гц)

	// Очищаем буферы каналов


	// Обновляем состояние светодиодов для каждого диска
	for (int i = 0; i < MAX_DISKS; ++i) {
		if (disks[i].isConnected) {
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
					greenBit = (i == 20) ? 6 : (i == 21) ? 4 : (i == 22) ? 2 : 1;
					redBit = (i == 20) ? 7 : (i == 21) ? 5 : (i == 22) ? 3 : 0;
				}
			}
			uint8_t byteIndex = (i < 4 || (i >= 8 && i < 12) || (i >= 16 && i < 20)) ? 2 : 1;
			// Управление светодиодами
			if (disks[i].error) {
				// Ошибка: красный светодиод горит постоянно
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
				channel[byteIndex] |= (1 << (greenBit % 8));  // Выключаем зеленый светодиод
				channel[byteIndex] |= (1 << (redBit % 8));  // Выключаем красный светодиод
			}
		}
	}

	// Обновляем светодиоды на расширителе
	Set_Led();
}

void InitializeDiskPins()

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

}



void PowerOnAdapter(uint8_t adapter_number)
{
	uint8_t i2c_buffer[1]; // Буфер для передачи данных по I2C

	if (adapter_number == 1 && adapter1_state != 1) {
		HAL_GPIO_WritePin(CPU_PSON_GPIO_Port, CPU_PSON_Pin, SET);
		HAL_GPIO_WritePin(MB1_STATUS_LED_GPIO_Port, MB1_STATUS_LED_Pin, SET);
		HAL_Delay(100);
		ResetBus();
		if(BP_ON == 0) Led_Init();
		HAL_Delay(300);
		// Подготовка команды для передачи по I2C
		i2c_buffer[0] = PWR_ON; // �?спользуем значение из enum
		HAL_Delay(500);
		while (HAL_I2C_IsDeviceReady(&hi2c2, (I2C_adapter_adr << 1), 10, HAL_MAX_DELAY) != HAL_OK)
			{
			}
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_2, 1, HAL_MAX_DELAY);
		HAL_Delay(50);
		while (HAL_I2C_IsDeviceReady(&hi2c2, (0x25 << 1), 10, HAL_MAX_DELAY) != HAL_OK)
			{
			}
		HAL_I2C_Master_Transmit(&hi2c2, (0x25 << 1), i2c_buffer, 1, HAL_MAX_DELAY); // Передаём буфер
		Set_devslp();
		HAL_TIM_Base_Start_IT(&htim2);
		HAL_TIM_Base_Start_IT(&htim3);
		Counter_sgpio_timeout = 0;
		StartBlinking(&led1, 2, 10000); // Мигание 2 Гц, 10 сек

		while (adapter1_state != 1 ){
			if(HAL_GPIO_ReadPin(MB1_BITCH_GPIO_Port, MB1_BITCH_Pin)!= 1){ adapter1_state = 1;
			}else adapter1_state = 0;

		}
	} else if (adapter_number == 2 && adapter2_state != 1 ) {
		HAL_GPIO_WritePin(CPU_PSON_GPIO_Port, CPU_PSON_Pin, SET);

		HAL_GPIO_WritePin(MB2_STATUS_LED_GPIO_Port, MB2_STATUS_LED_Pin, SET);
		HAL_Delay(100);
		ResetBus();
		if(BP_ON == 0) Led_Init();
		HAL_Delay(300);
		// Подготовка команды для передачи по I2C
		i2c_buffer[0] = PWR_ON; // �?спользуем значение из enum
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_3, 1, HAL_MAX_DELAY);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c2, (0x25 << 1), i2c_buffer, 1, HAL_MAX_DELAY); // Передаём буфер
		Set_devslp();
		HAL_TIM_Base_Start_IT(&htim2);
		HAL_TIM_Base_Start_IT(&htim3);
		Counter_sgpio_timeout = 0;
		StartBlinking(&led2, 2, 10000); // Мигание 2 Гц, 10 сек

		while (adapter2_state != 1  ){

			if(HAL_GPIO_ReadPin(MB2_BITCH_GPIO_Port, MB2_BITCH_Pin)!= 1) {
				adapter2_state = 1;
			} else adapter2_state = 0;

		}

	}
	UpdateCPU_PSON(); // Обновляем состояние CPU_PSON
}

void PowerOffAdapter(uint8_t adapter_number)
{
	uint8_t i2c_buffer[1]; // Буфер для передачи данных по I2C

	if (adapter_number == 1 && adapter1_state == 1) {
		// Подготовка команды для передачи по I2C
		i2c_buffer[0] = PWR_OFF; // �?спользуем значение из enum
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_2, 1, HAL_MAX_DELAY);
		HAL_Delay(100);
		HAL_I2C_Master_Transmit(&hi2c2, (0x25 << 1), i2c_buffer, 1, HAL_MAX_DELAY); // Передаём буфер
		while (adapter1_state != 0){
			if(HAL_GPIO_ReadPin(MB1_BITCH_GPIO_Port, MB1_BITCH_Pin)!= 1) {
				adapter1_state = 1;
			} else adapter1_state = 0;
		}


	} else if (adapter_number == 2 && adapter2_state == 1) {
		// Подготовка команды для передачи по I2C
		i2c_buffer[0] = PWR_OFF; // �?спользуем значение из enum
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_3, 1, HAL_MAX_DELAY);
		HAL_Delay(100);
		HAL_I2C_Master_Transmit(&hi2c2, (0x25 << 1), i2c_buffer, 1, HAL_MAX_DELAY); // Передаём буфер
		while (adapter2_state != 0){
			if(HAL_GPIO_ReadPin(MB2_BITCH_GPIO_Port, MB2_BITCH_Pin)!= 1) {
				adapter2_state = 1;
			} else adapter2_state = 0;
		}

	}
	UpdateCPU_PSON(); // Обновляем состояние CPU_PSON
}

void UpdateCPU_PSON()
{
	if (adapter1_state == 1 || adapter2_state == 1) {
		// Если хотя бы один адаптер включен, выставляем CPU_PSON в 1
		HAL_GPIO_WritePin(CPU_PSON_GPIO_Port, CPU_PSON_Pin, SET);
		BP_ON = 1;
	} else {
		// Если оба адаптера выключены, выставляем CPU_PSON в 0
		HAL_GPIO_WritePin(CPU_PSON_GPIO_Port, CPU_PSON_Pin, RESET);
		BP_ON = 0;
	}
}


void HardResetAdapter(uint8_t adapter_number)
{
	uint8_t i2c_buffer[1]; // Буфер для передачи данных по I2C

	if (adapter_number == 1 && adapter1_state == 1) {
		// Подготовка команды для передачи по I2C
		i2c_buffer[0] = HARD_RESET; // �?спользуем значение из enum
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_2, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c2, (0x25 << 1), i2c_buffer, 1, HAL_MAX_DELAY); // Передаём буфер
		HAL_Delay(6000);

		while (adapter1_state != 0){
			if(HAL_GPIO_ReadPin(MB1_BITCH_GPIO_Port, MB1_BITCH_Pin)!= 1) {
				adapter1_state = 1;
			} else adapter1_state = 0;
		}


	} else if (adapter_number == 2 && adapter2_state == 1) {
		// Подготовка команды для передачи по I2C
		i2c_buffer[0] = HARD_RESET; // �?спользуем значение из enum
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_3, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Transmit(&hi2c2, (0x25 << 1), i2c_buffer, 1, HAL_MAX_DELAY); // Передаём буфер
		HAL_Delay(6000);

		while (adapter2_state != 0){
			if(HAL_GPIO_ReadPin(MB2_BITCH_GPIO_Port, MB2_BITCH_Pin)!= 1) {
				adapter2_state = 1;
			} else adapter2_state = 0;
		}

	}

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1) // 8 раз в секунду
	{
		UpdateLEDStates();
		MB1_attach = HAL_GPIO_ReadPin(MB1_ATTACH_GPIO_Port, MB1_ATTACH_Pin);
		MB2_attach = HAL_GPIO_ReadPin(MB2_ATTACH_GPIO_Port, MB2_ATTACH_Pin);

	} else
		if (htim->Instance == TIM2) // 4 раза в секунду
		{

			Read_Disk_Status(0x24, disk_status, 6);
			Decode_Disk_Status(disk_status);
		} else
			if (htim->Instance == TIM3) // 1 раз в секунду
			{

				TransmitTemperature();
				Read_disks_connected();
				if(sgpio_started == 0 && (adapter1_state == 1 || adapter2_state == 1))
				{
					++Counter_sgpio_timeout;
					if (Counter_sgpio_timeout > 250 && sgpio_started == 0)
					{
						sgpio_timeout = 1;


					}
				}
			}
}

void TransmitTemperature() {




    // Получаем максимальную температуру


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

void ProcessPins(uint8_t diskIndex)
{

	if (sgpio_started != 1 && disks[diskIndex].isConnected == 1)
	{
		GPIO_PinState activState = HAL_GPIO_ReadPin(diskPins[diskIndex].activPort, diskPins[diskIndex].activPin);

		// Если состояние изменилось
		if ((activState == GPIO_PIN_SET && previousActivity[diskIndex] == 0) ||
				(activState == GPIO_PIN_RESET && previousActivity[diskIndex] == 1)) {
			// Обновляем статус активности диска
			disks[diskIndex].activity = 1;

			// Обновляем статус диска
			Update_Disk_Status(diskIndex, disks[diskIndex].activity, 0, 0);

			// Сохраняем текущее состояние как предыдущее
			previousActivity[diskIndex] = activState;
			// Запускаем таймер на 5 секунд
			activityTimer[diskIndex] = HAL_GetTick(); // Запоминаем текущее время
		}
	}
}



void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	HAL_TIM_Base_Stop_IT(&htim3);
	if (GPIO_Pin == FP_MB1_PWR_SW_Pin) {
		button1_pressed = 1;
		button1_press_time = HAL_GetTick();
	} else if (GPIO_Pin == FP_MB2_PWR_SW_Pin) {
		button2_pressed = 1;
		button2_press_time = HAL_GetTick();
	}

}
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
	flag_error = 1;
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
