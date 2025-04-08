/*
 * diks.c
 *
 *  Created on: Mar 28, 2025
 *      Author: user
 */

#include "i2c.h"
#include "config.h"
#include "main.h"
#include "disk.h"

void Read_disks_connected()
{

	static uint8_t prevConnected[MAX_DISKS] = {0}; // Предыдущее состояние подключения
	HAL_Delay(20);
	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_0, 1, 1000);
	HAL_Delay(20);
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
	HAL_Delay(20);
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

	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_1, 1, 1000);
	HAL_Delay(20);
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
	HAL_Delay(20);
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
	HAL_Delay(20);
	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_2, 1, 1000);
 	HAL_Delay(20);
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
	HAL_Delay(20);
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
	for (int i = 0; i < MAX_DISKS; ++i) {
		// Если состояние подключения изменилось с 0 на 1
		if (disks[i].isConnected == 1 && prevConnected[i] == 0) {
			disks[i].activity = 1; // Устанавливаем активность
			connectActivityTimer[i] = HAL_GetTick(); // Запускаем таймер
		}
		prevConnected[i] = disks[i].isConnected; // Сохраняем текущее состояние
	}
}

void Set_devslp()
{
	while (HAL_I2C_IsDeviceReady(&hi2c2, I2C_EXPAND_adr << 1, 3, 100) != HAL_OK){}
	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_0, 1, HAL_MAX_DELAY); //DevSLP  0 канал (E/F)
	HAL_Delay(20);
	while (HAL_I2C_IsDeviceReady(&hi2c2, Dev_SLP_adr << 1, 3, 100) != HAL_OK){}
	HAL_I2C_Master_Transmit(&hi2c2, (Dev_SLP_adr << 1), i2cbuff_IN, 3, HAL_MAX_DELAY); // init input
	HAL_Delay(20);
	while (HAL_I2C_IsDeviceReady(&hi2c2, Dev_SLP_adr << 1, 3, 100) != HAL_OK){}
	HAL_I2C_Master_Transmit(&hi2c2, (Dev_SLP_adr << 1), Dev_SLP_ON, 3, HAL_MAX_DELAY); //write
	HAL_Delay(20);

	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_1, 1, HAL_MAX_DELAY); //DevSLP  1 канал (C/D)
	HAL_Delay(20);

	HAL_I2C_Master_Transmit(&hi2c2, (Dev_SLP_adr << 1), i2cbuff_IN, 3, HAL_MAX_DELAY);
	HAL_Delay(20);

	HAL_I2C_Master_Transmit(&hi2c2, (Dev_SLP_adr << 1), Dev_SLP_ON, 3, HAL_MAX_DELAY);
	HAL_Delay(20);

	HAL_I2C_Master_Transmit(&hi2c2, (I2C_EXPAND_adr << 1), I2CInit_2, 1, HAL_MAX_DELAY); //DevSLP  2 канал (A/B)
	HAL_Delay(20);

	HAL_I2C_Master_Transmit(&hi2c2, (Dev_SLP_adr << 1), i2cbuff_IN, 3, HAL_MAX_DELAY);
	HAL_Delay(20);

	HAL_I2C_Master_Transmit(&hi2c2, (Dev_SLP_adr << 1), Dev_SLP_ON, 3, HAL_MAX_DELAY);
	HAL_Delay(20);
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

void UpdateDiskStatus(uint8_t diskIndex, uint8_t activity, uint8_t error, uint8_t locate) //функция для обновления данных о дисках
{
	disks[diskIndex].activity = activity;
	disks[diskIndex].error = error;
	disks[diskIndex].locate = locate;
}

void Read_Disk_Status(uint16_t slave_address, uint8_t *data, uint16_t size) {

	if(MB1_attach == 0  && MB2_attach == 0)
	{
		if(adapter1_state == 1 && adapter2_state == 1){
			if(flag)
					{
						HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_0, 1, 500);
						HAL_Delay(20);
						HAL_I2C_Master_Receive(&hi2c2, slave_address << 1, data, size, 500);
						flag = 0;
					} else {
						HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_1, 1, 500);
						HAL_Delay(20);
						HAL_I2C_Master_Receive(&hi2c2, slave_address << 1, data, size, 500);
						flag = 1;
					}
		} else if (adapter1_state == 1 && adapter2_state == 0)
		{
			HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_0, 1, 1000);
			HAL_Delay(20);
			HAL_I2C_Master_Receive(&hi2c2, slave_address << 1, data, size, 1000);
		}else if (adapter2_state == 1 && adapter1_state == 0)
		{
			HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_1, 1, 1000);
			HAL_Delay(20);
			HAL_I2C_Master_Receive(&hi2c2, slave_address << 1, data, size, 1000);
		}


	}else if ( MB1_attach == 0 && adapter1_state == 1){
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_0, 1, 1000);
		HAL_Delay(20);
		HAL_I2C_Master_Receive(&hi2c2, slave_address << 1, data, size, 1000);
	} else if (MB2_attach == 0 && adapter2_state == 1){
		HAL_I2C_Master_Transmit(&hi2c2, (I2C_adapter_adr << 1), I2CInit_1, 1, 1000);
		HAL_Delay(20);
		HAL_I2C_Master_Receive(&hi2c2, slave_address << 1, data, size, 1000);
	}else {

	}
}

void Decode_Disk_Status(uint8_t *data) {
    for (uint8_t disk_id = 0; disk_id < 24; disk_id++) {
        uint8_t byte_index = disk_id / 4; // Индекс байта
        uint8_t bit_offset = (disk_id % 4) * 2; // Смещение в байте
        uint8_t status = (data[byte_index] >> bit_offset) & 0x03; // Извлечение статуса

        // Получаем текущее состояние диска
        uint8_t current_error = disks[disk_id].error;
        uint8_t current_locate = disks[disk_id].locate;
        uint8_t current_activity = disks[disk_id].activity;

        // Декодирование нового статуса
        uint8_t new_activity = 0;
        uint8_t new_error = 0;
        uint8_t new_locate = 0;

        switch (status) {
            case 0x00: // Все флаги сброшены
                new_activity = 0;
                new_error = 0;
                new_locate = 0;
                break;
            case 0x01: // Activity
                new_activity = 1;
                new_error = 0;
                new_locate = 0;
                break;
            case 0x02: // Locate
                new_activity = 0;
                new_error = 0;
                new_locate = 1;
                break;
            case 0x03: // Error
                new_activity = 0;
                new_error = 1;
                new_locate = 0;
                break;
            default:
                break;
        }

        // Применяем приоритеты:
        // 1. Если уже есть error, сохраняем его
        // 2. Иначе если есть locate, сохраняем его, если новый статус не error
        // 3. Иначе обновляем activity

        uint8_t final_error = current_error || new_error;
        uint8_t final_locate = (current_locate || new_locate) && !final_error;
        uint8_t final_activity = new_activity && !final_error && !final_locate;

        // Обновляем статус только если он изменился
        if (disks[disk_id].error != final_error ||
            disks[disk_id].locate != final_locate ||
            disks[disk_id].activity != final_activity) {
            UpdateDiskStatus(disk_id, final_activity, final_error, final_locate);
        }
    }
}
