/*
 * disk.h
 *
 *  Created on: Mar 28, 2025
 *      Author: user
 */

#ifndef SRC_DISK_H_
#define SRC_DISK_H_


extern DiskStatus disks[MAX_DISKS]; // Массив для хранения статусов всех дисков

void Read_disks_connected();
void Set_devslp();
void Initialize_Disks();
void UpdateDiskStatus(uint8_t diskIndex, uint8_t activity, uint8_t error, uint8_t locate);
void Read_Disk_Status(uint16_t slave_address, uint8_t *data, uint16_t size);
void Decode_Disk_Status(uint8_t *data);

#endif /* SRC_DISK_H_ */
