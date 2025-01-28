/*
 * create_wave_data.h
 *
 *  Created on: Jan 23, 2025
 *      Author: johan
 */

#ifndef INC_CREATE_WAVE_DATA_H_
#define INC_CREATE_WAVE_DATA_H_

int mount_sd_card(FATFS* FatFs);
int create_wave_file(const char* name, FIL* fil);
int close_wave_file(FIL* fil, unsigned int* number_of_data_bytes_written);
void demount_sd_card();

#endif /* INC_CREATE_WAVE_DATA_H_ */
