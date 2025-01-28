
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "fatfs.h"



int mount_sd_card(FATFS* FatFs){
	//Open the file system
	if (f_mount(FatFs, "", 1) != FR_OK) { //1=mount now
		return 0;
	}
	return 1;
}

int create_wave_file(const char* name, FIL* fil)
{
	FRESULT fres;

	fres = f_open(fil, name, FA_WRITE | FA_CREATE_NEW);
	if (fres != FR_OK) {
		return 0;
	}


	struct wav_header
	{
	  char riff[4];           /* "RIFF"                                  */
	  int32_t flength;        /* file length in bytes                    */
	  char wave[4];           /* "WAVE"                                  */
	  char fmt[4];            /* "fmt "                                  */
	  int32_t chunk_size;     /* size of FMT chunk in bytes (usually 16) */
	  int16_t format_tag;     /* 1=PCM, 257=Mu-Law, 258=A-Law, 259=ADPCM */
	  int16_t num_chans;      /* 1=mono, 2=stereo                        */
	  int32_t srate;          /* Sampling rate in samples per second     */
	  int32_t bytes_per_sec;  /* bytes per second = srate*bytes_per_samp */
	  int16_t bytes_per_samp; /* 2=16-bit mono, 4=16-bit stereo          */
	  int16_t bits_per_samp;  /* Number of bits per sample               */
	  char data[4];           /* "data"                                  */
	  int32_t dlength;        /* data length in bytes (filelength - 44)  */
	};


	struct wav_header wavh;

	const int header_length = sizeof(struct wav_header);

	strncpy(wavh.riff, "RIFF", 4);
	strncpy(wavh.wave, "WAVE", 4);
	strncpy(wavh.fmt, "fmt ", 4);
	strncpy(wavh.data, "data", 4);

	wavh.chunk_size = 16;
	wavh.format_tag = 1;
	wavh.num_chans = 1;
	wavh.srate = 48000;
	wavh.bits_per_samp = 16;
	wavh.bytes_per_sec = wavh.srate * wavh.bits_per_samp / 8 * wavh.num_chans;
	wavh.bytes_per_samp = wavh.bits_per_samp / 8 * wavh.num_chans;


	unsigned int res;
    f_write(fil, &wavh, header_length, &res);

	return 1;
}

int close_wave_file(FIL* fil, unsigned int* number_of_data_bytes_written){
	//Insert data into header of file
	f_lseek(fil, 4);

	unsigned int res;

	unsigned int file_length = (*number_of_data_bytes_written) + 44;
	f_write(fil, &file_length, 4, &res);

	f_lseek(fil, 40);

	f_write(fil, number_of_data_bytes_written, 4, &res);

	//Close file
	if(f_close(fil) != FR_OK){
		return 0;
	}
	return 1;
}


void demount_sd_card() {
	f_mount(NULL, "", 0);
}



