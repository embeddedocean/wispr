/*
 * wispr.h
 *
 * Created: 12/24/2019 4:59:31 AM
 *  Author: Chris
 */ 

#ifndef WISPR_H_
#define WISPR_H_

#include "epoch.h"

#define WISPR_VERSION 2
#define WISPR_SUBVERSION 0

#define WISPR_SD_CARD_HEADER_BLOCK 30
#define WISPR_SD_CARD_CONFIG_BLOCK 31
#define WISPR_SD_CARD_START_BLOCK 32

#define WISPR_SD_CARD_BLOCK_SIZE (512)

// If the header size should be a multiple of a word size (4 bytes)
// to avoid issues with word alignment when casting
#define WISPR_DATA_HEADER_SIZE (32)

#define ADC_DEFAULT_SAMPLING_RATE (50000)
#define ADC_MIN_SAMPLE_SIZE (2)
#define ADC_MAX_BLOCKS_PER_BUFFER (32)
#define ADC_MAX_BUFFER_SIZE (ADC_MAX_BLOCKS_PER_BUFFER * WISPR_SD_CARD_BLOCK_SIZE)
#define ADC_MAX_SAMPLES_PER_BUFFER ((ADC_MAX_BUFFER_SIZE - WISPR_DATA_HEADER_SIZE) / ADC_MIN_SAMPLE_SIZE)

#define ADC_DEFAULT_AWAKE 10
#define ADC_DEFAULT_SLEEP 10

#define ADC_DEFAULT_GAIN 0

// adc reference voltage
#define ADC_VREF 5.0
#define ADC_SCALING 5.0

// spectrum uses float32_t ffts, so each freq bin is a float32
#define PSD_MAX_FFT_SIZE 512
#define PSD_MAX_BLOCKS_PER_BUFFER 2 
#define PSD_MAX_BUFFER_SIZE (PSD_MAX_BLOCKS_PER_BUFFER * WISPR_SD_CARD_BLOCK_SIZE) 
#define PSD_MAX_BINS_PER_BUFFER ((PSD_MAX_BUFFER_SIZE - WISPR_DATA_HEADER_SIZE) / 4)

// States
#define WISPR_READING_ADC 0x01
#define WISPR_WRITING_SD1 0x02
#define WISPR_WRITING_SD2 0x04
#define WISPR_POWER_OFF 0x10
#define WISPR_READY 0x20
#define WISPR_BOOTING 0x40
#define WISPR_SHUTTING_DOWN 0x80

// Modes
#define WISPR_WAVEFORM 0x01
#define WISPR_SPECTRUM 0x02

// Data types
#define WISPR_INT16 0x01
#define WISPR_INT24 0x02
#define WISPR_INT32 0x04
#define WISPR_FLOAT16 0x10
#define WISPR_FLOAT32 0x20

//
// Data header object written to the front of each data buffer
//
typedef struct {
	char      name[5];
	uint8_t   version[2];
	uint8_t   settings[4];
	uint8_t   sample_size; // number of bytes per sample
	uint16_t  block_size; // number of bytes in an adc data block
	uint16_t  samples_per_block;  // number of samples in a block
	uint32_t  sampling_rate; // samples per second
	uint32_t  second; // epoch time stamp
	uint32_t  usec;
	uint8_t   header_chksum;
	uint8_t   data_chksum;
} wispr_data_header_t;

//
// Configuration data object written to the configuration block of the sd card 
//
typedef struct {
	char     name[6];
	uint8_t  version[2];
	uint8_t  state;
	uint8_t  mode;
	uint32_t epoch;
	uint8_t  settings[8]; // various system and adc settings
	uint8_t  sample_size; // number of bytes per sample
	uint16_t samples_per_block; // number of samples in a block
	uint32_t sampling_rate; // samples per second
	uint16_t block_size;  // number of bytes in a block, this can be different than samples_per_block*sample_size
	uint16_t blocks_per_window; // 
	uint16_t awake_time; // time in seconds of the adc records block
	uint16_t sleep_time; // time in seconds between adc records (must be >= window)
	uint16_t fft_size; //
	uint8_t active_sd_card;
} wispr_config_t;

typedef struct {
	char    name[8];      // user set identifier
	uint8_t number;
	uint8_t state;
	uint8_t type;
	uint8_t version;
	uint32_t capacity;     // card capacity in KBytes
	uint32_t start_block;  // addr of start block
	uint32_t end_block;    // addr of end block
	uint32_t write_addr;   // addr of current write block
	uint32_t read_addr;    // addr of current read block
	uint32_t epoch;        // time last header was written
} wispr_sd_card_t;

extern int wispr_parse_data_header(uint8_t *buf, wispr_data_header_t *hdr);
extern int wispr_serialize_data_header(wispr_data_header_t *hdr, uint8_t *buf);
extern void wispr_print_data_header(wispr_data_header_t *header);

extern int wispr_gpbr_write_config(wispr_config_t *hdr);
extern int wispr_gpbr_read_config(wispr_config_t *hdr);

extern void wispr_print_config(wispr_config_t *hdr);

extern int wispr_parse_config(uint8_t *buf, wispr_config_t *hdr);
extern int wispr_serialize_config(wispr_config_t *hdr, uint8_t *buf);

extern void wispr_update_data_header(wispr_config_t *wispr, wispr_data_header_t *hdr);

extern int wispr_sd_card_parse_header(uint8_t *buf, wispr_sd_card_t *hdr);
extern int wispr_sd_card_serialize_header(wispr_sd_card_t *hdr, uint8_t *buf);

#endif /* WISPR_H_ */
