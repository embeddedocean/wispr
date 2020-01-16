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

// If the header size is a multiple of the sample size
// then the buffer will be completely filled by the header and data
// otherwise the buffer will have a few unused bytes at the end
#define ADC_HEADER_SIZE (30)
#define ADC_SAMPLE_SIZE (3)
#define ADC_BLOCKS_PER_BUFFER (48)

#define ADC_DEFAULT_SAMPLING_RATE (100000)

#define ADC_BUFFER_SIZE (ADC_BLOCKS_PER_BUFFER * SD_CARD_BLOCK_SIZE)

#define ADC_NUM_SAMPLES ((ADC_BUFFER_SIZE - ADC_HEADER_SIZE) / ADC_SAMPLE_SIZE)

#define ADC_DEFAULT_WINDOW 10
#define ADC_DEFAULT_INTERVAL 30

// States
#define READING_ADC 0x01
#define WRITING_SD1 0x02
#define WRITING_SD2 0x04
#define POWER_OFF 0x10
#define READY 0x20
#define BOOTING 0x40
#define SHUTTING_DOWN 0x80

// Modes
#define WISPR_WAVEFORM 0x01
#define WISPR_SPECTRUM 0x02
#define WISPR_FILTERED 0x04

//
// Data header object written to the front of each data buffer
//
typedef struct
{
	char      name[5];
	uint8_t   version[2];
	uint8_t   settings[4];
	uint8_t   sample_size; // number of bytes per sample
	uint16_t  blocks_size; // number of bytes in an adc record block
	uint16_t  samples_per_block;  // number of samples in a block
	uint32_t  sampling_rate; // samples per second
	uint32_t  second;
	uint32_t  usec;
	uint8_t   header_chksum;
	uint8_t   data_chksum;
	//uint8_t   year,month,day,hour,minute,second;
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
	uint16_t samples_per_block; // number of samples in a block
	uint16_t block_size;  // number of bytes in an adc record block
	uint8_t  sample_size; // number of bytes per sample
	uint32_t sampling_rate; // samples per second
	uint16_t window; // time in seconds of the adc recors block
	uint16_t interval; // time in seconds between adc records (must be >= window)
	uint8_t active_sd_card;
} wispr_config_t;

int wispr_parse_data_header(uint8_t *buf, wispr_data_header_t *hdr);
int wispr_serialize_data_header(wispr_data_header_t *hdr, uint8_t *buf);
void wispr_print_data_header(wispr_data_header_t *header);

int wispr_write_config(wispr_config_t *hdr);
int wispr_read_config(wispr_config_t *hdr);

void wispr_print_config(wispr_config_t *hdr);

int wispr_parse_config(uint8_t *buf, wispr_config_t *hdr);
int wispr_serialize_config(wispr_config_t *hdr, uint8_t *buf);

#endif /* WISPR_H_ */
