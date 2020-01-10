/*
 * wispr.h
 *
 * Created: 12/24/2019 4:59:31 AM
 *  Author: Chris
 */ 

#ifndef WISPR_H_
#define WISPR_H_

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

// States
#define READING_ADC 0x01
#define WRITING_SD1 0x02
#define WRITING_SD2 0x04
#define POWER_OFF 0x10
#define READY 0x20
#define BOOTING 0x40
#define SHUTTING_DOWN 0x80

//
// Data header object written to the front of each data buffer
//
typedef struct
{
	char      name[5];
	uint8_t   version[2];
	uint8_t   size;
	uint8_t   settings[2];
	uint8_t   bytes_per_sample;
	uint16_t  num_samples;
	uint32_t  sampling_rate;
	uint8_t   year;
	uint8_t   month;
	uint8_t   day;
	uint8_t   hour;
	uint8_t   minute;
	uint8_t   second;
	uint32_t  usec;
	uint8_t   header_chksum;
	uint8_t   data_chksum;
	uint8_t   pad;
} wispr_data_header_t;

//
// Configuration data object written to the configuration block of the sd card 
//
typedef struct {
	char     name[6];
	uint8_t  version[2];
	uint8_t  size;  // size in bytes of this record
	uint8_t  year;
	uint8_t  month;
	uint8_t  day;
	uint8_t  hour;
	uint8_t  minute;
	uint8_t  second;
	// adc config
	uint8_t  blocks_per_record;
	uint16_t samples_per_record;
	uint8_t  bytes_per_sample;
	uint32_t sampling_rate;
	uint8_t  adc_settings[2];
	uint8_t  preamp_settings[2];
	// other configs
	// ...
} wispr_config_t;


int wispr_parse_data_header(uint8_t *buf, wispr_data_header_t *hdr);
int wispr_serialize_data_header(wispr_data_header_t *hdr, uint8_t *buf);
void wispr_print_data_header(wispr_data_header_t *header);

int wispr_parse_config(uint8_t *buf, wispr_config_t *hdr);
int wispr_serialize_config(wispr_config_t *hdr, uint8_t *buf);
void wispr_print_config(wispr_config_t *hdr);


#endif /* WISPR_H_ */
