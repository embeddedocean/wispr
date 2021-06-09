/*
 * wispr.h
 *
 * Created: 12/24/2019
 *  Author: Chris
 */ 

#ifndef WISPR_H_
#define WISPR_H_

#include <stdint.h>
#include <arm_math.h>

#include "epoch.h"
#include "gps.h"

#include "board_v2.1.h"

#define WISPR_VERSION 2
#define WISPR_SUBVERSION 1

#define WISPR_I2C_SPEED 100000UL

#define WISPR_SD_CARD_HEADER_BLOCK (30)
#define WISPR_SD_CARD_CONFIG_BLOCK (31)
#define WISPR_SD_CARD_START_BLOCK (32)

// sd cards I/O is in blocks, not bytes, 
// so reads and writes must be done in blocks of this size
// fixed at 512 bytes for standard SD cards
#define WISPR_SD_CARD_BLOCK_SIZE (512)

// number of blocks in a file
//#define WISPR_FILE_SIZE (512)

// The header size should be a multiple of a word size (4 bytes)
// to avoid issues with word alignment when casting
#define WISPR_DATA_HEADER_SIZE (32)

// Fixed values used to define static buffers
#define ADC_SAMPLE_SIZE (2)
#define ADC_BLOCKS_PER_BUFFER (32)
#define ADC_BUFFER_SIZE (ADC_BLOCKS_PER_BUFFER * WISPR_SD_CARD_BLOCK_SIZE)

// if the header is NOT included in adc buffer use this
#define ADC_MAX_SAMPLES_PER_BUFFER (ADC_BUFFER_SIZE / ADC_SAMPLE_SIZE)

// if the header is included in adc buffer use this
//#define ADC_SAMPLES_PER_BUFFER ((ADC_BUFFER_SIZE - WISPR_DATA_HEADER_SIZE) / ADC_SAMPLE_SIZE)

// adc default values
#define ADC_DEFAULT_SAMPLING_RATE (50000)
#define ADC_DEFAULT_SAMPLE_SIZE (ADC_SAMPLE_SIZE)
#define ADC_DEFAULT_AWAKE 60
#define ADC_DEFAULT_SLEEP 0
#define ADC_DEFAULT_GAIN 0
#define ADC_DEFAULT_DECIMATION 4

// dma status flags
#define ADC_DMA_OVERFLOW 0x01
#define ADC_DMA_BUFFER_OVERRUN 0x02
#define ADC_DMA_MISSED_BUFFER 0x04

// adc reference voltage
#define ADC_VREF 5.0
#define ADC_SCALING 5.0 

// spectrum defaults
#define PSD_DEFAULT_FFT_SIZE (1024)
#define PSD_DEFAULT_OVERLAP (0)

// spectrum max buffer size
#define PSD_MAX_FFT_SIZE (2048)
#define PSD_MAX_NUM_BINS (PSD_MAX_FFT_SIZE/2 + 1)

// only float32 or q31 (4 bytes)
#define PSD_SAMPLE_SIZE (4)

// max buffer size in sd card blocks
#define PSD_MAX_BLOCKS_PER_BUFFER (PSD_MAX_NUM_BINS * PSD_SAMPLE_SIZE / WISPR_SD_CARD_BLOCK_SIZE) 
#define PSD_MAX_BUFFER_SIZE (PSD_MAX_BLOCKS_PER_BUFFER * WISPR_SD_CARD_BLOCK_SIZE) 

// if the header is NOT included in psd buffer use this
#define PSD_MAX_BINS_PER_BUFFER (PSD_MAX_BUFFER_SIZE / PSD_SAMPLE_SIZE)

// if the header is included in psd buffer use this
//#define PSD_MAX_BINS_PER_BUFFER ((PSD_BUFFER_SIZE - WISPR_DATA_HEADER_SIZE) / PSD_SAMPLE_SIZE)

// States
#define WISPR_IDLE 0x00
#define WISPR_RUNNING 0x01
#define WISPR_PAUSED 0x02
#define WISPR_SLEEP_WFI 0x04
#define WISPR_SLEEP_BACKUP 0x08
#define WISPR_LOGGING 0x10
#define WISPR_RESET 0x20

// Modes
#define WISPR_DAQ 0x01
#define WISPR_PSD 0x02

// Data types
#define WISPR_INT16 0x01
#define WISPR_INT24 0x02
#define WISPR_INT32 0x04
#define WISPR_FLOAT16 0x10
#define WISPR_FLOAT32 0x20

// Max file size is in units of 512 blocks, 50Mb is a good value 
//#define WISPR_MAX_FILE_SIZE (ADC_BLOCKS_PER_BUFFER * 3052)
//#define WISPR_MAX_FILE_SIZE (50003968 / 512)
#define WISPR_MAX_FILE_SIZE (10000000 / 512)

// settings array index for different data header types
#define ADC_SETTINGS_INDEX_GAIN 0
#define ADC_SETTINGS_INDEX_DF 1
#define ADC_SETTINGS_INDEX_2 2
#define ADC_SETTINGS_INDEX_3 3

#define PSD_SETTINGS_INDEX_SAMPLE_SIZE 0
#define PSD_SETTINGS_INDEX_FFT_WINTYPE 1
#define PSD_SETTINGS_INDEX_FFT_SIZE 2
#define PSD_SETTINGS_INDEX_FFT_OVERLAP 3

// Bit shifting macro with casting to load an int32 from a little-endian buffer containing an int24
#define LOAD_INT24(u8,n) ((int32_t)( ((uint32_t)u8[3*n+0] << 8) | ((uint32_t)u8[3*n+1] << 16) | ((uint32_t)u8[3*n+2] << 24) ) >> 8)

// Bit shifting macro with casting to load an int32 from a little-endian buffer containing an int16
#define LOAD_INT16(u8,n) ((int16_t)( ((uint16_t)u8[2*n+0] << 0) | ((uint16_t)u8[2*n+1] << 8) ))

#define INT16_to_INT32(u8,n) ((int32_t)( (((uint32_t)u8[2*n+0] << 16) | ((uint32_t)u8[2*n+1] << 24) ) >> 8))

//
// Data header object written to the front of each data buffer
//
typedef struct {
	char      name[6];
	uint8_t   version[2];
	uint8_t   type;
	uint32_t  second; // epoch time stamp
	uint32_t  usec;
	uint8_t   settings[4];
	uint8_t   sample_size; // number of bytes per sample
	uint16_t  buffer_size; // number of bytes in an adc data buffer
	uint16_t  samples_per_buffer;  // number of samples in a buffer
	uint32_t  sampling_rate; // samples per second
	uint8_t   channels;  // number of channels
	uint8_t   header_chksum;
	uint8_t   data_chksum;
} wispr_data_header_t;

typedef struct {
	uint8_t  sample_size; // number of bytes per sample
	uint16_t buffer_size;  // number of bytes in a buffer, this can be different than samples_per_buffer*sample_size
	uint16_t samples_per_buffer; // number of samples in a buffer
	uint32_t sampling_rate; // samples per second
	uint8_t  channels;  // number of channels
	uint8_t  gain;  // preamp gain
	uint8_t  decimation;  // 4, 8, 13, or 32
} wispr_adc_t;

//typedef struct {
//	uint8_t  state;
//	uint8_t  mode;
//	uint32_t epoch; // linux time in seconds
//	uint16_t buffers_per_window; // number of adc record buffers in a sampling window
//	uint16_t acquisition_time; // time in seconds of the adc sampling window
//	uint16_t sleep_time; // time in seconds between adc records (must be >= window)
//	uint32_t file_size; // number of block (512 byte) per file
//	uint8_t  active_sd_card; // last card written to
//	uint32_t free; // free number of block (512 byte) on active sd card
//} wispr_control_t;

typedef struct {
	uint32_t second;      // start time
	uint16_t size;        // size of fft
	uint16_t overlap;     // data overlap for each fft
	uint8_t  window_type;
	uint16_t nbins;       // num of freq bins in psd (typically fft_size/2)
	uint16_t navg;        // num of time bins to average
	uint16_t count;
	uint32_t nyquist;     // Hz
} wispr_psd_t;

//
// Configuration data object written to the configuration block of the sd card
//
typedef struct {
	char     name[6];
	uint8_t  version[2];
	uint8_t  mode;
	uint8_t  state;
	uint8_t  prev_state;
	uint32_t epoch; // linux time in seconds
	uint32_t acquisition_time; // time in seconds of the adc sampling window
	uint32_t sleep_time; // time in seconds between adc records (must be >= window)
	uint32_t pause_time; // time in seconds to pause
	uint32_t file_size; // number of block (512 byte) per file
	float32_t secs_per_file; // number of seconds per file
	uint16_t files; // number of files opened
	uint16_t resets; // number of resets
	uint8_t active_sd_card; // last card written to
	wispr_adc_t adc;
	wispr_psd_t psd;
	gps_t gps;
} wispr_config_t;

extern int wispr_parse_data_header(uint8_t *buf, wispr_data_header_t *hdr);
extern int wispr_serialize_data_header(wispr_data_header_t *hdr, uint8_t *buf);
extern void wispr_print_data_header(wispr_data_header_t *header);
//extern void wispr_print_config(wispr_config_t *hdr);
extern int wispr_parse_config(uint8_t *buf, wispr_config_t *hdr);
extern int wispr_serialize_config(wispr_config_t *hdr, uint8_t *buf);
extern void wispr_update_data_header(wispr_config_t *wispr, wispr_data_header_t *hdr);

#ifdef WINDOWS
typedef struct {
	char     label[16];     // user set identifier
	uint8_t  number;        // card number
	uint8_t  version[2];    // wispr version
	uint8_t  hw_ver;        // card hardware version
	uint8_t  state;         // current state (OPEN, ENABLED, ..)
	uint8_t  type;          // hardware type
	uint32_t capacity;      // total card capacity in KBytes
	uint32_t start;         // addr of start block
	uint32_t end;           // addr of end block
	uint32_t write_addr;    // addr of current write block
	uint32_t read_addr;     // addr of current read block
	uint32_t total;         // total number of sectors KBytes
	uint32_t free;          // available sectors in KBytes
	uint32_t epoch;         // mod time
} wispr_sd_card_t;

extern void wispr_print_sd_card_header(wispr_sd_card_t *hdr);
extern int wispr_sd_card_parse_header(uint8_t *buf, wispr_sd_card_t *hdr);
extern int wispr_sd_card_serialize_header(sd_card_t *hdr, uint8_t *buf);
#endif

//extern int wispr_gpbr_write_config(wispr_config_t *hdr);
//extern int wispr_gpbr_read_config(wispr_config_t *hdr);

#endif /* WISPR_H_ */
