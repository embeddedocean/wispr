/*
 * ltc2512.h
 *
 * Created: 5/6/2016 11:21:35 AM
 *  Author: chris
 */ 

#ifndef LTC2512_H_
#define LTC2512_H_

#include "wispr.h"

#include <string.h>

// SSC bus serial bit clock frequency (Hz)
#define LTC2512_SSC_RSCK (12000000)

#define LTC2512_DMA_BITS_PER_SAMPLE (32)
#define LTC2512_DMA_BYTES_PER_SAMPLE (4)

#define LTC2512_BYTES_PER_SAMPLE ADC_SAMPLE_SIZE
#define LTC2512_BITS_PER_SAMPLE (8*ADC_SAMPLE_SIZE)

// If the header size is a multiple of the sample size
// then the buffer will be completely filled by the header and data
// otherwise the buffer will have a few unused bytes at the end   
#define LTC2512_HEADER_NBYTES ADC_HEADER_SIZE

//#define LTC2512_SPI_BUFFER_NBLOCKS (6)
//#define LTC2512_SPI_BUFFER_NBYTES (LTC2512_SPI_BUFFER_NBLOCKS * SD_MMC_BLOCK_SIZE)

#define LTC2512_BUFFER_NBLOCKS ADC_BLOCKS_PER_BUFFER
#define LTC2512_BUFFER_NBYTES ADC_BUFFER_SIZE 
#define LTC2512_NUM_SAMPLES ADC_NUM_SAMPLES

#define LTC2512_DMA_BUFFER_NBYTES (LTC2512_NUM_SAMPLES * LTC2512_DMA_BYTES_PER_SAMPLE)

// MCLK modes
#define	LTC2512_MAX_MCLK  1600000  // MCLK supplied by TC
#define	LTC2512_MIN_MCLK  1000  // MCLK supplied by TC

// Down sampling factors
#define	LTC2512_DF4  4
#define	LTC2512_DF8  8
#define	LTC2512_DF16 16
#define	LTC2512_DF32 32

typedef struct {
	uint8_t settings;
	uint8_t bytes_per_samp;
	uint8_t nblocks_per_adc_buffer;
	uint32_t total_nblocks;
	uint16_t nsamps;
	uint32_t fs;
} ltc2512_t;

// nblocks per buffer is 3/4 the size of the dma buffer because we only need 24 of the 32 bits

extern int ltc2512_init(uint32_t *fs, uint8_t df, uint8_t gain);
extern int ltc2512_config_mclk(uint32_t *fs);

extern void ltc2512_start_conversion(void);
extern void ltc2512_stop_conversion(void);
extern uint16_t ltc2512_read_dma(uint8_t *hdr, uint8_t *data);
extern uint8_t *ltc2512_get_dma_buffer(void);
extern uint16_t ltc2512_init_dma(void);
extern void ltc2512_start_dma(void);
extern void ltc2512_stop_dma(void);
extern void ltc2512_update_header(uint8_t *hdr, uint8_t chksum);

extern void ltc2512_get_date(uint8_t *cent, uint8_t *year, uint8_t *month, uint8_t *day, uint8_t *week);
extern void ltc2512_get_time(uint8_t *hour, uint8_t *minute, uint8_t *second, uint32_t *usec);

extern int ltc2512_init2(uint32_t *fs, uint32_t df);
extern uint16_t ltc2512_init_dma2(void);

#endif /* LTC2512_H_ */