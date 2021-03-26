/*
 * ltc2512.h
 *
 * Created: 5/6/2016 11:21:35 AM
 *  Author: chris
 */ 

#ifndef LTC2512_H_
#define LTC2512_H_

#include "wispr.h"

#include "arm_math.h"
#include "arm_const_structs.h"

#include <string.h>

// SSC bus serial bit clock frequency (Hz)
// Make this a factor of the CPU clock so there's no jitter
//#define LTC2512_SSC_RSCK (16000000)
#define LTC2512_SSC_RSCK (24000000)

// If the buffer size is a multiple of the sample size
// then the buffer will be completely filled by the header and data
// otherwise the buffer will have a few unused bytes at the end   

#define LTC2512_MAX_SAMPLES ADC_MAX_SAMPLES_PER_BUFFER

#define LTC2512_DMA_BITS_PER_SAMPLE (32)
#define LTC2512_DMA_BYTES_PER_SAMPLE (4)
#define LTC2512_DMA_BUFFER_NBYTES (LTC2512_MAX_SAMPLES * LTC2512_DMA_BYTES_PER_SAMPLE)

// MCLK modes
#define	LTC2512_MAX_MCLK  1600000  /* MCLK supplied by TC */
#define	LTC2512_MIN_MCLK  1000  /* MCLK supplied by TC */

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

//extern int ltc2512_init(uint32_t *fs, uint8_t df, uint8_t gain);
//extern uint32_t ltc2512_init(wispr_config_t *wispr);
extern uint32_t ltc2512_init(wispr_config_t *wispr, wispr_data_header_t *hdr);
extern uint32_t ltc2512_config_mclk(uint32_t fs, uint8_t df);

extern uint32_t ltc2512_trigger(void);
extern void ltc2512_start(void);
extern void ltc2512_stop(void);
extern void ltc2512_shutdown(void);
extern void ltc2512_pause(void);

extern uint16_t ltc2512_read_dma(wispr_data_header_t *hdr, uint8_t *data);
extern uint8_t *ltc2512_get_dma_buffer(void);
extern uint16_t ltc2512_init_dma(uint16_t nsamps);
extern void ltc2512_start_dma(void);
extern void ltc2512_stop_dma(void);

extern void ltc2512_init_test(wispr_config_t *wispr, uint16_t nsamps, uint32_t freq, float32_t amp, float32_t noise_amp);

//extern void ltc2512_get_date(uint8_t *cent, uint8_t *year, uint8_t *month, uint8_t *day, uint8_t *week);
//extern void ltc2512_get_time(uint8_t *hour, uint8_t *minute, uint8_t *second, uint32_t *usec);
extern void ltc2512_get_datetime(uint8_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *minute, uint8_t *second, uint32_t *usec);

extern int ltc2512_init2(uint32_t *fs, uint32_t df);
extern uint16_t ltc2512_init_dma2(void);

#endif /* LTC2512_H_ */