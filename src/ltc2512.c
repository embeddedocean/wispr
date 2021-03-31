/*
 * ltc2512.c
 *
 * Created: 1/2018 
 *  Author: chris
 */ 
 
#include <asf.h>
#include <stdio.h>
#include <stdlib.h>

#include "ltc2512.h"
#include "wispr.h"
#include "pps_timer.h"

/* The SSC interrupt IRQ priority. */
#define SSC_ADC_IRQ_PRIO 0

/* DMA channel */
//#define SSC_ADC_DMA_CH 0

/* Pdc transfer buffer */
COMPILER_WORD_ALIGNED uint8_t ltc_adc_dma_buffer1[LTC2512_DMA_BUFFER_NBYTES];
COMPILER_WORD_ALIGNED uint8_t ltc_adc_dma_buffer2[LTC2512_DMA_BUFFER_NBYTES];

/* PDC data packet for transfer */
pdc_packet_t pdc_ssc_packet1;
pdc_packet_t pdc_ssc_packet2;

/* Pointer to SSC PDC register base */
Pdc *ssc_pdc;

/* active buffer number - the number of the buffer being read now */
volatile uint8_t pdc_active_buffer_number = 1;

volatile uint8_t pdc_buffer_locked = 0;

uint8_t pdc_buffer1_read = 0;
uint8_t pdc_buffer2_read = 0;

// finished buffer
uint8_t *ltc_adc_buffer;

uint8_t ltc_adc_sample_size; // bytes
uint32_t ltc_adc_sampling_freq; // Hz
uint8_t ltc_down_sampling_factor;
uint16_t ltc_samples_per_buffer = 0;

int ltc_adc_buffer_number = 1;

uint32_t ltc_adc_time = 0;
uint32_t ltc_adc_date = 0;
uint32_t ltc_adc_tcv = 0;
//uint32_t ltc_adc_time1 = 0;
//uint32_t ltc_adc_date1 = 0;
//uint32_t ltc_adc_tcv1 = 0;
//uint32_t ltc_adc_time2 = 0;
//uint32_t ltc_adc_date2 = 0;
//uint32_t ltc_adc_tcv2 = 0;

uint32_t ltc_adc_sec = 0;
uint32_t ltc_adc_usec = 0;
uint32_t ltc_adc_sec1 = 0;
uint32_t ltc_adc_usec1 = 0;
uint32_t ltc_adc_sec2 = 0;
uint32_t ltc_adc_usec2 = 0;

uint8_t ltc_adc_status = 0; // status flag

// local instance used to update the data buffer header
//wispr_data_header_t adc_data_header;

//
// initialize ltc2512 hardware
//
uint32_t ltc2512_init(wispr_config_t *wispr, wispr_data_header_t *hdr)
{
	// pull out the needed values
	uint32_t fs = wispr->sampling_rate;
	uint8_t df = wispr->adc_decimation;
	uint8_t gain = wispr->gain;
	
	ltc_samples_per_buffer = wispr->samples_per_buffer;
	ltc_adc_sample_size = wispr->sample_size;
	ltc_down_sampling_factor = df;
	
	if( (ltc_samples_per_buffer < 1) || (ltc_samples_per_buffer > LTC2512_MAX_SAMPLES) ) {
		printf("ltc2512_init: invalid number of samples per buffer\n\r");
		return(-1);
	}

	if( !((df == 4)||(df == 8)||(df == 16)||(df == 32)) ) {
		printf("ltc2512_init: invalid decimation factor %d\n\r", df);
		df = ADC_DEFAULT_DECIMATION;
	}
	
	if( (gain < 0) || (gain > 3) ) {
		printf("ltc2512_init: invalid gain %d\n\r", gain);
		gain = ADC_DEFAULT_GAIN;
	}

	// ADC control pins
	ioport_set_pin_dir(PIN_ENABLE_ADC_PWR, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_ENABLE_ADC_PWR, 0); // low is off

	ioport_set_pin_dir(PIN_ADC_SYNC, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_ADC_SYNC, 0);

	ioport_set_pin_dir(PIN_ADC_SEL0, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_ADC_SEL0, 0);

	ioport_set_pin_dir(PIN_ADC_SEL1, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_ADC_SEL1, 0);

	// ADC Preamp control pins
	ioport_set_pin_dir(PIN_PREAMP_SHDN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_PREAMP_SHDN, 0);

	ioport_set_pin_dir(PIN_PREAMP_G0, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_PREAMP_G0, 0);

	ioport_set_pin_dir(PIN_PREAMP_G1, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_PREAMP_G1, 0);

	// power up and initialize adc
	ioport_set_pin_level(PIN_ENABLE_ADC_PWR, 1);
	delay_ms(100);

	// turn on pre-amp
	ioport_set_pin_level(PIN_PREAMP_SHDN, 1);

	// select decimation factor using gpio
	if( df == LTC2512_DF4 ) {
		ioport_set_pin_level(PIN_ADC_SEL0, 0);
		ioport_set_pin_level(PIN_ADC_SEL1, 0);
	} else if( df == LTC2512_DF8 ) {
		ioport_set_pin_level(PIN_ADC_SEL0, 1);
		ioport_set_pin_level(PIN_ADC_SEL1, 0);
	} else if( df == LTC2512_DF16 ) {
		ioport_set_pin_level(PIN_ADC_SEL0, 0);
		ioport_set_pin_level(PIN_ADC_SEL1, 1);
	} else if( df == LTC2512_DF32 ) {
		ioport_set_pin_level(PIN_ADC_SEL0, 1);
		ioport_set_pin_level(PIN_ADC_SEL1, 1);
	} else {
		fprintf(stdout, "ltc2512_init: invalid DF\n\r");
		return(-1);
	}

	// set preamp gain
	ioport_set_pin_level(PIN_PREAMP_G0, (gain & 0x01));
	ioport_set_pin_level(PIN_PREAMP_G1, (gain & 0x02));
	//printf("pre-amp gain = 0x%d%d \n\r", (gain & 0x02), (gain & 0x01));
	
	/* Initialize the local variable. */
	clock_opt_t rx_clk_option;
	data_frame_opt_t rx_data_frame_option;
	memset((uint8_t *)&rx_clk_option, 0, sizeof(clock_opt_t));
	memset((uint8_t *)&rx_data_frame_option, 0, sizeof(data_frame_opt_t));

	// Configure SSC pins
	pio_configure_pin(PIN_SSC_RD, PIN_SSC_RD_FLAGS);
	pio_configure_pin(PIN_SSC_RF, PIN_SSC_RF_FLAGS);
	pio_configure_pin(PIN_SSC_RK, PIN_SSC_RK_FLAGS);
	pio_configure_pin(PIN_SSC_TD, PIN_SSC_TD_FLAGS);
	//  pio_configure_pin(PIN_SSC_TF, PIN_SSC_TF_FLAGS);
	pio_configure_pin(PIN_SSC_TK, PIN_SSC_TK_FLAGS);

	/* Initialize the SSC module */
	pmc_enable_periph_clk(ID_SSC);
	ssc_reset(SSC);

	// set serial clock rate
	//uint32_t sclk = board_get_cpu_clock_hz(); // sysclk_get_peripheral_bus_hz(TC0);
	uint32_t sclk = sysclk_get_peripheral_bus_hz(TC0);
	uint32_t rsck = LTC2512_SSC_RSCK;  // bit clock freq is fixed to the max reliable rate found by testing
	//uint32_t rsck = df * fs * 40;  // bit clock freq
	uint32_t rsck_div = sclk / (2 * rsck);
	rsck = sclk / rsck_div /2;

	SSC->SSC_CMR = SSC_CMR_DIV(rsck_div);
	
	/* Receiver clock mode configuration. */
	// It's really easy to loose the first received bit of the data word, which really messes up the sign of the word
	// So test any changes to these rx clock and frame registers
	rx_clk_option.ul_cks = SSC_RCMR_CKS_MCK;  // select divided clock source
	rx_clk_option.ul_cko = SSC_RCMR_CKO_TRANSFER; // Receive Clock only during data transfers, RK pin is an output
	rx_clk_option.ul_cki = 1; // sampled on Receive Clock rising edge - adc clocks out data on rising edge.
	rx_clk_option.ul_ckg = SSC_RCMR_CKG_EN_RF_LOW; // clock gating selection, enable clock only when RF is low
	rx_clk_option.ul_start_sel = SSC_RCMR_START_RF_FALLING; // Detection of a falling edge on RF signal
	rx_clk_option.ul_sttdly = 0; // num clock cycles delay between start event and reception
	rx_clk_option.ul_period = 0; //ul_rfck_div;  // length of frame in clock cycles
	
	/* Receiver frame mode configuration. */
	rx_data_frame_option.ul_datlen = 31; //LTC2512_BITS_PER_SAMPLE - 1; // number of bits per data word, should be 0 to 31

	// read data word MSB first - although it's stored in processor memory as little-endian or LSB first
	rx_data_frame_option.ul_msbf = SSC_RFMR_MSBF;  // MSB First
	//rx_data_frame_option.ul_msbf = 0;  // LSB First

	rx_data_frame_option.ul_datnb = 0;  //  number of data words per frame, should be 0 to 15. 0 = 1 word
	// 32 bit data word
	rx_data_frame_option.ul_fslen = 15; // Frame Sync. length should be 0 to 15
	rx_data_frame_option.ul_fslen_ext = 1; // Frame Sync. length extension field, should be 0 to 15.
	rx_data_frame_option.ul_fsos = SSC_RFMR_FSOS_NONE; // Frame Sync. is an input
	rx_data_frame_option.ul_fsedge = SSC_RFMR_FSEDGE_NEGATIVE; // Frame Sync. edge detection
	
	/* Configure the SSC receiver. */
	ssc_set_receiver(SSC, &rx_clk_option, &rx_data_frame_option);
	
	// config the adc master clock and return the actual sampling rate
	uint32_t actual_fs = ltc2512_config_mclk(fs, df);

	// initialize the circular dma buffers
	ltc2512_init_dma(ltc_samples_per_buffer);
	
	if(actual_fs != fs) {
		printf("Actual sampling rate: %lu Hz\n\r", actual_fs);
	}

	// update local static variables, in case they have changed
	ltc_samples_per_buffer = wispr->samples_per_buffer;
	ltc_adc_sample_size = wispr->sample_size;
	ltc_down_sampling_factor = df;
	ltc_adc_sampling_freq = actual_fs;
	ltc_adc_status = 0;

	// update adc data header structure with the current config
	// make sure to set all the fields because this is what gets written to storage
	hdr->version[0] = wispr->version[0];
	hdr->version[1] = wispr->version[1];
	hdr->type = WISPR_WAVEFORM;
	hdr->sample_size = wispr->sample_size; // number of bytes per sample
	hdr->buffer_size = wispr->buffer_size; // number of bytes in an adc record buffer
	hdr->samples_per_buffer = 0; // nothing yet
	hdr->sampling_rate = actual_fs; // actual samples per second
	hdr->settings[0] = gain;
	hdr->settings[1] = df;
	hdr->settings[2] = 0;
	hdr->settings[3] = ltc_adc_status;
	hdr->second = 0;
	hdr->usec = 0;

	// set sync high - adc won't start until sync goes low
	ioport_set_pin_level(PIN_ADC_SYNC, 1);
	
	return(actual_fs);
}

//
// Configure TC0_TIOA to supply MCLK for the ADC
//

uint8_t enable_sw_sync = 0;

uint32_t ltc2512_config_mclk(uint32_t fs, uint8_t df)
{
	// Configure TC TC_CHANNEL_WAVEFORM in waveform operating mode.
	//see: http://www.allaboutcircuits.com/projects/dma-digital-to-analog-conversion-with-a-sam4s-microcontroller-the-timer-cou/
	
	// Configure PA0 Pin for TC TIOA 
	ioport_set_pin_mode(PIN_TC0_TIOA0, PIN_TC0_TIOA0_MUX);
	
	// Disable IO to enable peripheral mode 
	ioport_disable_pin(PIN_TC0_TIOA0);
	
	// configure PA1 as either a clock or gpio
	ioport_set_pin_mode(PIN_TC0_TIOB0, PIN_TC0_TIOB0_MUX);
	ioport_disable_pin(PIN_TC0_TIOB0);

	if(enable_sw_sync) {
		ioport_set_pin_dir(PIN_TC0_TIOB0, IOPORT_DIR_OUTPUT);
		ioport_set_pin_level(PIN_TC0_TIOB0, 0);	
	}
	
	// Configure the PMC to enable the TC module.
	sysclk_enable_peripheral_clock(ID_TC0);
	
	// Init TC to waveform mode. 
	uint32_t mode =	  TC_CMR_TCCLKS_TIMER_CLOCK1	//MCK/2
					| TC_CMR_WAVE	//waveform mode
					| TC_CMR_WAVSEL_UP_RC	//UP mode with automatic trigger on RC Compare
					| TC_CMR_ACPA_TOGGLE	//toggle TIOA on RA match
					| TC_CMR_ACPC_TOGGLE;	//toggle TIOA on RC match

	if(enable_sw_sync) {
		mode |=   TC_CMR_BCPB_TOGGLE	//toggle TIOB on RB match
			    | TC_CMR_BCPC_TOGGLE	//toggle TIOB on RC match
			    // for TIOB to configured as an output an external event needs to be selected
			    | TC_CMR_EEVT_XC0 | TC_CMR_AEEVT_NONE | TC_CMR_BEEVT_NONE | TC_CMR_EEVTEDG_NONE;
	}
	
	tc_init(TC0, 0, mode);

	// Configure waveform frequency and duty cycle. 
	uint32_t ra, rc;
	uint32_t rb;
	uint32_t dutycycle = 10; // Duty cycle in percent (positive).
	uint32_t mclk;
	
	uint32_t sck = sysclk_get_peripheral_bus_hz(TC0);
	//uint32_t sck = board_get_cpu_clock_hz();
	uint32_t target_mclk = fs * (uint32_t)df;

//	uint32_t max_rc =  sck / 2 / LTC2512_MIN_MCLK;
//	for (rc = 2; rc < max_rc; rc++) {
//		mclk = sck / 2 / rc;
//		if( mclk <= target_mclk ) break;
//	}

	rc = sck / 2 / target_mclk; // actual mclk
	mclk = sck / 2 / rc; // actual mclk
	
	//mclk = *fs * (uint32_t)df; // target mclk
	//rc = sck / 2 / mclk; //
	//mclk = sck / 2 / rc; // actual mclk
	
	ra = (100 - dutycycle) * rc / 100;
	rb = ra;
	
	// actual sampling freq
	uint32_t act_fs = mclk / (uint32_t)df;
	
	//printf("TC0 configuration: sysclk = %lu, mclk = %lu, ra=%lu, rb=%lu, rc=%lu\n\r", sck, mclk, ra, rb, rc);
	//printf("ltc2512: sclk = %lu\n\r", sck);
	//printf("ltc2512: conversion clock = %lu (fs=%lu)\n\r", mclk, act_fs);

	tc_write_rc(TC0, 0, rc);
	tc_write_ra(TC0, 0, ra);
	if(enable_sw_sync) 	tc_write_rb(TC0, 0, rb);

	/* Enable TC0 Channel 0. */
	//tc_start(TC0, 0);

	return(act_fs);
}

uint32_t ltc2512_trigger(void)
{
	// just lower the sync to start
	// make sure that it's set high by the initialization prior to sync
	ioport_set_pin_level(PIN_ADC_SYNC, 0);

	// set time stamp for the first dma buffer because the ssc endrx interrupt 
	// hasn't happened yet so there'll be no time for the first buffer
	pps_timer_read(&ltc_adc_sec1, &ltc_adc_usec1);
	
	return(ltc_adc_sec1);
}

void ltc2512_pause(void)
{
	ioport_set_pin_level(PIN_ADC_SYNC, 1);
}

void ltc2512_start(void)
{
	tc_start(TC0, 0); // Enable the conversion clock (mck)
	ssc_enable_rx(SSC); // enable ssc receive
	ltc2512_start_dma();
}

void ltc2512_stop(void)
{
	ltc2512_stop_dma();
	tc_stop(TC0, 0); // stop the conversion clock
	ssc_disable_rx(SSC); // disable reciever
	//ioport_set_pin_level(PIN_ADC_SYNC, 1);
}


void ltc2512_shutdown(void)
{
	ltc2512_stop();
	
	// turn off pre-amp
	ioport_set_pin_level(PIN_PREAMP_SHDN, 0);

	// power down adc
	ioport_set_pin_level(PIN_ENABLE_ADC_PWR, 0);

	ioport_set_pin_dir(PIN_ENABLE_ADC_PWR, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(PIN_ADC_SYNC, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(PIN_ADC_SEL0, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(PIN_ADC_SEL1, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(PIN_PREAMP_SHDN, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(PIN_PREAMP_G0, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(PIN_PREAMP_G1, IOPORT_DIR_INPUT);

	ioport_set_pin_dir(PIN_PA20, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(PIN_PA19, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(PIN_PA18, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(PIN_PA17, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(PIN_PA16, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(PIN_PA15, IOPORT_DIR_INPUT);

}

/**
 * \brief DMA driver configuration
 */
uint16_t ltc2512_init_dma(uint16_t nsamps)
{
	if( nsamps > LTC2512_MAX_SAMPLES ) {
		printf("ltc2512_dma_init: %d samples is too large, use %lu max \n\r", nsamps);
		return(-1);
	}
	
	/* Get pointer to UART PDC register base */
	ssc_pdc = ssc_get_pdc_base(SSC);

	/* Initialize PDC data packets for transfer */
	pdc_ssc_packet1.ul_addr = (uint32_t)&ltc_adc_dma_buffer1;
	pdc_ssc_packet1.ul_size = (uint32_t)nsamps;
	pdc_ssc_packet2.ul_addr = (uint32_t)&ltc_adc_dma_buffer2;
	pdc_ssc_packet2.ul_size = (uint32_t)nsamps;

	pdc_active_buffer_number = 1;
	pdc_buffer_locked = 1;
	ltc_adc_buffer = NULL;  // no buffer ready

	/* Configure PDC for data receive */
	pdc_rx_init(ssc_pdc, &pdc_ssc_packet1, &pdc_ssc_packet2);

	//printf("ltc2512_adc_nsamps %u \n\r", ltc_adc_nsamps);
	//fprintf(stdout, "rx_cnt=%ld, rx_ptr=%lx, ", 
	//	pdc_read_rx_counter(ssc_pdc), pdc_read_rx_ptr(ssc_pdc));
	//fprintf(stdout, "rx_next_cnt=%ld, rx_next_ptr=%lx \n\r", 
	//	pdc_read_rx_next_counter(ssc_pdc), pdc_read_rx_next_ptr(ssc_pdc));

	/* Configure the RX End of Reception interrupt. */
	ssc_enable_interrupt(SSC, SSC_IER_ENDRX);

	/* Enable SSC interrupt line from the core */
	NVIC_DisableIRQ(SSC_IRQn);
	NVIC_ClearPendingIRQ(SSC_IRQn);
	NVIC_SetPriority(SSC_IRQn, SSC_ADC_IRQ_PRIO);
	NVIC_EnableIRQ(SSC_IRQn);

	/* Enable PDC receive transfers */
	//pdc_enable_transfer(ssc_pdc, PERIPH_PTCR_RXTEN);

	//ioport_set_pin_dir(SSC_ADC_BUF_PIN, IOPORT_DIR_OUTPUT);
	//ioport_set_pin_level(SSC_ADC_BUF_PIN, 0);
	//return(ltc_adc_nsamps);
	return(pdc_active_buffer_number);
}

/**
 *  \brief Start DMA .
 */
void ltc2512_start_dma(void) 
{
	pdc_active_buffer_number = 1;
	ltc_adc_buffer = NULL;  // no buffer ready

	pdc_buffer_locked = 0;
	
	// enable pdc receiver channel requests
	pdc_enable_transfer(ssc_pdc, PERIPH_PTCR_RXTEN);

	// disable pdc transmitter channel requests
	//pdc_enable_transfer(ssc_pdc, PERIPH_PTCR_TXTDIS);

	//ltc_adc_time1 = RTC->RTC_TIMR;
	//ltc_adc_date1 = RTC->RTC_CALR;
	//ltc_adc_time2 = RTC->RTC_TIMR;
	//ltc_adc_date2 = RTC->RTC_CALR;

}

void ltc2512_stop_dma(void)
{
	// enable pdc receiver channel requests
	pdc_disable_transfer(ssc_pdc, PERIPH_PTCR_RXTEN);

	// disable pdc transmitter channel requests
	//pdc_enable_transfer(ssc_pdc, PERIPH_PTCR_TXTDIS);

}


/**
 * Synchronous Serial Controller IRQ Handler.
 * It must be called SSC_Handle because that is the name defined in the sam4s***.h file
 
 When the current transfer counter reaches zero, the channel checks its next transfer counter. 
 If the value of the next counter is zero, the channel stops transferring data and sets the appropriate flag. 
 If the next counter value is greater than zero, the values of the next pointer/next counter are
 copied into the current pointer/current counter and the channel resumes the transfer, whereas next pointer/next counter
 get zero/zero as values.
 At the end of this transfer, the PDC channel sets the appropriate flags in the Peripheral Status register.
 
 */

//extern uint32_t pps_timer_seconds; // pps counter
//extern uint32_t pps_timer_usecs_per_tick;

void SSC_Handler(void)
{
	volatile uint32_t status = ssc_get_status(SSC);

	// not sure this is necessary and might cause a race condition
	//if ( pdc_buffer_locked ) {
	//	if( ltc_adc_status & (~ADC_DMA_MISSED_BUFFER) ) {
	//		// only print this message once
	//		printf("SSC_Handler: buffer locked\r\n");
	//	}
	//	ltc_adc_status |= ADC_DMA_MISSED_BUFFER;
	//	return;
	//}

	/* Get SSC status and check if the current PDC receive buffer is full */
	// ENDRX flag is set when the PDC Receive Counter register (PERIPH_RCR) reaches zero.
	if ( (status & SSC_SR_ENDRX) == SSC_SR_ENDRX ) {

		// if 
		if(ltc_adc_buffer != NULL) {
			printf("SSC_Handler: buffer overwrite\r\n");
		}

		// get time stamp for the end the buffer that just finished
		uint32_t sec, usec;
		pps_timer_read(&sec, &usec);
	
		// Current DMA buffer is done.
		/* Configure PDC for next data transfer */ 
		// writing to RNCR reset status flag
		// signal that the buffer is ready to be read
		if( (pdc_active_buffer_number == 1)   ) {

			// buffer1 is done, the dma is now reading buffer2, so set buffer1 as next buffer
			pdc_rx_init(ssc_pdc, NULL, &pdc_ssc_packet1);
			
			ltc_adc_buffer = ltc_adc_dma_buffer1; // set buffer 1 as ready to read
			ltc_adc_sec = ltc_adc_sec1;
			ltc_adc_usec = ltc_adc_usec1;

			pdc_active_buffer_number = 2;

			ltc_adc_sec2 = sec;
			ltc_adc_usec2 = usec;
						
			//fprintf(stdout, "SSC_Handler: buffer 1 done, status = %x\r\n", status);
			
			ioport_set_pin_level(PIN_PB11, 1); // debug pin
	
		} else if( pdc_active_buffer_number == 2 ) {

			// buffer2 is done, now reading buffer1, set buffer2 as next
			pdc_rx_init(ssc_pdc, NULL, &pdc_ssc_packet2); // set buffer2 as next rx buffer

			ltc_adc_buffer = ltc_adc_dma_buffer2; // set buffer 2 as ready to read
			ltc_adc_sec = ltc_adc_sec2;
			ltc_adc_usec = ltc_adc_usec2;

			pdc_active_buffer_number = 1;

			ltc_adc_sec1 = sec;
			ltc_adc_usec1 = usec;

			ioport_set_pin_level(PIN_PB11, 0);  // debug pin
			
			//fprintf(stdout, "SSC_Handler: buffer 2 done, status = %x\r\n", status);

		} 
		else {
			fprintf(stdout, "SSC_Handler: confused\r\n");
		}
		
		//fprintf(stdout, "pdc_current_buffer = %d, ", g_pdc_current_buffer);
		//fprintf(stdout, "rx_cnt=%ld, rx_ptr=%ld, ", 
		//	pdc_read_rx_counter(g_p_ssc_pdc), pdc_read_rx_ptr(g_p_ssc_pdc));
		//fprintf(stdout, "rx_next_cnt=%ld, rx_next_ptr=%ld \n\r", 
		//	pdc_read_rx_next_counter(g_p_ssc_pdc), pdc_read_rx_next_ptr(g_p_ssc_pdc));

	}

	// RXBUFF flag is set when both PERIPH_RCR and the PDC Receive Next Counter register (PERIPH_RNCR) reach zero.
	if ((status & SSC_SR_RXBUFF) == SSC_SR_RXBUFF) {
		printf("SSC_Handler: RX buffer\r\n");
	}
	
	if ((status & SSC_SR_OVRUN) == SSC_SR_OVRUN) {
		// buffer overflow
		printf("SSC_Handler: RX buffer overflow\r\n");
		ltc_adc_status |= ADC_DMA_OVERFLOW;
	} else {
		ltc_adc_status &= ~ADC_DMA_OVERFLOW;
	}

}


static inline uint8_t checksum(uint8_t *data, uint32_t len)
{
	uint32_t nbytes = len;
	uint8_t *buf = data;
	uint8_t sum = 0;
	while(nbytes--) sum += *buf++;
	return(sum);
}

uint8_t *ltc2512_get_dma_buffer(void)
{
	// no buffer ready or buffer has already been read
	if(ltc_adc_buffer == NULL) return(NULL);
	
	// set the pointer to the finished buffer
	uint8_t *buffer = ltc_adc_buffer;

	// indicate that the buffer has already been read
	ltc_adc_buffer = NULL;

	// return the buffer pointer
	return(buffer);	
}

//
// Copy finished dma buffer into user buffer and update data header.
// The processor stores the data in memory in little-endian order
// but when a data word is loaded into a register it is big endian.
// Note that printf displays the word as big endian, so be careful with the data word order
// See section 12.4.2.6 in SAM4S Datasheet.
// Here's an example showing this
//	uint8_t buf[4];
//	int *n = (int *)buf;
//	*n = -8;
//  int32_t m = -8;
//	printf("%d %x %02x%02x%02x%02x\r\n", m, m, buf[0], buf[1], buf[2], buf[3]);
//  output is 
//  -8 fffffff8 f8ffffff
//

// copy the most significant first 3 bytes of the dma buffer into the user buffer
//
static inline uint8_t ltc2512_copy_dma_int24(uint8_t *ibuf, uint8_t *obuf, uint16_t nsamps)
{
	// copy 32 bit data word as 24 bit word
	uint8_t chksum = 0;
	uint32_t nbytes = 4 * (uint32_t)nsamps;
	uint32_t m = 0;
	for(uint32_t n = 0; n < nbytes; n += 4) {
		//obuf[m++] = ibuf[n]; // status bit
		obuf[m++] = ibuf[n+1]; // LSB
		obuf[m++] = ibuf[n+2];
		obuf[m++] = ibuf[n+3]; // MSB
		//chksum += ibuf[n+1] + ibuf[n+2] + ibuf[n+3]; // not used and it takes too long
		//if(n < 256) printf("'%02x%02x%02x'; ", ibuf[n+3], ibuf[n+2], ibuf[n+1]);
	}
	//printf("\r\n");
	return(chksum);
}

//
// copy the most significant first 2 bytes of the dma buffer into the user buffer
//
static inline uint8_t ltc2512_copy_dma_int16(uint8_t *ibuf, uint8_t *obuf, uint16_t nsamps)
{
	// copy 32 bit data word as 16 bit word
	uint8_t chksum = 0;
	uint32_t nbytes = nsamps * 4;
	uint32_t m = 0;
	for(uint32_t n = 0; n < nbytes; n += 4) {
		//obuf[m++] = ibuf[n+1]; // LSB
		obuf[m++] = ibuf[n+2]; // 
		obuf[m++] = ibuf[n+3]; // MSB
		//chksum += ibuf[n+2] + ibuf[n+3]; // this takes too long
		//if(n < 32) printf("%x%x ", ibuf[n+3], ibuf[n+2]);
	}
	//printf("\r\n");
	return(chksum);
}

//
// be careful with timing here because the dma buffer gets overwritten.
//
uint16_t ltc2512_read_dma(wispr_data_header_t *hdr, uint8_t *data)
{
	// no buffer ready or buffer has already been read
	if(ltc_adc_buffer == NULL) return(0);

	uint16_t nsamps = ltc_samples_per_buffer;

	// use the latest buffer
	uint8_t *buffer = ltc_adc_buffer;

	// for testing only 
	//if( ltc_dma_test ) buffer = ltc_adc_test_buffer;
	
	uint8_t chksum = 0;
	
	pdc_buffer_locked = 1;
	
	ioport_set_pin_level(PIN_PB10, 1);  // debug signal to check timing
	
	if( ltc_adc_sample_size == 3) {
		chksum = ltc2512_copy_dma_int24(buffer, data, nsamps);
	}
	else if( ltc_adc_sample_size == 2 ) {
		chksum = ltc2512_copy_dma_int16(buffer, data, nsamps);
	}

	pdc_buffer_locked = 0;
	
	ioport_set_pin_level(PIN_PB10, 0);  // debug signal to check timing
	
	// check to see if the buffer pointer has changed during copy
	// this could mean that the data is corrupted.
	uint8_t status = ltc_adc_status;
	//if( !ltc_dma_test && (ltc_adc_buffer != buffer) ) {
	if( ltc_adc_buffer != buffer ) {
		status |= ADC_DMA_BUFFER_OVERRUN; // set the status bit
		printf("ltc2512_read_dma: buffer overrun\r\n");
	} else {
		status &= ~ADC_DMA_BUFFER_OVERRUN; // clear the status bit
	}

	// update the data header info
	//uint8_t year,month,day,hour,minute,sec;
	//uint32_t usec = 0;
	//ltc2512_get_datetime(&year, &month, &day, &hour, &minute, &sec, &usec);
	//hdr->second = time_to_epoch(year, month, day, hour, minute, sec);

	hdr->type = WISPR_WAVEFORM;
	hdr->second = ltc_adc_sec;
	hdr->usec = ltc_adc_usec;
	hdr->data_chksum = chksum; // checksum for data
	hdr->samples_per_buffer = nsamps;
	hdr->sample_size = ltc_adc_sample_size;
	hdr->settings[3] = status;

	// indicate that the dma buffer has been read
	ltc_adc_buffer = NULL;

	return(nsamps);
}


/*
// example of how to copy 24 bit data into a 32 bit signed int
// Note that bit shifting behavior is undefined for signed numbers and
// if the number is shifted more than the size of integer, so cast before shifting.

	if( ltc_adc_sample_size == 3) {
		int m = 0;
		for(int n = 0; n < 16; n++, m++) {
			// load the 24 bit word into a 32 bit word, preserving the sign bit
			uint32_t uv = ((uint32_t)data[3*n+0] << 8) | ((uint32_t)data[3*n+1] << 16) | ((uint32_t)data[3*n+2] << 24);
			int32_t v = (int32_t)uv >> 8; // this shifts back to the first 24 bits and preserves the sign bit
			printf("%d ", v);
		}
		printf("\r\n");
	}
	else if( ltc_adc_sample_size == 2 ) {
		int m = 0;
		for(int n = 0; n < 16; n++, m++) {
			// load the 16 bit word into a 32 bit word
			// the sign bit is preserved as is
			int16_t v = (int16_t)( ((uint16_t)data[2*n+0] << 0) | ((uint16_t)data[2*n+1] << 8) );
			printf("%d ", v);
		}
		printf("\r\n");
	}

example showing how memory is little endian but words are not

	int m, n;
	uint8_t buf[16*4];
	int *n32 = (int *)buf;
	for(n = 0, m = -8; m < 8; m++, n++) {
		n32[n] = m;
		// read the 32 bit word into a 24 bit word preserving the sign bit
		uint32_t temp = ((uint32_t)buf[n*4+0] << 8) | ((uint32_t)buf[n*4+1] << 16) | ((uint32_t)buf[n*4+2] << 24);
		int32_t n24 = (int32_t)temp >> 8;
		printf("%d %08x %02x%02x%02x%02x %d\r\n", n32[n], n32[n], buf[n*4+0], buf[n*4+1], buf[n*4+2], buf[n*4+3], n24);
	}
	
	this is the output:
	
	-8 fffffff8 f8ffffff -8
	-7 fffffff9 f9ffffff -7
	-6 fffffffa faffffff -6
	-5 fffffffb fbffffff -5
	-4 fffffffc fcffffff -4
	-3 fffffffd fdffffff -3
	-2 fffffffe feffffff -2
	-1 ffffffff ffffffff -1
	0 00000000 00000000 0
	1 00000001 01000000 1
	2 00000002 02000000 2
	3 00000003 03000000 3
	4 00000004 04000000 4
	5 00000005 05000000 5
	6 00000006 06000000 6
	7 00000007 07000000 7
	
*/

//
// TESTING ONLY - Simulate a sine wave wave input
// load the test buffer with a 24 bit signed int value to simulate the adc.
// the test buffer will replace the dma buffer in simulation, so it's an int32.
// remember that the buffer data will be formatted as little-endian

//
// Generate a sine wave test signal of amplitude (amp) in volts
// with additive uniform random noise with standard deviation (noise)
//
uint8_t ltc_dma_test = 0;
COMPILER_WORD_ALIGNED uint8_t *ltc_adc_test_buffer;

void ltc2512_init_test(wispr_config_t *wispr, uint16_t nsamps, uint32_t freq, float32_t amp, float32_t stdev)
{
	// allocate test buffer
	uint32_t nbytes = (uint32_t)nsamps * 4;
	ltc_adc_test_buffer = (uint8_t *)malloc(nbytes);
	if(ltc_adc_test_buffer == NULL) {
		printf("\r\nltc_init_test: malloc failed\r\n");
		return;
	}
	
	int32_t *buf = (int32_t *)ltc_adc_test_buffer;
	
	float32_t max_value = 8388607.0 / ADC_SCALING; // 2^23
	float32_t w = 2.0 * PI * (float32_t)freq;
	float32_t dt = 1.0 / (float32_t)wispr->sampling_rate;
	float32_t A = 1.73205080; // sqrt(12/4)
	for(int n = 0; n < nsamps; n++) {
		// uniform random noise with zero mean and variance = sqrt(stdev)
		int rv = rand();
		float32_t noise = stdev * A * (2.0 *((float32_t)rv / (float32_t)RAND_MAX) - 1.0f);
		float32_t t = (float32_t)n * dt;
		float32_t x =  (noise + amp*arm_sin_f32(w*t));
		buf[n] = (int32_t)(max_value * x);
	}
	
	printf("\r\nltc_init_test: test signal will replace adc data.\r\n");
	printf("test_sig = [\r\n");
	for(int n = 0; n < 1024; n++) {
		//printf("%f ", (float32_t)buf[n] / max_value);
		printf("%d ", buf[n]);
	}
	printf("];\r\n");

	ltc_dma_test = 1;
}

//
// Convert the current date and time stamps into readable numbers
//
/* RTC functions */
#define BCD_SHIFT      4
#define BCD_MASK       0xfu
#define BCD_FACTOR     10


void ltc2512_get_datetime(uint8_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *minute, uint8_t *second, uint32_t *usec)
{
	uint32_t date = ltc_adc_date;
	uint32_t time = ltc_adc_time;
	uint32_t temp;

	/* Hour */
	temp = (time & RTC_TIMR_HOUR_Msk) >> RTC_TIMR_HOUR_Pos;
	*hour = (uint8_t)((temp >> BCD_SHIFT) * BCD_FACTOR + (temp & BCD_MASK));

	if ((time & RTC_TIMR_AMPM) == RTC_TIMR_AMPM) {
		*hour += 12;
	}

	/* Minute */
	temp = (time & RTC_TIMR_MIN_Msk) >> RTC_TIMR_MIN_Pos;
	*minute = (uint8_t)((temp >> BCD_SHIFT) * BCD_FACTOR +  (temp & BCD_MASK));

	/* Second */
	temp = (time & RTC_TIMR_SEC_Msk) >> RTC_TIMR_SEC_Pos;
	*second = (uint8_t)((temp >> BCD_SHIFT) * BCD_FACTOR + (temp & BCD_MASK));

	// The TC value is clock ticks since the last updated second
	// using the 32khz slow clock as the clock source.
	// So each tick is 1/32k ~= 30 useconds
	*usec = ltc_adc_tcv * 30;

	/* Retrieve year 0 - 99 */
	temp = (date & RTC_CALR_YEAR_Msk) >> RTC_CALR_YEAR_Pos;
	*year = (uint8_t)((temp >> BCD_SHIFT) * BCD_FACTOR + (temp & BCD_MASK));

	/* Retrieve month */
	temp = (date & RTC_CALR_MONTH_Msk) >> RTC_CALR_MONTH_Pos;
	*month = (uint8_t)((temp >> BCD_SHIFT) * BCD_FACTOR + (temp & BCD_MASK));

	/* Retrieve day */
	temp = (date & RTC_CALR_DATE_Msk) >> RTC_CALR_DATE_Pos;
	*day = (uint8_t)((temp >> BCD_SHIFT) * BCD_FACTOR + (temp & BCD_MASK));

}

