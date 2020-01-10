/*
 * ltc2512.c
 *
 * Created: 1/2018 
 *  Author: chris
 */ 

//----------------------------------------------------------------------------------------
// 
#include <asf.h>

#include "ltc2512.h"
#include "sd_card.h"
#include "wispr.h"

/* The SSC interrupt IRQ priority. */
#define SSC_ADC_IRQ_PRIO 1

/* DMA channel */
//#define SSC_ADC_DMA_CH 0

/* Pdc transfer buffer */
COMPILER_WORD_ALIGNED uint8_t ssc_adc_dma_buffer1[LTC2512_DMA_BUFFER_NBYTES];
COMPILER_WORD_ALIGNED uint8_t ssc_adc_dma_buffer2[LTC2512_DMA_BUFFER_NBYTES];

/* PDC data packet for transfer */
pdc_packet_t pdc_ssc_packet1;
pdc_packet_t pdc_ssc_packet2;

/* Pointer to SSC PDC register base */
Pdc *ssc_pdc;

/* active buffer number, that is the number o the buffer being read now */
volatile int pdc_active_buffer_number = 1;

// finished buffer
uint8_t *ssc_adc_buffer;
uint16_t ssc_adc_nsamps;
uint32_t ssc_adc_sampling_freq; // Hz

int ssc_adc_buffer_number = 1;
uint32_t ssc_adc_time = 0;
uint32_t ssc_adc_date = 0;

uint32_t ssc_adc_time1 = 0;
uint32_t ssc_adc_date1 = 0;
uint32_t ssc_adc_time2 = 0;
uint32_t ssc_adc_date2 = 0;

uint8_t ltc2512_settings = 0;
uint32_t ltc2512_down_sampling_factor = 4;

wispr_data_header_t wispr;

//
// initialize ltc2512 hardware
//
int ltc2512_init(uint32_t *fs, uint8_t df, uint8_t gain)
{	
	
    if( (*fs <= 1) || (*fs > 400000) ) {
	    fprintf(stdout, "init_ltc2512: invalid sampling frequency\n\r");
	    return(-1);
    }
    
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
		fprintf(stdout, "init_ltc2512: invalid DF\n\r");
		return(-1);
	}

	ltc2512_down_sampling_factor = df;
	
	// set preamp gain
	ioport_set_pin_level(PIN_PREAMP_G0, (gain & 0x01));
	ioport_set_pin_level(PIN_PREAMP_G1, (gain & 0x02));
	//printf("pre-amp gain = 0x%d%d \n\r", (gain & 0x02), (gain & 0x01));
	
	
    /* Initialize the local variable. */
    clock_opt_t rx_clk_option;
    data_frame_opt_t rx_data_frame_option;
    memset((uint8_t *)&rx_clk_option, 0, sizeof(clock_opt_t));
    memset((uint8_t *)&rx_data_frame_option, 0, sizeof(data_frame_opt_t));

    /* Initialize the SSC module */
    pmc_enable_periph_clk(ID_SSC);
    ssc_reset(SSC);

	// set serial clock rate
	uint32_t sclk = sysclk_get_peripheral_bus_hz(TC0);
    uint32_t rsck = LTC2512_SSC_RSCK;  // bit clock freq is fixed to the max reliable rate found by testing
    //uint32_t rsck = df * (*fs) * 40;  // bit clock freq 
    uint32_t rsck_div = sclk / (2 * rsck);
    rsck = sclk / rsck_div /2;

    //printf("sclk = %lu, rsck = %lu, rsck_div = %lu \n\r", sclk, rsck, rsck_div);

    SSC->SSC_CMR = SSC_CMR_DIV(rsck_div);
    
    /* Receiver clock mode configuration. */
    rx_clk_option.ul_cks = SSC_RCMR_CKS_MCK;  // select divided clock source
    rx_clk_option.ul_cko = SSC_RCMR_CKO_TRANSFER; // Receive Clock only during data transfers, RK pin is an output
    //rx_clk_option.ul_cko = SSC_RCMR_CKO_CONTINUOUS; // Receive Clock only during data transfers, RK pin is an output
    //rx_clk_option.ul_cki = 0; // sampled on Receive Clock falling edge.
    rx_clk_option.ul_cki = 1; // sampled on Receive Clock rising edge.
    rx_clk_option.ul_ckg = SSC_RCMR_CKG_EN_RF_LOW; // clock gating selection, enable clock only when RF is low
    //rx_clk_option.ul_start_sel = SSC_RCMR_START_RF_LOW; // Detection of a low level on RF signal
    rx_clk_option.ul_start_sel = SSC_RCMR_START_RF_FALLING; // Detection of a falling edge on RF signal
    //rx_clk_option.ul_start_sel = SSC_RCMR_START_CONTINUOUS; // receive start selection
    rx_clk_option.ul_sttdly = 1; // num clock cycles delay between start event and reception
    rx_clk_option.ul_period = 0; //ul_rfck_div;  // length of frame in clock cycles
    
    /* Receiver frame mode configuration. */
    rx_data_frame_option.ul_datlen = LTC2512_BITS_PER_SAMPLE - 1; // number of bits per data word, should be 0 to 31
    rx_data_frame_option.ul_msbf = SSC_RFMR_MSBF;  // MSB
    rx_data_frame_option.ul_datnb = 0;  //  number of data words per frame, should be 0 to 15. 0 = 1 word
	// 32 bit data word
    rx_data_frame_option.ul_fslen = 15; // Frame Sync. length should be 0 to 15
    rx_data_frame_option.ul_fslen_ext = 1; // Frame Sync. length extension field, should be 0 to 15.
    rx_data_frame_option.ul_fsos = SSC_RFMR_FSOS_NONE; // Frame Sync. is an input
    rx_data_frame_option.ul_fsedge = SSC_RFMR_FSEDGE_NEGATIVE; // Frame Sync. edge detection
    
    /* Configure the SSC receiver. */
    ssc_set_receiver(SSC, &rx_clk_option, &rx_data_frame_option);

    //ssc_enable_tx(SSC);
    ssc_enable_rx(SSC);

    /* TESTING ONLY - Enable the loop mode. */
    //ssc_set_loop_mode(SSC);
	
	ioport_set_pin_level(PIN_ADC_SYNC, 1);
	ioport_set_pin_level(PIN_ADC_SYNC, 0);
	
	int ret = ltc2512_config_mclk(fs);

	// Initialize data header structure
	wispr.version[0] = WISPR_VERSION;
	wispr.version[1] = 0;
	wispr.size = LTC2512_HEADER_NBYTES;
	wispr.settings[0] = (uint8_t)df;
	wispr.settings[1] = (uint8_t)gain;
	wispr.bytes_per_sample = LTC2512_BYTES_PER_SAMPLE;
	wispr.num_samples = LTC2512_NUM_SAMPLES;
		
	return(ret);

}

//
// Configure TC0_TIOA to supply MCLK for the ADC
// Configure TC0_TIOB to supply SYNC for the switch regulators
//  
int ltc2512_config_mclk(uint32_t *fs)
{
	// Configure TC TC_CHANNEL_WAVEFORM in waveform operating mode.
	//see: http://www.allaboutcircuits.com/projects/dma-digital-to-analog-conversion-with-a-sam4s-microcontroller-the-timer-cou/

	uint32_t ra, rb, rc, sck;
	uint32_t dutycycle = 10; /** Duty cycle in percent (positive).*/
	uint32_t mclk;

	/* Configure PIO Pins for TC */
	ioport_set_pin_mode(PIN_TC0_TIOA0, PIN_TC0_TIOA0_MUX);

	/* Disable IO to enable peripheral mode) */
	ioport_disable_pin(PIN_TC0_TIOA0);

	// configure PA1 as either a clock or gpio	
	//ioport_set_pin_mode(PIN_TC0_TIOB0, PIN_TC0_TIOB0_MUX);
	//ioport_disable_pin(PIN_TC0_TIOB0);
	ioport_set_pin_dir(PIN_TC0_TIOB0, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_TC0_TIOB0, 0);

	/* Configure the PMC to enable the TC module. */
	sysclk_enable_peripheral_clock(ID_TC0);

	/* Init TC to waveform mode. */
	tc_init(TC0, 0,
	  TC_CMR_TCCLKS_TIMER_CLOCK1	//MCK/2
	  | TC_CMR_WAVE	//waveform mode
	  | TC_CMR_WAVSEL_UP_RC	//UP mode with automatic trigger on RC Compare
	  | TC_CMR_ACPA_TOGGLE	//toggle TIOA on RA match
	  | TC_CMR_ACPC_TOGGLE	//toggle TIOA on RC match
	  | TC_CMR_BCPB_TOGGLE	//toggle TIOB on RB match
	  | TC_CMR_BCPC_TOGGLE	//toggle TIOB on RC match
	  // for TIOB to configured as an output an external event needs to be selected
	  | TC_CMR_EEVT_XC0 | TC_CMR_AEEVT_NONE | TC_CMR_BEEVT_NONE | TC_CMR_EEVTEDG_NONE
	);
	
	/* Configure waveform frequency and duty cycle. */
	sck = sysclk_get_peripheral_bus_hz(TC0);
	uint32_t target_mclk = (*fs) * ltc2512_down_sampling_factor;
	uint32_t max_rc =  sck / 2 / LTC2512_MIN_MCLK;
	for (rc = 2; rc < max_rc; rc++) {
		mclk = sck / 2 / rc;
		if( mclk <= target_mclk ) break;		
	}
	//rc =  1 + sck / 2 / mclk;
	//if(rc < 2) rc = 2;
	//mclk = sck / 2 / rc; // actual mclk
	//mclk = 1600000;
	//rc =  1 + sck / 2 / mclk;
	//if(rc < 2) rc = 2;
	mclk = sck / 2 / rc; // actual mclk
	ra = (100 - dutycycle) * rc / 100;
	rb = ra;
	
	//printf("TC0 configuration: sysclk = %lu, mclk = %lu, ra=%lu, rb=%lu, rc=%lu\n\r", sck, mclk, ra, rb, rc);

	tc_write_rc(TC0, 0, rc);
	tc_write_ra(TC0, 0, ra);
	tc_write_rb(TC0, 0, rb);

	// actual sampling freq
	*fs = mclk / ltc2512_down_sampling_factor;
	ssc_adc_sampling_freq = *fs;

	return(1);
}

void ltc2512_start_conversion(void)
{
	ioport_set_pin_level(PIN_ADC_SYNC, 0);
	/* Enable TC0 Channel 0. */
	tc_start(TC0, 0);
}

void ltc2512_stop_conversion(void)
{
	/* Disable TC0 Channel 0. */
	tc_stop(TC0, 0);
}

void ltc2512_shutdown(void)
{
	ltc2512_stop_conversion();
	
	// turn off pre-amp
	ioport_set_pin_level(PIN_PREAMP_SHDN, 0);

	// power down adc
	ioport_set_pin_level(PIN_ENABLE_ADC_PWR, 0);
}

//
// Convert the current date and time stamps into readable numbers
//
/* RTC functions */
#define BCD_SHIFT      4
#define BCD_MASK       0xfu
#define BCD_FACTOR     10

void ltc2512_get_time(uint8_t *hour, uint8_t *minute, uint8_t *second, uint32_t *usec)
{
	uint32_t time = ssc_adc_time;
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

	// temporary !!!
	*usec = 0;
}

void ltc2512_get_date(uint8_t *cent, uint8_t *year, uint8_t *month, uint8_t *day, uint8_t *week)
{
	uint32_t date = ssc_adc_date;
	uint32_t temp;

	/* Retrieve century */
	if (year) {
		temp = (date & RTC_CALR_CENT_Msk) >> RTC_CALR_CENT_Pos;
		*cent = (uint8_t)((temp >> BCD_SHIFT) * BCD_FACTOR + (temp & BCD_MASK));
	}

	/* Retrieve year 0 - 99 */
	if (year) {
		temp = (date & RTC_CALR_YEAR_Msk) >> RTC_CALR_YEAR_Pos;
		//*year = (uint16_t)((cent * BCD_FACTOR * BCD_FACTOR) + (temp >> BCD_SHIFT) * BCD_FACTOR + (temp & BCD_MASK));
		*year = (uint8_t)((temp >> BCD_SHIFT) * BCD_FACTOR + (temp & BCD_MASK));
	}

	/* Retrieve month */
	if (month) {
		temp = (date & RTC_CALR_MONTH_Msk) >> RTC_CALR_MONTH_Pos;
		*month = (uint8_t)((temp >> BCD_SHIFT) * BCD_FACTOR + (temp & BCD_MASK));
	}

	/* Retrieve day */
	if (day) {
		temp = (date & RTC_CALR_DATE_Msk) >> RTC_CALR_DATE_Pos;
		*day = (uint8_t)((temp >> BCD_SHIFT) * BCD_FACTOR + (temp & BCD_MASK));
	}

	/* Retrieve week */
	if (week) {
		*week = (uint8_t)(((date & RTC_CALR_DAY_Msk) >> RTC_CALR_DAY_Pos));
	}
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
void SSC_Handler(void)
{
	uint32_t status;
	
	status = ssc_get_status(SSC);
	
	/* Get SSC status and check if the current PDC receive buffer is full */
	// ENDRX flag is set when the PDC Receive Counter register (PERIPH_RCR) reaches zero.
	if ((status & SSC_SR_ENDRX) == SSC_SR_ENDRX) {
		// Current DMA buffer is done.
		/* Configure PDC for next data transfer */ 
		// writing to RNCR reset status flag
		// signal that the buffer is ready to be read
		if(pdc_active_buffer_number == 1) {
			// buffer1 is done, the dma is now reading buffer2, so set buffer1 as next buffer
			ssc_adc_time2 = RTC->RTC_TIMR;
			ssc_adc_date2 = RTC->RTC_CALR;
			pdc_rx_init(ssc_pdc, NULL, &pdc_ssc_packet1);
			pdc_active_buffer_number = 2;
			ssc_adc_buffer = ssc_adc_dma_buffer1;
			ssc_adc_time = ssc_adc_time1;
			ssc_adc_date = ssc_adc_date1;
			//fprintf(stdout, "SSC_Handler: buffer 1 done, status = %x\r\n", status);
			//ioport_set_pin_level(SSC_ADC_BUF_PIN, 0);
		} else if(pdc_active_buffer_number == 2) {
			// buffer2 is done, now reading buffer1, set buffer2 as next
			ssc_adc_time1 = RTC->RTC_TIMR;
			ssc_adc_date1 = RTC->RTC_CALR;
			pdc_rx_init(ssc_pdc, NULL, &pdc_ssc_packet2);
			pdc_active_buffer_number = 1;
			ssc_adc_buffer = ssc_adc_dma_buffer2;
			ssc_adc_time = ssc_adc_time2;
			ssc_adc_date = ssc_adc_date2;
			//ioport_set_pin_level(SSC_ADC_BUF_PIN, 1);
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
		fprintf(stdout, "SSC_Handler: buffer overflow\r\n");
		// buffer overflow
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

void ltc2512_update_header(uint8_t *hdr, uint8_t chksum)
{
	// update the data header info
	uint8_t year, month,day,hour,minute,sec;
	uint32_t usec = 0;

	ltc2512_get_time(&hour, &minute, &sec, &usec);
	ltc2512_get_date(NULL, &year, &month, &day, NULL);

	wispr.sampling_rate = ssc_adc_sampling_freq;
	wispr.year = year;
	wispr.month = month;
	wispr.day = day;
	wispr.hour = hour;
	wispr.minute = minute;
	wispr.second = sec;
	wispr.usec = usec;
	wispr.data_chksum = chksum; // checksum for data

	wispr_serialize_data_header(&wispr, hdr);

}

uint8_t *ltc2512_get_dma_buffer(void)
{
	// no buffer ready or buffer has already been read
	if(ssc_adc_buffer == NULL) return(NULL);
	
	// set the pointer to the finished buffer
	uint8_t *buffer = ssc_adc_buffer;

	// indicate that the buffer has already been read
	ssc_adc_buffer = NULL;

	// update the data header info
	//ltc2512_update_header(hdr, chksum);
	
	// return the buffer pointer
	return(buffer);	
}

//
// copy finished dma buffer into user buffer and update data header.
// flags are anything extra that you want to put in the header.
// be careful with timing here because the dma buffer gets overwritten.
//
uint16_t ltc2512_read_dma(uint8_t *hdr, uint8_t *data)
{
	// no buffer ready or buffer has already been read
	if(ssc_adc_buffer == NULL) return(0);
	
	// save 32 bit data word as 24 bit word
	uint8_t *obuf = data;  // output buffer
	uint8_t *ibuf = ssc_adc_buffer;  // finished dma buffer
	uint8_t chksum = 0;
	uint32_t nbytes = LTC2512_NUM_SAMPLES * 4;
	uint32_t m = 0;
	for(uint32_t n = 0; n < nbytes; n += 4) {
		for (uint32_t k = 0; k < LTC2512_BYTES_PER_SAMPLE; k++) {
			obuf[m++] = ibuf[n+k];
			chksum += ibuf[n+k];
		}
	}

	//data = ssc_adc_buffer;
	
	// indicate that the dma buffer has been read
	ssc_adc_buffer = NULL;
	
	// update the data header info
	ltc2512_update_header(hdr, chksum);
	
	// return the buffer pointer
	return(LTC2512_NUM_SAMPLES);
}

uint16_t ltc2512_read_dma_int24(uint8_t *hdr, uint8_t *data)
{
	// no buffer ready or buffer has already been read
	if(ssc_adc_buffer == NULL) return(0);
	
	// save 32 bit data word as 24 bit word
	uint8_t *obuf = data;  // output buffer
	uint8_t *ibuf = ssc_adc_buffer;  // finished dma buffer
	uint8_t chksum = 0;
	uint32_t nbytes = LTC2512_NUM_SAMPLES * 4;
	uint32_t m = 0;
	for(uint32_t n = 0; n < nbytes; n += 4) {
		obuf[m++] = ibuf[n]; 
		obuf[m++] = ibuf[n+1];
		obuf[m++] = ibuf[n+2];
		chksum = ibuf[n] + ibuf[n+1] + ibuf[n+2];
		//for (uint32_t k = 0; k < LTC2512_BYTES_PER_SAMPLE; k++) {
		//	obuf[m++] = ibuf[n+k];
		//	chksum += ibuf[n+k];
		//}
	}

	//data = ssc_adc_buffer;
	
	// indicate that the dma buffer has been read
	ssc_adc_buffer = NULL;
	
	// update the data header info
	ltc2512_update_header(hdr, chksum);
	
	// return the buffer pointer
	return(LTC2512_NUM_SAMPLES);
}

uint16_t ltc2512_read_dma_int16(uint8_t *hdr, uint8_t *data)
{
	// no buffer ready or buffer has already been read
	if(ssc_adc_buffer == NULL) return(0);
	
	// save 32 bit data word as 24 bit word
	uint8_t *obuf = data;  // output buffer
	uint8_t *ibuf = ssc_adc_buffer;  // finished dma buffer
	uint8_t chksum = 0;
	uint32_t nbytes = LTC2512_NUM_SAMPLES * 4;
	uint32_t m = 0;
	for(uint32_t n = 0; n < nbytes; n += 4) {
		obuf[m++] = ibuf[n+1];
		obuf[m++] = ibuf[n+2];
		chksum = ibuf[n+1] + ibuf[n+2];
	}

	//??? data = ssc_adc_buffer;
	
	// indicate that the dma buffer has been read
	ssc_adc_buffer = NULL;
	
	// update the data header info
	ltc2512_update_header(hdr, chksum);
	
	// return the buffer pointer
	return(LTC2512_NUM_SAMPLES);
}



/*
// example of how to copy 24 bit data into a 32 bit signed int
uint32_t *buf = (uint32_t *)adc_buffer_data;
for (m = 0; m < LTC2512_NUM_SAMPLES; m++) {
	// load the 24 bit word into a 32 bit word, preserving the sign bit
	uint32_t uv = ((uint32_t)adc_buffer_data[3*m+0] << 8) | ((uint32_t)adc_buffer_data[3*m+1] << 16) | ((uint32_t)adc_buffer_data[3*m+2] << 24);
	int32_t v = (int32_t)uv >> 8;
}
*/


/**
 * \brief DMA driver configuration
 */
uint16_t ltc2512_init_dma()
{
	//if( nsamps > LTC2512_MAX_DMA_BUFFER_NSAMPS ) {
	//	printf("ltc2512_dma_init: error, nsamps (%d) too large, use %lu max \n\r", nsamps, LTC2512_MAX_DMA_BUFFER_NSAMPS);
	//	return(-1);
	//}
	
	/* Get pointer to UART PDC register base */
	ssc_pdc = ssc_get_pdc_base(SSC);

	/* Initialize PDC data packets for transfer */
	pdc_ssc_packet1.ul_addr = (uint32_t)&ssc_adc_dma_buffer1;
	pdc_ssc_packet1.ul_size = (uint32_t)LTC2512_NUM_SAMPLES;
	pdc_ssc_packet2.ul_addr = (uint32_t)&ssc_adc_dma_buffer2;
	pdc_ssc_packet2.ul_size = (uint32_t)LTC2512_NUM_SAMPLES;

	pdc_active_buffer_number = 1;
	ssc_adc_buffer = NULL;  // no buffer ready
	ssc_adc_nsamps = LTC2512_NUM_SAMPLES;
	
	/* Configure PDC for data receive */
	pdc_rx_init(ssc_pdc, &pdc_ssc_packet1, &pdc_ssc_packet2);

	//printf("ltc2512_adc_nsamps %u \n\r", ssc_adc_nsamps);
	
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
	pdc_enable_transfer(ssc_pdc, PERIPH_PTCR_RXTEN);

	//ioport_set_pin_dir(SSC_ADC_BUF_PIN, IOPORT_DIR_OUTPUT);
	//ioport_set_pin_level(SSC_ADC_BUF_PIN, 0);
	//return(ssc_adc_nsamps);
	return(pdc_active_buffer_number);
}

/**
 *  \brief Start DMA .
 */
void ltc2512_start_dma(void) 
{
	pdc_active_buffer_number = 1;
	ssc_adc_buffer = NULL;  // no buffer ready
	ssc_adc_nsamps = LTC2512_NUM_SAMPLES;

	// enable pdc receiver channel requests
	pdc_enable_transfer(ssc_pdc, PERIPH_PTCR_RXTEN);

	// disable pdc transmitter channel requests
	//pdc_enable_transfer(ssc_pdc, PERIPH_PTCR_TXTDIS);

	ssc_enable_rx(SSC);
	//ssc_enable_tx(SSC);
	
	ssc_adc_time1 = RTC->RTC_TIMR;
	ssc_adc_date1 = RTC->RTC_CALR;
	ssc_adc_time2 = RTC->RTC_TIMR;
	ssc_adc_date2 = RTC->RTC_CALR;

}

void ltc2512_stop_dma(void)
{
	// enable pdc receiver channel requests
	pdc_disable_transfer(ssc_pdc, PERIPH_PTCR_RXTEN);

	// disable pdc transmitter channel requests
	//pdc_enable_transfer(ssc_pdc, PERIPH_PTCR_TXTDIS);

	ssc_disable_rx(SSC);
	//ssc_enable_tx(SSC);
}


/*  Experimental 

uint16_t ltc2512_init_dma2()
{
	//if( nsamps > LTC2512_MAX_DMA_BUFFER_NSAMPS ) {
	//	printf("ltc2512_dma_init: error, nsamps (%d) too large, use %lu max \n\r", nsamps, LTC2512_MAX_DMA_BUFFER_NSAMPS);
	//	return(-1);
	//}
	
	// Get pointer to UART PDC register base
	ssc_pdc = ssc_get_pdc_base(SSC);

	// Initialize PDC data packets for transfer
	pdc_ssc_packet1.ul_addr = (uint32_t)&ssc_adc_dma_buffer1;
	pdc_ssc_packet1.ul_size = (uint32_t)(LTC2512_NUM_SAMPLES * LTC2512_BYTES_PER_SAMPLE);
	pdc_ssc_packet2.ul_addr = (uint32_t)&ssc_adc_dma_buffer2;
	pdc_ssc_packet2.ul_size = (uint32_t)(LTC2512_NUM_SAMPLES * LTC2512_BYTES_PER_SAMPLE);

	pdc_active_buffer_number = 1;
	ssc_adc_buffer = NULL;  // no buffer ready
	ssc_adc_nsamps = LTC2512_NUM_SAMPLES;
	
	// Configure PDC for data receive
	pdc_rx_init(ssc_pdc, &pdc_ssc_packet1, &pdc_ssc_packet2);

	printf("ltc2512_adc_nsamps %u \n\r", ssc_adc_nsamps);
	
//	fprintf(stdout, "rx_cnt=%ld, rx_ptr=%lx, ", 
//		pdc_read_rx_counter(ssc_pdc), pdc_read_rx_ptr(ssc_pdc));
//	fprintf(stdout, "rx_next_cnt=%ld, rx_next_ptr=%lx \n\r", 
//		pdc_read_rx_next_counter(ssc_pdc), pdc_read_rx_next_ptr(ssc_pdc));

	// Configure the RX End of Reception interrupt.
	ssc_enable_interrupt(SSC, SSC_IER_ENDRX);

	// Enable SSC interrupt line from the core
	NVIC_DisableIRQ(SSC_IRQn);
	NVIC_ClearPendingIRQ(SSC_IRQn);
	NVIC_SetPriority(SSC_IRQn, SSC_ADC_IRQ_PRIO);
	NVIC_EnableIRQ(SSC_IRQn);

	// Enable PDC receive transfers
	pdc_enable_transfer(ssc_pdc, PERIPH_PTCR_RXTEN);

	//ioport_set_pin_dir(SSC_ADC_BUF_PIN, IOPORT_DIR_OUTPUT);
	//ioport_set_pin_level(SSC_ADC_BUF_PIN, 0);
	return(ssc_adc_nsamps);
}


int ltc2512_init2(uint32_t *fs, uint32_t df)
{
	
	//    if( (*fs <= 1) || (*fs > 400000) ) {
	//	    fprintf(stdout, "init_ltc2380: invalid sampling frequency\n\r");
	//	    return(-1);
	//   }
	
	ltc2512_down_sampling_factor = df; // this is configured in hardware
	
	// power up and initialize adc
	ioport_set_pin_level(PIN_ENABLE_ADC_PWR, 1);
	ioport_set_pin_level(PIN_ADC_SYNC, 1);
	delay_ms(100);

	// Initialize the local variable.
	clock_opt_t rx_clk_option;
	data_frame_opt_t rx_data_frame_option;
	memset((uint8_t *)&rx_clk_option, 0, sizeof(clock_opt_t));
	memset((uint8_t *)&rx_data_frame_option, 0, sizeof(data_frame_opt_t));

	// Initialize the SSC module
	pmc_enable_periph_clk(ID_SSC);
	ssc_reset(SSC);

	uint32_t sclk = sysclk_get_peripheral_bus_hz(TC0);
	uint32_t rsck = 20000000;  // bit clock freq is fixed to the max reliable rate found by testing
	//uint32_t rsck = df * (*fs) * 40;  // bit clock freq is fixed to the max reliable rate found by testing
	uint32_t rsck_div = sclk / (2 * rsck);
	rsck = sclk / rsck_div /2;

	printf("sclk = %lu, rsck = %lu, rsck_div = %lu \n\r", sclk, rsck, rsck_div);

	SSC->SSC_CMR = SSC_CMR_DIV(rsck_div);
	
	// Receiver clock mode configuration.
	rx_clk_option.ul_cks = SSC_RCMR_CKS_MCK;  // select divided clock source
	rx_clk_option.ul_cko = SSC_RCMR_CKO_TRANSFER; // Receive Clock only during data transfers, RK pin is an output
	//rx_clk_option.ul_cko = SSC_RCMR_CKO_CONTINUOUS; // Receive Clock only during data transfers, RK pin is an output
	//rx_clk_option.ul_cki = 0; // sampled on Receive Clock falling edge.
	rx_clk_option.ul_cki = 1; // sampled on Receive Clock rising edge.
	rx_clk_option.ul_ckg = SSC_RCMR_CKG_EN_RF_LOW; // clock gating selection, enable clock only when RF is low
	//rx_clk_option.ul_start_sel = SSC_RCMR_START_RF_LOW; // Detection of a low level on RF signal
	rx_clk_option.ul_start_sel = SSC_RCMR_START_RF_FALLING; // Detection of a falling edge on RF signal
	//rx_clk_option.ul_start_sel = SSC_RCMR_START_CONTINUOUS; // receive start selection
	rx_clk_option.ul_sttdly = 1; // num clock cycles delay between start event and reception
	rx_clk_option.ul_period = 0; //ul_rfck_div;  // length of frame in clock cycles
	
	// Receiver frame mode configuration. 
	rx_data_frame_option.ul_datlen = 7; //LTC2512_BITS_PER_SAMPLE - 1; // number of bits per data word
	rx_data_frame_option.ul_msbf = SSC_RFMR_MSBF;  // MSB
	rx_data_frame_option.ul_datnb = 0;  //  number of data words per frame, should be 0 to 15. 0 = 1 word
	rx_data_frame_option.ul_fsos = SSC_RFMR_FSOS_NONE; // Frame Sync. is an input
	rx_data_frame_option.ul_fsedge = SSC_RFMR_FSEDGE_NEGATIVE; // Frame Sync. edge detection
	
	// Configure the SSC receiver.
	ssc_set_receiver(SSC, &rx_clk_option, &rx_data_frame_option);

	//ssc_enable_tx(SSC);
	ssc_enable_rx(SSC);

	//ssc_set_loop_mode(SSC);
	
	int ret = 0;
	
	ret = ltc2512_config_mclk(fs);
	
	return(ret);

}
*/
