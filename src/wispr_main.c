/**
 *
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include <string.h>
#include <stdio.h>

#include "wispr.h"
#include "board.h"
#include "sd_card.h"
#include "ltc2512.h"
#include "console.h"
#include "rtc_time.h"
#include "uart_queue.h"
#include "gps.h"
#include "i2c.h"
#include "ds3231.h"
#include "ina260.h"
#include "pcf8574.h"

#include "spectrogram.h"
#include "arm_math.h"
#include "arm_const_structs.h"

COMPILER_WORD_ALIGNED uint8_t adc_buffer[ADC_BUFFER_SIZE];
uint8_t *adc_buffer_header = adc_buffer;  // header is at start of buffer
uint8_t *adc_buffer_data = &adc_buffer[ADC_HEADER_SIZE]; // data follows header

wispr_config_t wispr_config;

// active sd card pointer
uint8_t active_sd_card = 0;

uint32_t sd_card_nblocks = 1024; // used for testing

ds3231_datetime  datetime;

// local function prototypes
int swap_sd_cards(void);
void prompt_config(wispr_config_t *hdr, int timeout);
void set_default_config(wispr_config_t *hdr);

//
// main
//
int main (void)
{
	//int result = 0;
	int go = 1;

	sysclk_init();
	board_init();
	i2c_init(TWI0, 100000UL);

	//// Setup the DS3231 External RTC
	if( ds3231_init() != STATUS_OK ) {
		printf("Error initializing DS3231 RTC\r\n");
	}

	// read and display the time set in the external rtc
	ds3231_get_datetime(&datetime);
	printf("\r\nExternal RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n",
	  datetime.year, datetime.month, datetime.day, datetime.hour, datetime.minute, datetime.second);
	
	// Initialize the internal RTC using the external RTC time
	rtc_init((rtc_time_t *)&datetime);
	printf("\r\nInternal RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n",
	datetime.year, datetime.month, datetime.day, datetime.hour, datetime.minute, datetime.second);

	// setup INA260
	uint32_t volts; // mVolts
	int32_t amps; // mAmps
	ina260_init();
	ina260_read_power(&amps, &volts); // first read returns zero 

	// gpio control example
	uint8_t gpio = 0;
	pcf8574_write(gpio);
	gpio = 0xFF;
	pcf8574_write(gpio);
	gpio = 0x0;
	pcf8574_write(gpio);

	/* Initialize SD MMC stack */
	sd_mmc_init();

	// Test all sd card
	// Open, reset and display SD 2
	if(sd_card_open(2, "card2", 1) == SD_MMC_OK) {
		printf("\n\rSD Card 2\n\r");
		sd_card_print_info(2);
	} else {
		printf("SD Card 2 Failed\n\r");
	}
	sd_card_close(2);

	// Open, reset and display SD 1 and leave it active to start writing
	if(sd_card_open(1, "card1", 1) == SD_MMC_OK) {
		active_sd_card = 1;
		printf("\n\rSD Card 1\n\r");
		sd_card_print_info(1);
	} else {
		printf("SD Card 1 Failed\n\r");
	}
	printf("\n\r");
	
	// Read current configuration from sd card
	if( sd_card_read_config(active_sd_card, &wispr_config) != SD_MMC_OK ) {
		set_default_config(&wispr_config);
	}
	
	// Display configuration prompts
	wispr_print_config(&wispr_config);
	
	// Prompt for new configuration
	if( console_prompt_int("Set new configuration?", 0, 10) ) {
		prompt_config(&wispr_config, 60);
		// Display configuration prompts
		wispr_print_config(&wispr_config);
	}

	// initialize ADC - returns the closest sampling rate to the desired rate
	//uint32_t adc_sampling_rate = 50000;
	//uint8_t adc_preamp_gain = 0;

	uint16_t bps = (uint16_t)wispr_config.bytes_per_sample;
	uint16_t samples_per_buffer = (ADC_BUFFER_SIZE - ADC_HEADER_SIZE) / bps;
	uint32_t blocks_per_buffer = ADC_BLOCKS_PER_BUFFER;

	uint32_t fs = wispr_config.sampling_rate;
	uint8_t df = wispr_config.adc_settings[0];
	uint8_t gain = wispr_config.preamp_settings[0];
	ltc2512_init(&fs, df, gain);
	
	// start adc conversions and dma
	ltc2512_init_dma();
	ltc2512_start_dma();
	ltc2512_start_conversion();	

	ina260_read_power(&amps, &volts);
	printf("\r\nina260: mA = %lu, mV = %lu, mW = %lu\r\n", amps, volts, amps*volts/1000);

	// update and save the new config to the sd card
	wispr_config.sampling_rate = fs;
	wispr_config.samples_per_record = samples_per_buffer;
	sd_card_write_config(active_sd_card, &wispr_config);

	printf("\n\rStart read loop\n\r");

	// throw away the first adc buffer
	while( ltc2512_read_dma(adc_buffer_header, adc_buffer_data) == 0 ) {}

	// Initialize spectrum 
	uint16_t nfbins = 128;
	uint16_t overlap = nfbins/2;
	float32_t adc_spectrum_f32[128];	
	spectrum_init_f32(nfbins, HAMMING_WINDOW);
	//q31_t adc_spectrum_q31[128];
	//spectrum_init_q31(nfbins, HAMMING_WINDOW);

	go = 1;
	while (go) {
		
		uint16_t nsamps = ltc2512_read_dma(adc_buffer_header, adc_buffer_data);

		if( nsamps == samples_per_buffer ) {

			// reset the wdt every time a buffer is read
			wdt_restart(WDT);

			// if sd card is full
			if( sd_card_is_full(active_sd_card, blocks_per_buffer) ) {
				swap_sd_cards(); // toggle between the available sd cards
			}

			if( sd_card_write(active_sd_card, adc_buffer, blocks_per_buffer) != SD_MMC_OK ) {
				printf("sd_card_write_raw: failed\n\r");
			}
		
			//spectrum_q31(adc_buffer_data, adc_spectrum_q31, nsamps, overlap);
			spectrum_f32(adc_buffer_data, adc_spectrum_f32, nsamps, overlap);
			
		}
		
		//ina260_read_power(&amps, &volts);
		//printf("ina260: mA = %lu, mV = %lu\r\n", amps, volts);

		// sleep between dma buffers, the next dma interrupt will wake from sleep
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		
		//ina260_read_power(&amps, &volts);
		//printf("ina260: mA = %lu, mV = %lu\r\n", amps, volts);

		// sleep between dma buffers, the next dma interrupt will wake from sleep
		//pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		
	}

	ltc2512_stop_conversion();
	ltc2512_stop_dma();

	printf("Finished\n\r");

	exit(0);
}

//
// swap active sd card
// close active card and open (no reset) the other card
// 
int swap_sd_cards()
{
	if(active_sd_card == 1 ) {
		sd_card_close(1);
		if(sd_card_open(2, "card2", 0) == SD_MMC_OK) {
			active_sd_card = 2;
		} else {
			printf("SD Card2 Failed\n\r");
		}
	} 
	else if(active_sd_card == 2 ) {
		sd_card_close(2);
		if( sd_card_open(1, "card1", 0) == SD_MMC_OK ) {
			active_sd_card = 1;
		} else {
			printf("SD Card1 Failed\n\r");
		}
	} else {
		printf("SD Card Swap Failed\n\r");
	}

	// grow the card size - used for testing	
	uint32_t nblocks = sd_card_get_number_of_blocks(active_sd_card);
	sd_card_set_number_of_blocks(active_sd_card, nblocks+sd_card_nblocks);

	rtc_time_t time;
	rtc_get_datetime(&time);
	printf("SD Card %d Active at ", active_sd_card);
	printf("%02d/%02d/%02d %02d:%02d:%02d\r\n",
		time.year, time.month, time.day, time.hour, time.minute, time.second);

	// update the save config
	sd_card_write_config(active_sd_card, &wispr_config);

	sd_card_print_info(active_sd_card);

	return(active_sd_card);		
}


void prompt_config(wispr_config_t *hdr, int timeout)
{
	hdr->version[0] = WISPR_VERSION;
	hdr->version[1] = WISPR_SUBVERSION;
	
	hdr->bytes_per_sample = console_prompt_uint32("Enter sample size in bytes", hdr->bytes_per_sample, timeout);	
	hdr->sampling_rate = console_prompt_uint32("Enter sampling rate in Hz", hdr->sampling_rate, timeout);
	hdr->preamp_settings[0] = console_prompt_uint8("Enter preamp gain", hdr->preamp_settings[0], timeout);
	hdr->adc_settings[0] = console_prompt_uint8("Enter adc decimation factor", hdr->adc_settings[0], timeout);

	hdr->blocks_per_record = ADC_BLOCKS_PER_BUFFER;
	hdr->samples_per_record = (ADC_BUFFER_SIZE - ADC_HEADER_SIZE) / (uint16_t)hdr->bytes_per_sample;
	//hdr->samples_per_record = ADC_NUM_SAMPLES;

	// prompt to  reset the time
	if( console_prompt_int("Enter new time?", 0, timeout) ) {
		int go = 1;
		while(go) {
			datetime.year = (uint8_t)console_prompt_int("Enter year", (int)datetime.year, timeout);
			datetime.month = (uint8_t)console_prompt_int("Enter month", (int)datetime.month, timeout);
			datetime.day = (uint8_t)console_prompt_int("Enter day", (int)datetime.day, timeout);
			datetime.hour = (uint8_t)console_prompt_int("Enter hour", (int)datetime.hour, timeout);
			datetime.minute = (uint8_t)console_prompt_int("Enter minute", (int)datetime.minute, timeout);
			datetime.second = (uint8_t)console_prompt_int("Enter second", (int)datetime.second, timeout);
			// set the external RTC using
			if( ds3231_set_datetime(&datetime) == TWI_SUCCESS) break;
		}
		ds3231_get_datetime(&datetime);  // read back time
		   printf("\r\nExternal RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n",
		datetime.year, datetime.month, datetime.day, datetime.hour, datetime.minute, datetime.second);
	}
	
	// timestamp the config
	hdr->year = datetime.year;
	hdr->month = datetime.month;
	hdr->day = datetime.day;
	hdr->hour = datetime.hour;
	hdr->minute = datetime.minute;
	hdr->second = datetime.second;
	
	// Prompt for number of blocks to write. This is useful for testing
	sd_card_nblocks = sd_card_get_number_of_blocks(active_sd_card);
	sd_card_nblocks = (uint32_t)console_prompt_int("Enter number of block to write on SD cards", (int)sd_card_nblocks, timeout);
	sd_card_set_number_of_blocks(active_sd_card, sd_card_nblocks);	
}

void set_default_config(wispr_config_t *hdr)
{
	hdr->version[0] = WISPR_VERSION;
	hdr->bytes_per_sample = ADC_SAMPLE_SIZE;
	hdr->samples_per_record = (ADC_BUFFER_SIZE - ADC_HEADER_SIZE) / (uint16_t)hdr->bytes_per_sample;
	//hdr->samples_per_record = ADC_NUM_SAMPLES;
	hdr->sampling_rate = ADC_DEFAULT_SAMPLING_RATE;
	hdr->adc_settings[0] = LTC2512_DF4;
	hdr->adc_settings[1] = 0;
	hdr->preamp_settings[0] = 0;
	hdr->preamp_settings[1] = 0;
	hdr->blocks_per_record = ADC_BLOCKS_PER_BUFFER;
}

