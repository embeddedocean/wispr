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

#define TEST_CARD_SWAP

uint32_t test_sd_card_nblocks = 0; //(ADC_BLOCKS_PER_BUFFER * 100); // TESTING ONLY 

wispr_config_t wispr_config;

// active sd card pointer
uint8_t active_sd_card = 0;

rtc_time_t  datetime;

int measure_power = 0;

// local function prototypes
void go_to_sleep(void);
int swap_sd_cards(void);
void prompt_config(wispr_config_t *hdr, int timeout);
void set_default_config(wispr_config_t *hdr);

void set_default_config(wispr_config_t *hdr)
{
	// set config mod time
	rtc_get_epoch(&wispr_config.epoch);
	//epoch_to_time(wispr_config.epoch, &hdr->year, &hdr->month, &hdr->day, &hdr->hour, &hdr->minute, &hdr->second);
	
	hdr->version[1] = WISPR_VERSION;
	hdr->version[0] = WISPR_SUBVERSION;
	
	hdr->block_size = ADC_BLOCKS_PER_BUFFER * SD_MMC_BLOCK_SIZE;
	hdr->sample_size = ADC_SAMPLE_SIZE;
	hdr->samples_per_block = (ADC_BLOCKS_PER_BUFFER * SD_MMC_BLOCK_SIZE - ADC_HEADER_SIZE) / ADC_SAMPLE_SIZE;
	hdr->sampling_rate = ADC_DEFAULT_SAMPLING_RATE;
	hdr->window = ADC_DEFAULT_WINDOW;
	hdr->interval = ADC_DEFAULT_INTERVAL;

	hdr->settings[7] = LTC2512_DF4; // adc df
	hdr->settings[6] = 0; // adc gain
	hdr->settings[5] = 0;
	hdr->settings[4] = 0;
	hdr->settings[3] = 0;
	hdr->settings[2] = 0;
	hdr->settings[1] = ADC_SAMPLE_SIZE; // sample size
	hdr->settings[0] = 1; // active card

	hdr->state = 0; //
	hdr->mode = WISPR_WAVEFORM; // 

	hdr->active_sd_card = 1;

}

//
// main
//
int main (void)
{
	//int result = 0;
	int go = 1;

	// initialize the board specific function (clocks, gpio, console, wdt, ...)
	// returns the reason the board was last reset (user, sleep, watchdog, ...)
	int reset_type = board_init();
	
	i2c_init(TWI0, 100000UL);
	
	// Read current configuration from general purpose backup registers (gpbr)
	// if config is not valid, set defaults
	if( wispr_read_config(&wispr_config) == 0 ) {
		printf("\r\nInitializing WISPR configuration\r\n");
		set_default_config(&wispr_config);	// set defaults
		wispr_write_config(&wispr_config); // write default configuration
	}
	wispr_print_config(&wispr_config);	// display configs
	
	// Setup the DS3231 External RTC
	if( ds3231_init() != STATUS_OK ) {
		printf("Error initializing DS3231 RTC\r\n");
	}
	
	// read and display the time set in the external rtc
	ds3231_get_datetime(&datetime);
	printf("\r\nExternal RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n",
	  datetime.year, datetime.month, datetime.day, datetime.hour, datetime.minute, datetime.second);
	
	// Initialize the internal RTC using the external RTC time
	rtc_init(&datetime);
	
	printf("Internal RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n",
		datetime.year, datetime.month, datetime.day, datetime.hour, datetime.minute, datetime.second);
	
	// prompt to change config and display card info	
	// skip if reset from backup because there typically will be no user when the system wakes up from backup
	if( reset_type != BOARD_BACKUP_RESET ) {
			
		// Prompt for new configuration
		if( console_prompt_int("Set new configuration?", 0, 10) ) {
			prompt_config(&wispr_config, 60);
			// Display configuration prompts
			wispr_print_config(&wispr_config);
			// set the config time and write it to memory
			rtc_get_epoch(&wispr_config.epoch);
			wispr_write_config(&wispr_config);
		}
				
		// reset the cards to start writing and reading from the beginning 
		// this will overwrite the existing data on the card
		uint8_t reset_sd_card = 0;
		reset_sd_card = (uint8_t)console_prompt_int("Overwrite data on SD cards", (int)reset_sd_card, 10);

		// this is mainly for testing		
		if(reset_sd_card) {
			uint32_t nrecs = 0;
			nrecs = (uint32_t)console_prompt_int("Number of blocks to write on SD cards (0 to use full card)", (int)nrecs, 10);	
			test_sd_card_nblocks = ADC_BLOCKS_PER_BUFFER * nrecs;		
		}
		
		// Initialize all sd card 1 and display info
		if( sd_card_init(1, "card1", reset_sd_card) ) {
			if(test_sd_card_nblocks) sd_card_set_number_of_blocks(1, test_sd_card_nblocks);
			sd_card_write_config(1, &wispr_config); // save the new config to the card
			printf("\n\rSD Card 1\n\r");
			sd_card_print_info(1);
			sd_card_close(1);
		} else {
			printf("SD Card 1 Failed\n\r");
		}
		
		// Initialize all sd card 2 and display info
		if( sd_card_init(2, "card2", reset_sd_card) ) {
			if(test_sd_card_nblocks) sd_card_set_number_of_blocks(2, test_sd_card_nblocks);
			sd_card_write_config(2, &wispr_config); // save the new config to the card
			printf("\n\rSD Card 2\n\r");
			sd_card_print_info(2);
			sd_card_close(2);
		} else {
			printf("SD Card 2 Failed\n\r");
		}
		
		// set card 1 as the active card
		wispr_config.active_sd_card = 1;
		
	}
	
	// set active sd card to what it was before going to sleep
	active_sd_card = wispr_config.active_sd_card;
	
	// Open the active sd card
	if( sd_card_open(active_sd_card) ) {
	} else {
		printf("SD Card %d failed to open\n\r", active_sd_card);
	}
	printf("\n\r");

	// setup INA260
	uint32_t volts; // mVolts
	int32_t amps; // mAmps
//	ina260_init();
//	ina260_read_power(&amps, &volts); // first read returns zero

	// gpio control example
	uint8_t gpio = 0;
//	pcf8574_write(gpio);
	gpio = 0xFF;
//	pcf8574_write(gpio);
	gpio = 0x0;
//	pcf8574_write(gpio);
	
	// initialize ADC - returns the closest sampling rate to the desired rate
	//uint32_t adc_sampling_rate = 50000;
	//uint8_t adc_preamp_gain = 0;
	
	uint16_t adc_bps = (uint16_t)wispr_config.sample_size;
	uint16_t adc_samples_per_buffer = (ADC_BUFFER_SIZE - ADC_HEADER_SIZE) / adc_bps;
	uint32_t adc_blocks_per_buffer = ADC_BLOCKS_PER_BUFFER;
	
	uint32_t fs = wispr_config.sampling_rate;
	uint8_t df = wispr_config.settings[7];
	uint8_t gain = wispr_config.settings[6];
	ltc2512_init(&fs, df, gain);
	
	// start adc conversions and dma
	ltc2512_init_dma();
	ltc2512_start_dma();
	ltc2512_start_conversion();	
	
//	ina260_read_power(&amps, &volts);
//	printf("\r\nina260: mA = %lu, mV = %lu, mW = %lu\r\n", amps, volts, amps*volts/1000);
	
	// update and save the new config to the sd card
	wispr_config.sampling_rate = fs;
	wispr_config.block_size = adc_blocks_per_buffer * SD_MMC_BLOCK_SIZE;
	wispr_config.samples_per_block = adc_samples_per_buffer;
	sd_card_write_config(active_sd_card, &wispr_config);
	
	uint32_t blocks_per_buffer = adc_blocks_per_buffer;
	uint8_t *active_buffer = adc_buffer;
	
		// Initialize spectrum
	uint16_t nfbins = 128;
	uint16_t overlap = nfbins/2;
	float32_t adc_spectrum_f32[128];
	if( wispr_config.mode & WISPR_SPECTRUM ) {
		spectrum_init_f32(nfbins, HAMMING_WINDOW);
		//q31_t adc_spectrum_q31[128];
		//spectrum_init_q31(nfbins, HAMMING_WINDOW);	
	}
	
	float buffer_duration =  (float)wispr_config.samples_per_block / (float)wispr_config.sampling_rate;

	uint32_t samples_per_window = (uint32_t)wispr_config.window * wispr_config.sampling_rate;
	uint16_t buffers_per_window = (uint16_t)(samples_per_window / (uint32_t)wispr_config.samples_per_block);
	uint16_t buffers_per_interval = (uint16_t)(samples_per_window / (uint32_t)wispr_config.samples_per_block);
	uint16_t count = 0;

	uint32_t alarm_seconds = (uint32_t)(wispr_config.interval - wispr_config.window);

	float actual_duration = (float)buffers_per_window * buffer_duration;
	float actual_interval = (float)alarm_seconds + actual_duration;
	
	printf("Actual window: %f\n\r", actual_duration);
	printf("Actual interval: %f\n\r", actual_interval);

	printf("\n\rStart read loop: %d buffers per window\n\r", buffers_per_window);

//	pmc_disable_periph_clk(ID_UART1);
//	pmc_disable_periph_clk(ID_TWI0);

	// throw away the first adc buffer
	while( ltc2512_read_dma(adc_buffer_header, adc_buffer_data) == 0 ) {}

	uint32_t start;
	rtc_get_epoch(&start);
	
	go = 1; 
	while (go) {
		
		if( count < buffers_per_window ) {
		
			uint16_t nsamps = ltc2512_read_dma(adc_buffer_header, adc_buffer_data);
			
			if( nsamps == adc_samples_per_buffer ) {
				
				// reset the wdt every time a buffer is read
				wdt_restart(WDT);
				
				// if sd card is full
				if( sd_card_is_full(active_sd_card, blocks_per_buffer) ) {
					swap_sd_cards(); // toggle between the available sd cards
				}
				
				if( wispr_config.mode & WISPR_SPECTRUM ) {
					
					//spectrum_q31(adc_buffer_data, adc_spectrum_q31, nsamps, overlap);
					spectrum_f32(adc_buffer_data, adc_spectrum_f32, nsamps, overlap);

					if( sd_card_write(active_sd_card, adc_buffer, blocks_per_buffer) == 0 ) {
						printf("sd_card_write: failed\n\r");
					}
					
				}
				
				if( sd_card_write(active_sd_card, active_buffer, blocks_per_buffer) == 0 ) {
					printf("sd_card_write: failed\n\r");
				}
				
				// increment buffer count
				count++;
				
			}
		
			if( measure_power ) {
				ina260_read_power(&amps, &volts);
				printf("ina260: mA = %lu, mV = %lu\r\n", amps, volts);
			}
		
			// sleep between dma buffers, the next dma interrupt will wake from sleep
			pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);

		} else if( alarm_seconds > 0 ) {

			count = 0;

			// set next alarm
			uint32_t now;
			rtc_get_epoch(&now);		
			epoch_to_rtc_time((rtc_time_t *)&datetime, now + alarm_seconds);
			ds3231_set_alarm(&datetime);
			printf("\r\nAlarm set for %s\r\n", epoch_time_string(now + alarm_seconds));				

			go_to_sleep();
			
			// Note: The core will reset when exiting from backup mode. 
			
		}
	
	}
	
	ltc2512_stop_conversion();
	ltc2512_stop_dma();
	
	printf("Finished\n\r");
	
	exit(0);
}


void go_to_sleep(void)
{
	// close the active card
	sd_card_close(active_sd_card);
	sd_card_disable(1);
	sd_card_disable(2);

	// save the latest config to gpbr
	wispr_config.active_sd_card = active_sd_card;
	wispr_write_config(&wispr_config);
				
	ltc2512_shutdown();
	ltc2512_stop_dma();
	
	// enable wakeup input on WKUP2 (PA2) with active low
	supc_set_wakeup_inputs(SUPC, SUPC_WUIR_WKUPEN2_ENABLE, SUPC_WUIR_WKUPT2_LOW);
	
	/* Switch MCK to slow clock  */
	pmc_switch_mck_to_sclk(PMC_MCKR_PRES_CLK_1);
	//pmc_switch_mainck_to_fastrc(CKGR_MOR_MOSCRCF_4_MHz);
	//pmc_switch_mck_to_mainck(PMC_PCK_PRES_CLK_64);

	// Configure all PIOs as inputs to save power
	pio_set_input(PIOA, 0xFFFFFFFF, PIO_PULLUP);
	pio_set_input(PIOB, 0xFFFFFFFF, PIO_PULLUP);
	//pio_set_input(PIOC, 0xFFFFFFFF, PIO_PULLUP);

	/* Disable unused clock to save power */
	pmc_osc_disable_xtal(1);
	pmc_disable_pllack();
	pmc_disable_all_periph_clk();

	// Enter into backup mode
	pmc_enable_backupmode();

}

//
// swap active sd card
// close active card and open (no reset) the other card
// 
int swap_sd_cards()
{
	if( active_sd_card == 1 ) {
		sd_card_close(1);
		active_sd_card = sd_card_open(2);
	} 
	else if( active_sd_card == 2 ) {
		sd_card_close(2);
		active_sd_card = sd_card_open(1);
	} else {
		printf("SD Card Swap Failed\n\r");
	}

	// for testing only
	if( test_sd_card_nblocks ) {
		
		// grow the card size - used for testing
		uint32_t nblocks = sd_card_get_number_of_blocks(active_sd_card);
		sd_card_set_number_of_blocks(active_sd_card, nblocks + test_sd_card_nblocks);

		rtc_time_t time;
		rtc_get_datetime(&time);
		printf("SD Card %d Active at ", active_sd_card);
		printf("%02d/%02d/%02d %02d:%02d:%02d\r\n",
			time.year, time.month, time.day, time.hour, time.minute, time.second);

		sd_card_print_info(active_sd_card);		
	}

	return(active_sd_card);		
}


void prompt_config(wispr_config_t *hdr, int timeout)
{	

	uint16_t blocks_per_buffer = console_prompt_uint32("Enter blocks per buffer", ADC_BLOCKS_PER_BUFFER, timeout);

	hdr->sample_size = console_prompt_uint32("Enter sample size in bytes", hdr->sample_size, timeout);
	hdr->sampling_rate = console_prompt_uint32("Enter sampling rate in Hz", hdr->sampling_rate, timeout);
	hdr->window = console_prompt_uint32("Enter sampling time window in seconds", hdr->window, timeout);
	hdr->interval = console_prompt_uint32("Enter sampling interval in seconds", hdr->interval, timeout);

	hdr->settings[6] = console_prompt_uint8("Enter preamp gain", hdr->settings[6], timeout);
	hdr->settings[7] = console_prompt_uint8("Enter adc decimation factor (4, 8, 16, or 32)", hdr->settings[7], timeout);

	//float buffer_duration =  (float)hdr->samples_per_block / (float)hdr->sampling_rate;
	hdr->block_size = (uint16_t)(blocks_per_buffer * SD_MMC_BLOCK_SIZE);
	hdr->samples_per_block = (hdr->block_size - ADC_HEADER_SIZE) / (uint16_t)hdr->sample_size;
	
	//records_per_window = (float32_t)hdr->window / buffer_duration;

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
			rtc_init((rtc_time_t *)&datetime);
			if( ds3231_set_datetime(&datetime) == TWI_SUCCESS) break;
		}
		ds3231_get_datetime(&datetime);  // read back time
		printf("\r\nExternal RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n",
			datetime.year, datetime.month, datetime.day, datetime.hour, datetime.minute, datetime.second);
	}
	
	// set config mod time
	rtc_get_epoch(&wispr_config.epoch);
	
}


