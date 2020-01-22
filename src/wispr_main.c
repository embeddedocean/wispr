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

//COMPILER_WORD_ALIGNED uint8_t adc_test_buffer[ADC_MAX_BUFFER_SIZE];
//uint8_t *adc_test_header = adc_test_buffer;  // header is at start of buffer
//uint8_t *adc_test_data = &adc_test_buffer[WISPR_DATA_HEADER_SIZE]; // data follows header

COMPILER_WORD_ALIGNED uint8_t adc_buffer[ADC_MAX_BUFFER_SIZE];
uint8_t *adc_buffer_header = adc_buffer;  // header is at start of buffer
uint8_t *adc_buffer_data = &adc_buffer[WISPR_DATA_HEADER_SIZE]; // data follows header

COMPILER_WORD_ALIGNED uint8_t psd_buffer[PSD_MAX_BUFFER_SIZE];
uint8_t *psd_buffer_header = psd_buffer;  // header is at start of buffer
uint8_t *psd_buffer_data = &psd_buffer[WISPR_DATA_HEADER_SIZE]; // data follows header

uint16_t psd_nfft;
uint16_t psd_nbins;
uint16_t psd_overlap;
uint8_t psd_sample_size;

#define TEST_CARD_SWAP

uint32_t test_sd_card_nblocks = 0; //(ADC_BLOCKS_PER_BUFFER * 100); // TESTING ONLY 

wispr_config_t wispr_config;

// active sd card pointer
uint8_t active_sd_card = 1;

rtc_time_t  datetime;

int measure_power = 0;

// local function prototypes
void go_to_sleep(void);
int swap_sd_cards(void);
void prompt_config_menu(wispr_config_t *hdr, int timeout);
void set_default_config(wispr_config_t *hdr);

void set_default_config(wispr_config_t *hdr)
{
	// set config mod time
	rtc_get_epoch(&wispr_config.epoch);
	//epoch_to_time(wispr_config.epoch, &hdr->year, &hdr->month, &hdr->day, &hdr->hour, &hdr->minute, &hdr->second);
	
	hdr->version[1] = WISPR_VERSION;
	hdr->version[0] = WISPR_SUBVERSION;
	
	hdr->block_size = ADC_MAX_BLOCKS_PER_BUFFER * SD_MMC_BLOCK_SIZE;
	hdr->sample_size = 3;
	hdr->samples_per_block = (hdr->block_size - WISPR_DATA_HEADER_SIZE) / 3;
	hdr->sampling_rate = ADC_DEFAULT_SAMPLING_RATE;
	hdr->window = ADC_DEFAULT_WINDOW;
	hdr->interval = ADC_DEFAULT_INTERVAL;

	hdr->settings[7] = LTC2512_DF4; // adc df
	hdr->settings[6] = 0; // adc gain
	hdr->settings[5] = 0;
	hdr->settings[4] = 0;
	hdr->settings[3] = 0;
	hdr->settings[2] = 0;
	hdr->settings[1] = hdr->sample_size; // sample size
	hdr->settings[0] = 1; // active card

	hdr->state = 0; //
	hdr->mode = WISPR_WAVEFORM; // 

	hdr->fft_size = 512;
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
	if( reset_type == BOARD_BACKUP_RESET ) {
		if(wispr_gpbr_read_config(&wispr_config) == 0 ) {
			// if gpbr config is not valid - something whent wrong
			printf("\r\nWARNING - Initializing WISPR configuration coming out of backup reset.\r\n");
			set_default_config(&wispr_config);	// set defaults
		}
		// recover active sd card number 
		active_sd_card = wispr_config.active_sd_card;
	}
	
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
	// skip if reset from backup because there typically will be no user at the console
	// this means that the configuration can only be changed after a user reset or powerup
	if( reset_type != BOARD_BACKUP_RESET ) {
			
		// read the last config from the cards
		if( sd_card_open(1) ) {
			sd_card_read_config(1, &wispr_config);
			sd_card_close(1);
		} else 	if( sd_card_open(2) ) {
			sd_card_read_config(2, &wispr_config);
			sd_card_close(2);
		} else {
			printf("Failed to read configuration from either SD card\n\r");
			set_default_config(&wispr_config);
		}
		
		// display the config
		wispr_print_config(&wispr_config);
		
		// Prompt to set new configuration
		if( console_prompt_int("Changed configuration", 0, 8) ) {
			prompt_config_menu(&wispr_config, 60);		
		}
		
		// save the new config for backup reset
		wispr_gpbr_write_config(&wispr_config);

		// Prompt to reset the cards to start writing and reading from the beginning
		// this will overwrite the existing data on the card
		uint8_t reset_sd_card = 0;
		reset_sd_card = (uint8_t)console_prompt_int("Overwrite data on SD cards", (int)reset_sd_card, 6);

		// this is mainly for testing
		if(reset_sd_card) {
			uint32_t nrecs = 0;
			nrecs = (uint32_t)console_prompt_int("Number of data blocks to write on SD cards (0 to use full card)", (int)nrecs, 8);
			test_sd_card_nblocks = (uint32_t)wispr_config.block_size * nrecs / SD_MMC_BLOCK_SIZE;
		}
		
		// Initialize sd card 1, display card info, and write the latest config
		if( sd_card_init(1, "card1", reset_sd_card) ) {
			if(test_sd_card_nblocks != 0) sd_card_set_number_of_blocks(1, test_sd_card_nblocks);
			sd_card_print_info(1);
			// save the new config to this card
			sd_card_write_config(1, &wispr_config);
			sd_card_close(1);
		} else {
			printf("SD Card 1 Failed\n\r");
		}
		
		// Initialize sd card 2, display card info, and write the latest config
		if( sd_card_init(2, "card2", reset_sd_card) ) {
			if(test_sd_card_nblocks != 0) sd_card_set_number_of_blocks(2, test_sd_card_nblocks);
			sd_card_print_info(2);
			// save the new config to this card also
			sd_card_write_config(2, &wispr_config);
			sd_card_close(2);
		} else {
			printf("SD Card 2 Failed\n\r");
		}

		// always set active card to 1 after non-backup reset
		wispr_config.active_sd_card = 1;

	}
	
	wispr_print_config(&wispr_config);
	
	// set active sd card number 
	active_sd_card = wispr_config.active_sd_card;
	
	// Open the active sd card
	if( sd_card_open(active_sd_card) == 0 ) {
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
	
	//uint16_t adc_bps = (uint16_t)wispr_config.sample_size;
	//uint32_t adc_blocks_per_buffer = ADC_MAX_BLOCKS_PER_BUFFER;
	//wispr_config.block_size = adc_blocks_per_buffer * SD_MMC_BLOCK_SIZE;
	//wispr_config.samples_per_block = adc_samples_per_buffer;
	
	// initialize the adc with the current config 
	ltc2512_init(&wispr_config);
	
	// initialize the adc dma
	uint16_t samples_per_adc_block = wispr_config.samples_per_block;
	ltc2512_init_dma(samples_per_adc_block);
	
//	ina260_read_power(&amps, &volts);
//	printf("\r\nina260: mA = %lu, mV = %lu, mW = %lu\r\n", amps, volts, amps*volts/1000);
	
	// update and save the new config to the sd card
//	sd_card_write_config(active_sd_card, &wispr_config);
	
//	uint8_t *active_buffer = adc_buffer;
	
	// Initialize spectrum
	if( wispr_config.mode & WISPR_SPECTRUM ) {
		psd_nfft = wispr_config.fft_size;
		psd_nbins = psd_nfft/2;
		psd_overlap = 0;
		psd_sample_size = wispr_config.sample_size;
		spectrum_init_f32(&wispr_config, &psd_nbins, psd_nfft, psd_overlap, psd_sample_size, HAMMING_WINDOW);
		//spectrum_init_q31(&wispr_config, psd_nbins, psd_nfft, psd_overlap, HAMMING_WINDOW);
	}
	
	// Define some local variable
	// There are two block sizes here - adc_block size and sd block size.
	// The sd block size is always 512
	uint32_t samples_per_window = (uint32_t)wispr_config.window * wispr_config.sampling_rate;
	uint16_t adc_blocks_per_window = (uint16_t)(samples_per_window / (uint32_t)wispr_config.samples_per_block);
	uint16_t buffers_per_interval = (uint16_t)(samples_per_window / (uint32_t)wispr_config.samples_per_block);

	// number of 512 bytes blocks to write for each adc buffer
	uint16_t adc_write_size = wispr_config.block_size / SD_MMC_BLOCK_SIZE;

	// number of 512 bytes blocks to write for each psd buffer
	uint16_t psd_write_size = PSD_MAX_BLOCKS_PER_BUFFER;

	float block_duration =  (float)wispr_config.samples_per_block / (float)wispr_config.sampling_rate;

	uint32_t alarm_seconds = (uint32_t)(wispr_config.interval - wispr_config.window);

	float actual_duration = (float)adc_blocks_per_window * block_duration;
	float actual_interval = (float)alarm_seconds + actual_duration;
	
	printf("Actual window: %f\n\r", actual_duration);
	printf("Actual interval: %f\n\r", actual_interval);

	printf("\n\rStart read loop: %d data blocks per window\n\r", adc_blocks_per_window);

	ltc2512_init_test(&wispr_config, samples_per_adc_block, wispr_config.sampling_rate/10);
	
	// start adc
	ltc2512_start_dma();
	ltc2512_start_conversion();

	// throw away the first adc buffer
	while( ltc2512_read_dma(adc_buffer_header, adc_buffer_data, samples_per_adc_block) == 0 ) {}

	uint32_t start;
	rtc_get_epoch(&start);
	
	// this doesn't help much
	//pmc_disable_periph_clk(ID_UART1);
	//pmc_disable_periph_clk(ID_TWI0);
	
	uint16_t count = 0;
	go = 1; 
	while (go) {
		
		if( count < adc_blocks_per_window ) {
		
			// read the current a buffer. If a new buffer is not ready read returns 0
			uint16_t nsamps = ltc2512_read_dma(adc_buffer_header, adc_buffer_data, samples_per_adc_block);
			
			// if a new buffer is available
			if( nsamps == samples_per_adc_block ) {
				
				// reset the wdt every time a buffer is read
				wdt_restart(WDT);
				
				// calculate the spectrum and save it to the active sd card
				if( wispr_config.mode & WISPR_SPECTRUM ) {
					
					// if sd card is full, swap cards
					if( sd_card_is_full(active_sd_card, psd_write_size) ) {
						swap_sd_cards(); // toggle between the available sd cards
					}
					
					// call spectrum function
					spectrum_f32(psd_buffer, adc_buffer, nsamps);
					//spectrum_q31(psd_buffer, adc_buffer, nsamps);
					
					// write the psd buffer - both header and data
					if( sd_card_write(active_sd_card, psd_buffer, psd_write_size) == 0 ) {
						printf("sd_card_write: failed\n\r");
					}
					
				}

				//write the waveform to the active sd card 				
				if( wispr_config.mode & WISPR_WAVEFORM ) {
				
					// if sd card is full
					if( sd_card_is_full(active_sd_card, adc_write_size) ) {
						swap_sd_cards(); // toggle between the available sd cards
					}

					// write the adc buffer - both header and data
					if( sd_card_write(active_sd_card, adc_buffer, adc_write_size) == 0 ) {
						printf("sd_card_write: failed\n\r");
					}	
				
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

			float32_t *psd = (float32_t *)psd_buffer_data;
			//q31_t *psd = (q31_t *)psd_buffer_data;
			for(int n = 0; n < psd_nbins; n++) {
				//printf("%f ", (float32_t)psd[n] / 2147483648.0 );
				printf("%f ", psd[n] );
				//printf("%d ", psd[n]);
			}
			printf("\r\n");
			
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

	// save the latest config to gpbr, mainly to save the active sd card number 
	wispr_config.active_sd_card = active_sd_card;
	wispr_gpbr_write_config(&wispr_config);
				
	ltc2512_shutdown();
	ltc2512_stop_dma();
	
	// enable wakeup input on WKUP2 (PA2) with active low
	// this is the external rtc interrupt line
	supc_set_wakeup_inputs(SUPC, SUPC_WUIR_WKUPEN2_ENABLE, SUPC_WUIR_WKUPT2_LOW);
	
	/* Switch MCK to slow clock  */
	pmc_switch_mck_to_sclk(PMC_MCKR_PRES_CLK_1);
	//pmc_switch_mainck_to_fastrc(CKGR_MOR_MOSCRCF_4_MHz);
	//pmc_switch_mck_to_mainck(PMC_PCK_PRES_CLK_64);

	// Configure all PIOs as inputs to save power
	pio_set_input(PIOA, 0xFFFFFFFF, PIO_PULLUP);
	pio_set_input(PIOB, 0xFFFFFFFF, PIO_PULLUP);

	/* Disable unused clock to save power */
	pmc_osc_disable_xtal(1);
	pmc_disable_pllack();
	pmc_disable_all_periph_clk();

	// Enter into backup mode
	pmc_enable_backupmode();

}

//
// swap active sd card
// close active card and open the other card
// 
int swap_sd_cards()
{
	if( active_sd_card == 1 ) {
		sd_card_write_config(1, &wispr_config);
		sd_card_close(1);
		active_sd_card = sd_card_open(2);
	} 
	else if( active_sd_card == 2 ) {
		sd_card_write_config(2, &wispr_config);
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


void prompt_config_menu(wispr_config_t *hdr, int timeout)
{	
	hdr->version[1] = WISPR_VERSION;
	hdr->version[0] = WISPR_SUBVERSION;
	
	hdr->block_size = ADC_MAX_BLOCKS_PER_BUFFER * SD_MMC_BLOCK_SIZE;
	hdr->samples_per_block = (hdr->block_size - WISPR_DATA_HEADER_SIZE) / 3;

	hdr->settings[5] = 0;
	hdr->settings[4] = 0;
	hdr->settings[3] = 0;
	hdr->settings[2] = 0;
	hdr->settings[1] = 0; // sample size
	hdr->settings[0] = 1; // active card
	
	uint16_t blocks_per_buffer = ADC_MAX_BLOCKS_PER_BUFFER;
	//blocks_per_buffer = console_prompt_uint32("Enter number of blocks (512 bytes) per buffer", blocks_per_buffer, timeout);
	
	hdr->sample_size = console_prompt_uint32("Enter sample size in bytes", hdr->sample_size, timeout);
	hdr->sampling_rate = console_prompt_uint32("Enter sampling rate in Hz", hdr->sampling_rate, timeout);
	hdr->window = console_prompt_uint32("Enter sampling time window in seconds", hdr->window, timeout);
	hdr->interval = console_prompt_uint32("Enter sampling interval in seconds", hdr->interval, timeout);

	hdr->settings[6] = console_prompt_uint8("Enter preamp gain", hdr->settings[6], timeout);
	hdr->settings[7] = console_prompt_uint8("Enter adc decimation factor (4, 8, 16, or 32)", hdr->settings[7], timeout);

	//float buffer_duration =  (float)hdr->samples_per_block / (float)hdr->sampling_rate;
	hdr->block_size = (uint16_t)(blocks_per_buffer * SD_MMC_BLOCK_SIZE);
	hdr->samples_per_block = (hdr->block_size - WISPR_DATA_HEADER_SIZE) / (uint16_t)hdr->sample_size;
	
	hdr->state = WISPR_READY;

	// prompt to record waveform data
	int record_waveform = 0;
	if(hdr->mode & WISPR_WAVEFORM) record_waveform = 1;	
	if( console_prompt_int("Record_waveform?", record_waveform, timeout) ) {
		hdr->mode |= WISPR_WAVEFORM;
	} else {
		hdr->mode &= ~WISPR_WAVEFORM;	
	}

	// prompt for spectrum parameters
	int record_spectrum = 0;
	if(hdr->mode & WISPR_SPECTRUM) record_spectrum = 1;
	if( console_prompt_int("Record spectrum?", record_spectrum, timeout) ) {
		psd_nfft = hdr->fft_size;
		psd_nfft = console_prompt_uint16("Enter fft size (32, 64, 126, 512 or 1024)", psd_nfft, timeout);
		psd_overlap = 0;
		psd_overlap = console_prompt_uint16("Enter fft overlap size", psd_overlap, timeout);	
		//psd_nbins = psd_nfft / 2;	
		hdr->fft_size = psd_nfft;
		hdr->mode |= WISPR_SPECTRUM;
	} else {
		hdr->mode &= ~WISPR_SPECTRUM;
	}	
	psd_sample_size = hdr->sample_size;
	psd_nfft = hdr->fft_size;

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
	
	//hdr->active_sd_card = console_prompt_uint8("Enter active SD card number (1 or 2)", hdr->active_sd_card, timeout);

	// set config mod time
	rtc_get_epoch(&hdr->epoch);
	
}


