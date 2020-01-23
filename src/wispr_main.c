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

#define WISPR_I2C_SPEED 100000UL

uint32_t test_sd_card_nblocks = 0; //(ADC_BLOCKS_PER_BUFFER * 100); // TESTING ONLY 

// local function prototypes
void go_to_sleep(void);
uint8_t swap_sd_cards(wispr_config_t *config);
void initialize_sd_cards(wispr_config_t *config);
void set_default_config(wispr_config_t *config);
void initialize_config(wispr_config_t *config);
void prompt_config_menu(wispr_config_t *config, int timeout);

//
// main
//
int main (void)
{
	wispr_config_t wispr; // current configuration
	wispr.active_sd_card = 0; // active sd card number
		
	// initialize the board specific function (clocks, gpio, console, wdt, ...)
	// returns the reason the board was last reset (user, sleep, watchdog, ...)
	int reset_type = board_init();

	i2c_init(TWI0, WISPR_I2C_SPEED);
	
	// Read current configuration from general purpose backup registers (gpbr)
	// if config is not valid, set defaults
//	if( reset_type == BOARD_BACKUP_RESET ) {
//		if(wispr_gpbr_read_config(&wispr) == 0 ) {
//			// if gpbr config is not valid - something whent wrong
//			printf("\r\nWARNING - Initializing WISPR configuration coming out of backup reset.\r\n");
//			set_default_config(&wispr);	// set defaults
//		}
//	}

	// Setup the DS3231 External RTC
	if( ds3231_init() != STATUS_OK ) {
		printf("Error initializing DS3231 RTC\r\n");
	}
	
	// read and display the external rtc time
	rtc_time_t  datetime;
	ds3231_get_datetime(&datetime);
	printf("\r\nExternal RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n",
	  datetime.year, datetime.month, datetime.day, datetime.hour, datetime.minute, datetime.second);
	
	// Initialize the internal RTC using the external RTC time
	rtc_init(&datetime);
	rtc_get_datetime(&datetime);
	
	printf("Internal RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n",
		datetime.year, datetime.month, datetime.day, datetime.hour, datetime.minute, datetime.second);

	// check all sd cards and format and configure them if needed
	// this sets the active sd card number 
	initialize_sd_cards(&wispr);
	
	// read the configuration from the active sd card 
	sd_card_open(wispr.active_sd_card);
	sd_card_print_info(wispr.active_sd_card);
	sd_card_read_config(wispr.active_sd_card, &wispr);
	sd_card_close(wispr.active_sd_card);

	// prompt to change/initialize config and display card info	
	// skip if reset from backup because there typically will be no user at the console
	// this means that the configuration can only be changed after a user reset or powerup
	if( reset_type != BOARD_BACKUP_RESET ) {
		initialize_config(&wispr);
	}
	
	// save the config because it could have changed
	wispr_print_config(&wispr);
	
	// Open the active sd card
	if( sd_card_open(wispr.active_sd_card) == 0 ) {
		printf("SD Card %d failed to open\n\r", wispr.active_sd_card);
	}
	printf("\n\r");

	// setup INA260
	uint32_t volts; // mVolts
	int32_t amps; // mAmps
	ina260_init();
	ina260_read_power(&amps, &volts); // first read returns zero

	// gpio control example
	//uint8_t gpio = 0;
	//pcf8574_write(gpio);
	//gpio = 0xFF;
	//pcf8574_write(gpio);
	//gpio = 0x0;
	//pcf8574_write(gpio);
		
	// initialize the adc with the current config 
	ltc2512_init(&wispr);
	
	// initialize the adc dma
	uint16_t samples_per_adc_block = wispr.samples_per_block;
	ltc2512_init_dma(samples_per_adc_block);
	
	//ina260_read_power(&amps, &volts);
	//printf("\r\nina260: mA = %lu, mV = %lu, mW = %lu\r\n", amps, volts, amps*volts/1000);
		
	// Initialize spectrum
	if( wispr.mode & WISPR_SPECTRUM ) {
		psd_nfft = wispr.fft_size;
		psd_nbins = psd_nfft/2;
		psd_overlap = 0;
		psd_sample_size = wispr.sample_size;
		spectrum_init_f32(&wispr, &psd_nbins, psd_nfft, psd_overlap, psd_sample_size, HAMMING_WINDOW);
	}
	
	// Define the variables that control the window and interval timing.
	float adc_block_duration =  (float)wispr.samples_per_block / (float)wispr.sampling_rate; // seconds
	uint16_t adc_blocks_per_window = (uint16_t)( (float)wispr.window / adc_block_duration ); // truncated number of blocks
	uint16_t sleep_seconds = (wispr.interval - wispr.window); // sleep time between windows in seconds 
	// since the adc buffer duration is defined by a fixed number of blocks
	// the actual window and interval durations are:
	float actual_window = (float)adc_blocks_per_window * adc_block_duration;
	float actual_interval = (float)sleep_seconds + actual_window;
	printf("Actual window: %f\n\r", actual_window);
	printf("Actual interval: %f\n\r", actual_interval);

	//uint32_t samples_per_window = (uint32_t)wispr.window * wispr.sampling_rate;
	//uint16_t adc_blocks_per_window = (uint16_t)(samples_per_window / (uint32_t)wispr.samples_per_block);
	//uint16_t buffers_per_interval = (uint16_t)(samples_per_window / (uint32_t)wispr.samples_per_block);

	// number of 512 bytes blocks to write for each adc buffer
	uint16_t adc_write_size = wispr.block_size / WISPR_SD_CARD_BLOCK_SIZE;

	// number of 512 bytes blocks to write for each psd buffer
	// this is a fixed number now, but could be variable to save card memory
	uint16_t psd_write_size = PSD_MAX_BLOCKS_PER_BUFFER;

	printf("\n\rStart read loop: %d data blocks per window\n\r", adc_blocks_per_window);

	ltc2512_init_test(&wispr, samples_per_adc_block, wispr.sampling_rate/10);
	
	// start adc
	ltc2512_start_dma();
	ltc2512_start_conversion();

	// throw away the first adc buffer
	while( ltc2512_read_dma(adc_buffer_header, adc_buffer_data, samples_per_adc_block) == 0 ) {}

	uint32_t start;
	rtc_get_epoch(&start);
	
	// enter read loop
	int go = 1;
	uint16_t count = 0;
	while (go) {
		
		if( count < adc_blocks_per_window ) {
		
			// read the current a buffer. If a new buffer is not ready read returns 0
			uint16_t nsamps = ltc2512_read_dma(adc_buffer_header, adc_buffer_data, samples_per_adc_block);
			
			// if a new buffer is available
			if( nsamps == samples_per_adc_block ) {
				
				// reset the wdt every time a buffer is read
				wdt_restart(WDT);
				
				// calculate the spectrum and save it to the active sd card
				if( wispr.mode & WISPR_SPECTRUM ) {
					
					// if sd card is full, swap cards
					if( sd_card_is_full(wispr.active_sd_card, psd_write_size) ) {
						// toggle between the available sd cards
						swap_sd_cards(&wispr);
					}
					
					// call spectrum function
					spectrum_f32(psd_buffer, adc_buffer, nsamps);
					//spectrum_q31(psd_buffer, adc_buffer, nsamps);
					
					// write the psd buffer - both header and data
					if( sd_card_write(wispr.active_sd_card, psd_buffer, psd_write_size) == 0 ) {
						printf("sd_card_write: failed\n\r");
					}
					
				}

				//write the waveform to the active sd card 				
				if( wispr.mode & WISPR_WAVEFORM ) {
				
					// if sd card is full
					if( sd_card_is_full(wispr.active_sd_card, psd_write_size) ) {
						// toggle between the available sd cards
						swap_sd_cards(&wispr);
					}

					// write the adc buffer - both header and data
					if( sd_card_write(wispr.active_sd_card, adc_buffer, adc_write_size) == 0 ) {
						printf("sd_card_write: failed\n\r");
					}	
				
				}
				
				// increment buffer count
				count++;
				
			}
		
			//ina260_read_power(&amps, &volts);
			//printf("ina260: mA = %lu, mV = %lu\r\n", amps, volts);
		
			// sleep between dma buffers, the next dma interrupt will wake from sleep
			pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);

		} else if( sleep_seconds > 0 ) {

			uint32_t now;
			rtc_get_epoch(&now);

			float32_t *psd = (float32_t *)psd_buffer_data;
			//q31_t *psd = (q31_t *)psd_buffer_data;
			for(int n = 0; n < psd_nbins; n++) {
				//printf("%f ", (float32_t)psd[n] / 2147483648.0 );
				printf("%f ", psd[n] );
				//printf("%d ", psd[n]);
			}
			printf("\r\n");
			
			// close active sd card 
			// but first save the latest config to gpbr and sd card,
			//  mainly to preserve the active sd card number for when it wakes up
			wispr.epoch = now;
			wispr_gpbr_write_config(&wispr);
			sd_card_write_config(wispr.active_sd_card, &wispr);
			sd_card_close(wispr.active_sd_card);

			// set the alarm to wakeup for the next window read cycle
			epoch_to_rtc_time((rtc_time_t *)&datetime, now + sleep_seconds);
			ds3231_set_alarm(&datetime);
			printf("\r\nAlarm set for %s\r\n", epoch_time_string(now + sleep_seconds));				

			count = 0; // reset block counter

			// enter backup mode sleep
			go_to_sleep();	// The core will reset when exiting from backup mode. 
		
		}
	
	}
	
	ltc2512_stop_conversion();
	ltc2512_stop_dma();
	
	printf("Finished\n\r");
	
	exit(0);
}

//
// Enter backup mode sleep, shutting down as much as possible to save power
//
void go_to_sleep(void)
{
	// save config, close the active card, and disable all the sd cards
	sd_card_disable(1);
	sd_card_disable(2);
	
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
uint8_t swap_sd_cards(wispr_config_t *config)
{
	uint8_t new_sd_card = 0;
	
	if( config->active_sd_card == 1 ) {
		sd_card_close(1);
		sd_card_open(2);
		config->active_sd_card = 2;
	} 
	else if( config->active_sd_card == 2 ) {
		sd_card_close(2);
		sd_card_open(1);
		config->active_sd_card = 1;
	} else {
		printf("SD Card Swap Failed\n\r");
		return(0);
	}

	// for testing only
	if( test_sd_card_nblocks ) {
		
		// grow the card size - used for testing
		uint32_t nblocks = sd_card_get_number_of_blocks(config->active_sd_card);
		sd_card_set_number_of_blocks(config->active_sd_card, nblocks + test_sd_card_nblocks);

		rtc_time_t time;
		rtc_get_datetime(&time);
		printf("SD Card %d Active at ", config->active_sd_card);
		printf("%02d/%02d/%02d %02d:%02d:%02d\r\n",
			time.year, time.month, time.day, time.hour, time.minute, time.second);

		sd_card_print_info(config->active_sd_card);		
	}

	// update the sd card config preserve the active card number
	sd_card_write_config(config->active_sd_card, config);

	return(config->active_sd_card);		
}

void set_default_config(wispr_config_t *config)
{
	// set config mod time
	rtc_get_epoch(&config->epoch);
	
	config->version[1] = WISPR_VERSION;
	config->version[0] = WISPR_SUBVERSION;
	
	config->block_size = ADC_MAX_BLOCKS_PER_BUFFER * SD_MMC_BLOCK_SIZE;
	config->sample_size = 3;
	config->samples_per_block = (config->block_size - WISPR_DATA_HEADER_SIZE) / 3;
	config->sampling_rate = ADC_DEFAULT_SAMPLING_RATE;
	config->window = ADC_DEFAULT_WINDOW;
	config->interval = ADC_DEFAULT_INTERVAL;

	config->settings[7] = LTC2512_DF4; // adc df
	config->settings[6] = 0; // adc gain
	config->settings[5] = 0;
	config->settings[4] = 0;
	config->settings[3] = 0;
	config->settings[2] = 0;
	config->settings[1] = config->sample_size; // sample size
	config->settings[0] = 1; // active card

	config->state = 0; //
	config->mode = WISPR_WAVEFORM; //

	config->fft_size = 512;
	config->active_sd_card = 1;
}

void initialize_sd_cards(wispr_config_t *config)
{
	//uint8_t active_sd_card = config->active_sd_card;
	uint8_t state;
	wispr_config_t config1, config2;
	
	// open, format if needed, and read config from the card 2
	state = sd_card_open(2);
	if( !(state & SD_CARD_FORMATED) ) {
		printf("SD Card 2 is unformatted.\r\n");
		if( console_prompt_int("Reformat SD Card2?", 1, 10) ) {
			sd_card_format(2,"Card2");
		}
		set_default_config(&config2);
		sd_card_write_config(2, &config2);
	}
	if( state == SD_CARD_READY ) {
		sd_card_read_config(2, &config2);
		//sd_card_print_info(2);
	} else {
		printf("SD Card 2 failed to initialize\r\n");
	}
	sd_card_close(2);
	
	// open, format if needed, and read config from card 1
	state = sd_card_open(1);
	if( !(state & SD_CARD_FORMATED) ) {
		printf("SD Card 1 is unformatted.\r\n");
		if( console_prompt_int("Reformat SD Card1?", 1, 10) ) {
			sd_card_format(1,"Card1");
		}
		set_default_config(&config1);
		sd_card_write_config(1, &config1);
	}
	if ( state == SD_CARD_READY ) {
		sd_card_read_config(1, &config1);	
		//sd_card_print_info(1);
	} else {
		printf("SD Card 1 failed to initialize\r\n");
	}
	sd_card_close(1);
	
	// set the active sd card number based on time

	// use 1 by default
	config->active_sd_card = 1;
	
	// use card 2 if config2 is newer than config1 
	if(config2.epoch > config1.epoch) config->active_sd_card = 2;

}

void initialize_config(wispr_config_t *config)
{		
	uint8_t card_num = config->active_sd_card;
	//card_num = (uint8_t)console_prompt_int("Enter active SD card number", (int)card_num, 5);
	//config->active_sd_card = card_num;

	// read config from the specified card
	uint8_t state = sd_card_open(card_num);
	if( !(state & SD_CARD_READY) ) {
		printf("Can't read configuration from SD card %d, using defaults.\r\n", card_num);
		set_default_config(config);
	} else {
		printf("Read configuration from SD card %d.\r\n", card_num);
		sd_card_read_config(card_num, config);
	}
		
	// display the config
	wispr_print_config(config);
			
	// Prompt to set new configuration
	if( console_prompt_int("Change configuration?", 0, 5) ) {
		prompt_config_menu(config, 60);
	}
	
	// save the new config and close the card
	sd_card_write_config(card_num, config);
	sd_card_close(card_num);

	wispr_gpbr_write_config(config);

	// Prompt to reset the cards to start writing and reading from the beginning
	// this will overwrite the existing data on the card
	uint8_t reset_sd_card = 0;
	reset_sd_card = (uint8_t)console_prompt_int("Reformat and overwrite data on both SD cards", (int)reset_sd_card, 5);

	// this is mainly for testing
	if(reset_sd_card) {

		uint32_t nrecs = 0;
		nrecs = (uint32_t)console_prompt_int("Number of data blocks to write on SD cards (0 to use full card)", (int)nrecs, 10);
		test_sd_card_nblocks = (uint32_t)config->block_size * nrecs / SD_MMC_BLOCK_SIZE;
	
		sd_card_open(2);
		sd_card_format(2,"Card2");
		sd_card_write_config(2, config);
		if(test_sd_card_nblocks != 0) sd_card_set_number_of_blocks(2, test_sd_card_nblocks);
		sd_card_close(2);

		sd_card_open(1);
		sd_card_format(1,"Card1");
		sd_card_write_config(1, config);
		if(test_sd_card_nblocks != 0) sd_card_set_number_of_blocks(1, test_sd_card_nblocks);
		sd_card_close(1);
	
	}

}

void prompt_config_menu(wispr_config_t *config, int timeout)
{	
	config->version[1] = WISPR_VERSION;
	config->version[0] = WISPR_SUBVERSION;
	
	config->block_size = ADC_MAX_BLOCKS_PER_BUFFER * SD_MMC_BLOCK_SIZE;
	config->samples_per_block = (config->block_size - WISPR_DATA_HEADER_SIZE) / 3;

	config->settings[5] = 0;
	config->settings[4] = 0;
	config->settings[3] = 0;
	config->settings[2] = 0;
	config->settings[1] = 0; // sample size
	config->settings[0] = 1; // active card
	
	uint16_t blocks_per_buffer = ADC_MAX_BLOCKS_PER_BUFFER;
	//blocks_per_buffer = console_prompt_uint32("Enter number of blocks (512 bytes) per buffer", blocks_per_buffer, timeout);
	
	config->sample_size = console_prompt_uint32("Enter sample size in bytes", config->sample_size, timeout);
	config->sampling_rate = console_prompt_uint32("Enter sampling rate in Hz", config->sampling_rate, timeout);
	config->window = console_prompt_uint32("Enter sampling time window in seconds", config->window, timeout);
	config->interval = console_prompt_uint32("Enter sampling interval in seconds", config->interval, timeout);

	config->settings[6] = console_prompt_uint8("Enter preamp gain", config->settings[6], timeout);
	config->settings[7] = console_prompt_uint8("Enter adc decimation factor (4, 8, 16, or 32)", config->settings[7], timeout);

	//float buffer_duration =  (float)config->samples_per_block / (float)config->sampling_rate;
	config->block_size = (uint16_t)(blocks_per_buffer * SD_MMC_BLOCK_SIZE);
	config->samples_per_block = (config->block_size - WISPR_DATA_HEADER_SIZE) / (uint16_t)config->sample_size;
	
	config->state = WISPR_READY;

	// prompt to record waveform data
	int record_waveform = 0;
	if(config->mode & WISPR_WAVEFORM) record_waveform = 1;	
	if( console_prompt_int("Record_waveform?", record_waveform, timeout) ) {
		config->mode |= WISPR_WAVEFORM;
	} else {
		config->mode &= ~WISPR_WAVEFORM;	
	}

	// prompt for spectrum parameters
	int record_spectrum = 0;
	if(config->mode & WISPR_SPECTRUM) record_spectrum = 1;
	if( console_prompt_int("Record spectrum?", record_spectrum, timeout) ) {
		psd_nfft = config->fft_size;
		psd_nfft = console_prompt_uint16("Enter fft size (32, 64, 126, 512 or 1024)", psd_nfft, timeout);
		psd_overlap = 0;
		psd_overlap = console_prompt_uint16("Enter fft overlap size", psd_overlap, timeout);	
		//psd_nbins = psd_nfft / 2;	
		config->fft_size = psd_nfft;
		config->mode |= WISPR_SPECTRUM;
	} else {
		config->mode &= ~WISPR_SPECTRUM;
	}	
	psd_sample_size = config->sample_size;
	psd_nfft = config->fft_size;

	// prompt to  reset the time
	if( console_prompt_int("Enter new time?", 0, timeout) ) {
		int go = 1;
		rtc_time_t datetime;
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
	rtc_get_epoch(&config->epoch);
	
}


