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
#include "com.h"

#include "spectrum.h"
#include "arm_math.h"
#include "arm_const_structs.h"

#include "ff.h"

// Allocate max size buffer
// The compiler will give warning when the buffers are cast into the appropriate data types:
// "cast increases required alignment of target type [-Wcast-align]"
// But using the COMPILER_WORD_ALIGNED macro will avoid any memory alignment problem,
// although the compiler will still give the warning.

COMPILER_WORD_ALIGNED uint8_t adc_buffer[ADC_MAX_BUFFER_SIZE+1];
uint8_t *adc_data = &adc_buffer[WISPR_DATA_HEADER_SIZE]; // data follows header
wispr_data_header_t adc_header;

uint16_t psd_nfft;
uint16_t psd_nbins;
uint16_t psd_overlap;

COMPILER_WORD_ALIGNED uint8_t psd_buffer[PSD_MAX_BUFFER_SIZE+1];
float32_t *psd_data = (float32_t *)&psd_buffer[WISPR_DATA_HEADER_SIZE]; // data follows header
wispr_data_header_t psd_header;
  

#define WISPR_I2C_SPEED 100000UL

// set this only for testing - this will force card swaps 
//uint32_t test_sd_card_nblocks = 300 * ADC_MAX_BLOCKS_PER_BUFFER;
uint32_t test_sd_card_nblocks = 0;

// local function prototypes
void go_to_sleep(void);
uint8_t swap_sd_cards(wispr_config_t *config);
void format_sd_cards(void);
int initialize_sd_cards(wispr_config_t *config);
void set_default_config(wispr_config_t *config);
void initialize_config(wispr_config_t *config);
void prompt_config_menu(wispr_config_t *config, int timeout);

char config_filename[] = "wispr1.txt";

//
// main
//
int main (void)
{
	wispr_config_t wispr; // current configuration
	wispr.active_sd_card = 0; // active sd card number
	
	FRESULT res;
	fat_file_t dat_file;
	fat_file_t psd_file;
	
	// initialize the board specific functions (clocks, gpio, console, wdt, ...)
	// returns the reason the board was last reset (user, sleep, watchdog, ...)
	int reset_type = board_init();

	// Initialize the WDT
	// Once the WDT starts it can't be stopped, Only a processor reset resets it.
	// So make sure wdt_restart() is called regularly. wdt_disable() doesn't stop the wdt.
	uint32_t msecs = board_wdt_init();
	printf("Enable watchdog with %d msec period\n\r", msecs);

	// init I2C bus for the rtc and gpio
	i2c_init(TWI0, WISPR_I2C_SPEED);
	
	// Setup the DS3231 External RTC
	if( ds3231_init() != STATUS_OK ) {
		printf("Error initializing DS3231 RTC\r\n");
	}
	
	// read and display the external rtc time
	rtc_time_t  datetime;
	ds3231_get_datetime(&datetime);
	printf("\r\n");
	printf("External RTC time is %02d/%02d/%02d %02d:%02d:%02d\r\n",
	  datetime.year, datetime.month, datetime.day, datetime.hour, datetime.minute, datetime.second);
	
	// Initialize the internal RTC using the external RTC time
	uint32_t rtc_status = rtc_init(&datetime);
	while ( rtc_status != RTC_STATUS_OK ) {
		printf("Waiting for RTC, status %d\r\n", rtc_status);
		rtc_status = rtc_init(&datetime);
	}
	printf("Internal RTC set to  %02d/%02d/%02d %02d:%02d:%02d\r\n",
		datetime.year, datetime.month, datetime.day, datetime.hour, datetime.minute, datetime.second);
	printf("\r\n");

	// check all sd cards and configure them if needed
	// this also sets the active sd card number 
	res = initialize_sd_cards(&wispr);
	if( res != FR_OK) {
		printf("SD Cards failed to initialize: error %d\n\r", res);
		return(0);
	}
	
	// read the configuration from the active sd card 
	printf("\r\nWriting to ");
	sd_card_print_info(wispr.active_sd_card);

	// if user reset then prompt to change/initialize config
	// skip if reset from backup because there typically will be no user at the console
	// this means that the configuration can only be changed after a user reset or power up
	if( reset_type != BOARD_BACKUP_RESET ) {
		initialize_config(&wispr);
	}

	// save the config because it could have changed
	wispr_print_config(&wispr);
	
	// setup INA260 powe monitor
	uint32_t volts; // mVolts
	int32_t amps; // mAmps
	ina260_init();
	ina260_read_power(&amps, &volts); // first read returns zero
	ina260_read_power(&amps, &volts);

	// gpio control example
	//uint8_t gpio = 0;
	//pcf8574_write(gpio);
	//gpio = 0xFF;
	//pcf8574_write(gpio);
	//gpio = 0x0;
	//pcf8574_write(gpio);
		
	// initialize the adc with the current config 
	ltc2512_init(&wispr, &adc_header);
	
	// initialize the adc dma
	uint16_t samples_per_adc_block = wispr.samples_per_block;
	ltc2512_init_dma(samples_per_adc_block);
	
	// Initialize spectrum
	if( wispr.mode & WISPR_SPECTRUM ) {
		psd_nfft = wispr.fft_size;
		psd_nbins = psd_nfft/2;
		psd_overlap = wispr.fft_overlap;
		//spectrum_init_f32(&psd_nbins, psd_nfft, psd_overlap, wispr.sampling_rate, wispr.sample_size, HANN_WINDOW);
		spectrum_init_q31(&psd_nbins, psd_nfft, psd_overlap, wispr.sample_size, HANN_WINDOW);
	}
	
	// Define the variables that control the window and interval timing.
	float adc_block_duration =  (float)wispr.samples_per_block / (float)wispr.sampling_rate; // seconds
	uint16_t adc_blocks_per_window = (uint16_t)( (float)wispr.awake_time / adc_block_duration ); // truncated number of blocks
	// since the adc buffer duration is defined by a fixed number of blocks
	// the actual sampling window may be different
	uint16_t wakeup_interval = wispr.awake_time + wispr.sleep_time;
	float actual_sampling_time = (float)adc_blocks_per_window * adc_block_duration;
	printf("Actual sampling time: %.2f\n\r", actual_sampling_time);

	// number of 512 bytes blocks to write for each adc buffer
	uint16_t adc_write_size = wispr.block_size / WISPR_SD_CARD_BLOCK_SIZE;

	// number of 512 bytes blocks to write for each psd buffer
	// this is a fixed number now, but could be variable to save card memory
	uint16_t psd_write_size = PSD_MAX_BLOCKS_PER_BUFFER;
	
	printf("\n\rStart read loop: %.2f second windows (%d block) at %d second intervals\n\r", 
		actual_sampling_time, adc_blocks_per_window, wakeup_interval);

	//float32_t amp = 1.0; // test sig amp
	//float32_t noise = 0.2; // noise std dev
	//uint32_t fc = console_prompt_uint32("Enter test signal center freq", wispr.sampling_rate/10, 10);
	//amp = console_prompt_f32("Enter test signal amplitude", amp, 10);
	//noise = console_prompt_f32("Enter test signal noise std dev", noise, 10);
	//ltc2512_init_test(&wispr, samples_per_adc_block, fc, amp, noise);		
	
	// initialize the uart com communications port
	wispr_com_msg_t com_msg;
	char com_buf[COM_MAX_MESSAGE_SIZE];
	com_init(BOARD_COM_PORT, BOARD_COM_BAUDRATE);

	// start adc
	ltc2512_start_dma();
	ltc2512_start_conversion();
	
	// open a dat file
	if( wispr.mode & WISPR_WAVEFORM ) {
		res = sd_card_open_fat(&dat_file, "WISPR", "dat", FA_OPEN_APPEND | FA_WRITE, wispr.active_sd_card);
		if( res != FR_OK ) {
			printf("sd_card_open_fat: error %d\r\n", res);
		} else {
			printf("Open new raw data file: %s\r\n", dat_file.name);
		}
	}

	// open a psd file
	if( wispr.mode & WISPR_SPECTRUM ) {
		res = sd_card_open_fat(&psd_file, "WISPR", "psd", FA_OPEN_APPEND | FA_WRITE, wispr.active_sd_card);
		if( res != FR_OK ) {
			printf("sd_card_open_fat: error %d\r\n", res);
		} else {
			printf("Open new spectrum file: %s\r\n", psd_file.name);
		}
	}

	// throw away the first adc buffer
	while( ltc2512_read_dma(&adc_header, adc_data, samples_per_adc_block) == 0 ) {}
	
	// enter read loop
	int go = 1;
	uint16_t count = 0;
	while (go) {

		// if within the sampling window
		if( count < adc_blocks_per_window ) {
	
			// check for a com message
			int nrd = com_read_msg (BOARD_COM_PORT, com_buf);
			if( nrd > 0) {
				com_parse_msg(&com_msg, com_buf, nrd);
				printf("com message received: %s\r\n", com_buf);
			}
				
			// read the current a buffer. If a new buffer is not ready read returns 0
			uint16_t nsamps = ltc2512_read_dma(&adc_header, adc_data, samples_per_adc_block);
			
			// if a new buffer is available
			if( nsamps == samples_per_adc_block ) {
				 
				// reset the wdt every time a buffer is read
				wdt_restart(WDT);
				
				// calculate the spectrum and save it to the active sd card
				if( wispr.mode & WISPR_SPECTRUM ) {
					
					// if sd card is full, swap cards
					if( sd_card_is_full(wispr.active_sd_card, psd_write_size) ) {
						// toggle between the available sd cards
						sd_card_close_fat(&psd_file);
						swap_sd_cards(&wispr);
					}
					
					// call spectrum function
					//spectrum_f32(&psd_header, psd_data, &adc_header, adc_data, nsamps);
					spectrum_q31(&psd_header, psd_data, &adc_header, adc_data, nsamps);
					
					// serialize the buffer header - write the latest header onto the front of the buffer
					wispr_serialize_data_header(&psd_header, psd_buffer);

					// write the psd buffer - both header and data
					res = sd_card_write_fat(&psd_file, psd_buffer, psd_write_size);
					if( res != FR_OK ) {
						printf("sd_card_write_fat error %d\r\n", res);
						return(0);
					}

					//if( sd_card_write_fat(wispr.active_sd_card, psd_buffer, psd_write_size) == 0 ) {
					//	printf("sd_card_write: failed\n\r");
					//}
					//printf("psd_write_size: %d\n\r", psd_write_size);
				
				}

				//write the waveform to the active sd card 				
				if( wispr.mode & WISPR_WAVEFORM ) {
				
					// if sd card is full
					if( sd_card_is_full(wispr.active_sd_card, adc_write_size) ) {
						// toggle between the available sd cards
						sd_card_close_fat(&dat_file);
						swap_sd_cards(&wispr);
					}

					// serialize the buffer header - write the latest adc header onto the front of the buffer
					wispr_serialize_data_header(&adc_header, adc_buffer);

					// write the adc buffer - both header and data
					res = sd_card_write_fat(&dat_file, adc_buffer, adc_write_size);
					if( res != FR_OK ) {
						printf("sd_card_write_fat error %d\r\n", res);
						return(0);
					}
					//if( sd_card_write_fat(wispr.active_sd_card, adc_buffer, adc_write_size) == 0 ) {
					//	printf("sd_card_write: failed\n\r");
					//}
					//printf("adc_write_size: %d\n\r", adc_write_size);
				
				}
				
				// increment buffer count
				count++;
				
			}
		
			//ina260_read_power(&amps, &volts);
			//printf("ina260: mA = %lu, mV = %lu\r\n", amps, volts);
		
			// sleep between dma buffers, the next dma interrupt will wake from sleep
			pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);

		}

		// else go to deep sleep 
		else if( wispr.sleep_time > 0 ) {

			// for testing only
			//printf("nbins = %d\r\n", psd_nbins);
			//printf("psd = [\r\n");
			//for(int n = 0; n < psd_nbins; n++) {
			//	printf("%f ", psd_data[n]);
			//}
			//printf("];\r\n");				

			uint32_t now;
			rtc_get_epoch(&now);
			
			// save the latest config
			// update the config time so the last active card number can be determined on wakeup
			wispr.epoch = now;
			sd_card_write_config_fat(config_filename, &wispr);
			
			if( wispr.mode & WISPR_WAVEFORM ) sd_card_close_fat(&dat_file);
			if( wispr.mode & WISPR_SPECTRUM ) sd_card_close_fat(&psd_file);
			
			// close active sd card
			sd_card_umount_fat(wispr.active_sd_card);

			// set the alarm to wakeup for the next window read cycle
			epoch_to_rtc_time((rtc_time_t *)&datetime, now + wispr.sleep_time);
			ds3231_set_alarm(&datetime);
			printf("\r\nAlarm set for %s\r\n", epoch_time_string(now + wispr.sleep_time));				

			count = 0; // reset block counter

			// enter backup mode sleep
			go_to_sleep();	// The core will reset when exiting from backup mode. 
		
		} 

		// else sleep interval, just start new sampling window
		else { 
			count = 0; // reset block counter			
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
	
	// flush the uarts
	while (!uart_is_tx_empty(UART1)) {}

	// enable wakeup input on WKUP2 (PA2) with active low
	// this is the external rtc interrupt line
	supc_set_wakeup_inputs(SUPC, SUPC_WUIR_WKUPEN2_ENABLE, SUPC_WUIR_WKUPT2_LOW);
	
	/* Switch MCK to slow clock  */
	pmc_switch_mck_to_sclk(PMC_MCKR_PRES_CLK_1);
	//pmc_switch_mainck_to_fastrc(CKGR_MOR_MOSCRCF_4_MHz);
	//pmc_switch_mck_to_mainck(PMC_PCK_PRES_CLK_64);

	// Configure all PIOs as inputs to save power
	//pio_set_input(PIOA, 0xFFFFFFFF, PIO_PULLUP);
	//pio_set_input(PIOB, 0xFFFFFFFF, PIO_PULLUP);

	/* Disable unused clock to save power */
	pmc_osc_disable_xtal(1);
	pmc_disable_pllack();
	pmc_disable_all_periph_clk();

	// Enter into backup mode
	pmc_enable_backupmode();

}

uint8_t swap_sd_cards(wispr_config_t *config)
{
	
	if( config->active_sd_card == 1 ) {
		sd_card_umount_fat(1);
		sd_card_mount_fat(2);
		config->active_sd_card = 2;
	}
	else if( config->active_sd_card == 2 ) {
		sd_card_umount_fat(2);
		sd_card_mount_fat(1);
		config->active_sd_card = 1;
	} else {
		printf("SD Card Swap Failed\n\r");
		return(0);
	}

	// for testing only
	if( test_sd_card_nblocks > 0 ) {
		
		// grow the card size - used for testing
		uint32_t nblocks = sd_card_get_number_of_blocks(config->active_sd_card);
		sd_card_set_number_of_blocks(config->active_sd_card, nblocks + test_sd_card_nblocks);

		uint32_t epoch = 0;
		rtc_get_epoch(&epoch);
		printf("\r\nSwitch to Card %d at %s\r\n", config->active_sd_card, epoch_time_string(epoch));

		//sd_card_print_info(config->active_sd_card);

	}

	// update the config time and save it to preserve the active card number
	rtc_get_epoch(&config->epoch);
	sd_card_write_config_fat(config_filename, config);

	return(config->active_sd_card);
}

void set_default_config(wispr_config_t *config)
{
	// set config mod time
	rtc_get_epoch(&config->epoch);
	
	config->version[1] = WISPR_VERSION;
	config->version[0] = WISPR_SUBVERSION;
	
	config->block_size = ADC_MAX_BLOCKS_PER_BUFFER * SD_MMC_BLOCK_SIZE;
	config->sample_size = ADC_MIN_SAMPLE_SIZE;
	config->sampling_rate = ADC_DEFAULT_SAMPLING_RATE;

	config->awake_time = ADC_DEFAULT_AWAKE;
	config->sleep_time = ADC_DEFAULT_SLEEP;

	config->samples_per_block = (uint32_t)(config->block_size - WISPR_DATA_HEADER_SIZE) / (uint32_t)config->sample_size;

	config->gain = ADC_DEFAULT_GAIN; // adc gain
	config->adc_decimation = LTC2512_DF8; // adc df

	config->state = 0; //
	config->mode = WISPR_WAVEFORM; //

	config->fft_size = 1024;
	config->fft_overlap = 0;
	config->active_sd_card = 1;
}

void format_sd_cards()
{
	//uint8_t active_sd_card = config->active_sd_card;
	wispr_config_t config;
	
	// Prompt to reset the cards to start writing and reading from the beginning
	// this will overwrite the existing data on the card
	int format_sd_card = 0;
	format_sd_card = console_prompt_int("Reformat and erase SD cards", format_sd_card, 8);

	wdt_restart(WDT);

	// this should only need to be done once
	if(format_sd_card) {
		// loop over all the cards
		for(int n = 1; n <= NUMBER_SD_CARDS; n++) {
			char str[16];
			sprintf(str, "Reformat SD card %d", n);
			if( console_prompt_int(str, 0, 8) ) {
				sd_card_format_fat(n);
				sd_card_mount_fat(n);
				set_default_config(&config);
				sd_card_write_config_fat(config_filename, &config);
			}
		}
	}
	
}

// 
// Open each sd card and check the free storage.
// Stop and return when a card has space, making that card active.
// Exits with active card mounted and config updated with values from the active card.
//
int initialize_sd_cards(wispr_config_t *config)
{
	FRESULT res;
	
	// loop over all the cards 
	for(int n = 1; n <= NUMBER_SD_CARDS; n++) {
	
		// mount card
		res = sd_card_mount_fat(n);

		// if problem mounting, then continue to next card
		if (res != FR_OK) {
			if ( res == FR_NO_FILESYSTEM ) {		
				printf("initialize_sd_cards: card %d if unformatted\r\n", n);
			} else {
				printf("initialize_sd_cards: card %d error %d\r\n", n, res);
			}
			res = sd_card_umount_fat(n);
			continue;
		}
		
		// check free space on card
		// continue to next card if there's not enough
		if( sd_card_free_space(n) < 32 ) {
			printf("initialize_sd_cards: card %d is full\r\n", n);
			res = sd_card_umount_fat(n);
			continue;
		}

		// read the configuration
		res = sd_card_read_config_fat(config_filename, config);
		if( res != FR_OK) {
			if( res == FR_NO_FILE) {
				// set defaults config and write it 
				set_default_config(config);
				if( sd_card_write_config_fat(config_filename, config) == FR_OK ) {
					printf("Default configuration set and written to card %d\r\n", n);
				}
			} else {			
				printf("initialize_sd_cards: error %d reading config from card %d\r\n", res, n);
			}
		}
		
		// use this card and break from loop
		config->active_sd_card = n;
		
		//printf("initialize_sd_cards: card %d selected\r\n", n);
		
		break;
		
	}
	return(res);
}

void initialize_config(wispr_config_t *config)
{		
	//uint8_t card_num = config->active_sd_card;
	// writes config to whatever card that is currently mounted	

	// display the config
	wispr_print_config(config);
			
	// Prompt to set new configuration
	while(1) {
		if( console_prompt_int("Change configuration?", 0, 8) ) {
			prompt_config_menu(config, 60);
		} else break;	
		wispr_print_config(config);
	}
	
	// save the new config and close the card
	sd_card_write_config_fat(config_filename, config);

}

void prompt_config_menu(wispr_config_t *config, int timeout)
{	
	uint32_t u32;
	uint16_t u16;
	uint8_t u8;
	
	config->version[1] = WISPR_VERSION;
	config->version[0] = WISPR_SUBVERSION;
	
	config->block_size = ADC_MAX_BLOCKS_PER_BUFFER * SD_MMC_BLOCK_SIZE;
	config->samples_per_block = (config->block_size - WISPR_DATA_HEADER_SIZE) / 3;

	// these settings are not used yet, so clear them
	//config->settings[7] = 0;
	//config->settings[6] = 0;
	//config->settings[5] = 0;
	//config->settings[4] = 0;
	//config->settings[3] = 0;
	//config->settings[2] = 0;
	
	uint16_t blocks_per_buffer = ADC_MAX_BLOCKS_PER_BUFFER;
	//blocks_per_buffer = console_prompt_uint32("Enter number of blocks (512 bytes) per buffer", blocks_per_buffer, timeout);
	
	u8 = console_prompt_uint8("Enter sample size in bytes", config->sample_size, timeout);
	if( u8 >= 2 && u8 <= 3 ) config->sample_size = u8;

	u32 = console_prompt_uint32("Enter sampling rate in Hz", config->sampling_rate, timeout);
	if( u32 > 0 && u32 <= 350000 ) config->sampling_rate = u32;

	u8 = config->gain;
	u8 = console_prompt_uint8("Enter preamp gain setting (0 to 4)", u8, timeout);
	if( u8 >= 0 && u8 <= 4 ) config->gain = u8;

	u8 = config->adc_decimation;
	u8 = console_prompt_uint8("Enter adc decimation factor (4, 8, 16, or 32)", u8, timeout);
	if( u8 == 4 || u8 == 8 || u8 == 16 || u8 == 32) config->adc_decimation = u8;

	// prompt for sampling interval
	u16 = 1;
	if( config->sleep_time > 0 ) u16 = 0;
	u16 = console_prompt_uint16("Enter continuous sampling", u16, timeout);
	if( u16 == 1 ) {
		config->awake_time = 10;
		config->sleep_time = 0;
	} else {
		u16 = console_prompt_uint16("Enter sampling time window in seconds", config->awake_time, timeout);
		if( u16 >= 1 ) config->awake_time = u16;
		u16 = console_prompt_uint16("Enter sleep time between sampling windows in seconds", config->sleep_time, timeout);
		if( u16 >= 0 ) config->sleep_time = u16;
	}
	
	// update variables based on new input
	config->block_size = (uint16_t)(blocks_per_buffer * SD_MMC_BLOCK_SIZE);
	config->samples_per_block = (config->block_size - WISPR_DATA_HEADER_SIZE) / (uint16_t)config->sample_size;
	float adc_block_duration =  (float)config->samples_per_block / (float)config->sampling_rate; // seconds
	config->blocks_per_window = (uint16_t)( (float)config->awake_time / adc_block_duration ); // truncated number of blocks
	
	config->state = WISPR_READY;
	uint8_t mode = 0;
	
	// prompt to record waveform data
	int record_waveform = 0;
	if(config->mode & WISPR_WAVEFORM) record_waveform = 1;	
	if( console_prompt_int("Record_waveform?", record_waveform, timeout) ) {
		mode |= WISPR_WAVEFORM;
	}

	// prompt for spectrum parameters
	int record_spectrum = 0;
	if(config->mode & WISPR_SPECTRUM) record_spectrum = 1;
	if( console_prompt_int("Record spectrum?", record_spectrum, timeout) ) {
		psd_nfft = config->fft_size;
		psd_nfft = console_prompt_uint16("Enter fft size (32, 64, 126, 512 or 1024)", psd_nfft, timeout);
		psd_overlap = config->fft_overlap;
		psd_overlap = console_prompt_uint16("Enter fft overlap size", psd_overlap, timeout);	
		config->fft_size = psd_nfft;
		config->fft_overlap = psd_overlap;
		mode |= WISPR_SPECTRUM;
	} 	
	psd_nfft = config->fft_size;
	psd_nbins = psd_nfft / 2;
	psd_overlap = config->fft_overlap;

	// set the new mode
	config->mode = mode;
	
	// prompt to  reset the time
	if( console_prompt_int("Enter new time?", 0, timeout) ) {
		int go = 1;
		rtc_time_t datetime;
		ds3231_get_datetime(&datetime);  // get current time
		while(go) {
			ds3231_get_datetime(&datetime);  // get current time
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


