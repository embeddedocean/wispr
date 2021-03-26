/*
 * wispr_example2.c
 *
 * - Continuous data acquisition using a double buffering scheme where 
 * - one data buffer is written to the SD card while the the adc dma fills the other.
 * - Uninterrupted data acquisition is limited by the write speed of the sd card and whatever processing
 * - is done on the data between buffer reads, such as spectral analysis.
 * - To optimize write speed there is no file system on the SD card. The card is used as non-volatile memory.
 * - Each data buffer has a header that included the timestamp and sampling parameters.
 * - The program will read/write data until interrupted by a serial port input.
 * - To optimize memory usage, the sample size and buffer size are fixed.
 * - The fft size for spectral analysis is also fixed.
 * - The user is prompted of other data collection parameters when the system is reset by the user.
 *
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include <string.h>
#include <stdio.h>

#include "wispr.h"
#include "wispr_config.h"
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
#include "pps_timer.h"

#include "spectrum.h"
#include "arm_math.h"
#include "arm_const_structs.h"

#include "ff.h"

// Allocate max size buffer
// The compiler will give warning when the buffers are cast into the appropriate data types:
// "cast increases required alignment of target type [-Wcast-align]"
// But using the COMPILER_WORD_ALIGNED macro will avoid any memory alignment problem,
// although the compiler will still give the warning.

COMPILER_WORD_ALIGNED uint8_t adc_buffer[ADC_BUFFER_SIZE+4];
uint8_t *adc_data = &adc_buffer[WISPR_DATA_HEADER_SIZE]; // data follows header
wispr_data_header_t adc_header;

COMPILER_WORD_ALIGNED uint8_t psd_buffer[PSD_BUFFER_SIZE+4];
float32_t *psd_data = (float32_t *)&psd_buffer[WISPR_DATA_HEADER_SIZE]; // data follows header
wispr_data_header_t psd_header;

#define WISPR_I2C_SPEED 100000UL

// set this only for testing - this will force card swaps 
//uint32_t test_sd_card_nblocks = 300 * ADC_MAX_BLOCKS_PER_BUFFER;
uint32_t test_sd_card_nblocks = 0;

// local function prototypes
uint8_t swap_sd_cards(wispr_config_t *config);
int initialize_sd_cards(wispr_config_t *config);
void initialize_config(wispr_config_t *config);
void change_gain(wispr_config_t *config);
uint32_t initialize_datetime(void);
uint32_t initialize_datetime_with_gps(void);

// local variables
wispr_com_msg_t com_msg;
char com_buf[COM_MAX_MESSAGE_SIZE];

//
// main
//
int main (void)
{
	wispr_config_t wispr; // current configuration

	// initialize global variables
	wispr.active_sd_card = 0; // no active sd card number yet
	
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
	// This supplies the 32k clock to the internal rtc
	if( ds3231_init() != STATUS_OK ) {
		printf("Error initializing DS3231 RTC\r\n");
	}
	
	// initialize the uart com communications port
	com_init(BOARD_COM_PORT, BOARD_COM_BAUDRATE);

	// start the pps timer and synchronize the rtc with the pps input
	if ( pps_timer_init() != RTC_STATUS_OK ) {
		printf("RTC Sync failed\r\n");
	}

	//pps_timer_calib();

	// synchronize the rtc initialization with the pps
	//pps_timer_sync( initialize_datetime_with_gps );
	if ( pps_timer_sync( initialize_datetime ) != RTC_STATUS_OK ) {
		printf("RTC Sync failed\r\n");
	}

	// check all sd cards and configure them if needed
	// this also sets the active sd card number 
	if( initialize_sd_cards(&wispr) == 0) {
		printf("SD Cards failed to initialize\n\r");
		return(0);
	}
	
	// read the configuration from the active sd card
	sd_card_open(wispr.active_sd_card);
	printf("\r\nStartup using ");
	sd_card_print_info(wispr.active_sd_card);
	
	// read the configuration from the active sd card
	sd_card_read_config(wispr.active_sd_card, &wispr);
	sd_card_close(wispr.active_sd_card);

	// if user reset then prompt to change/initialize config
	// skip if reset from backup because there typically will be no user at the console
	// this means that the configuration can only be changed after a user reset or power up
	if( reset_type != BOARD_BACKUP_RESET ) {
		initialize_config(&wispr);
	}

	// Define the variables that control the window and interval timing.
	uint16_t samples_per_adc_buffer = wispr.samples_per_buffer;
	float adc_buffer_duration =  (float)wispr.samples_per_buffer / (float)wispr.sampling_rate; // seconds

	// number of 512 bytes blocks to write for each adc buffer
	uint16_t adc_nblocks = wispr.buffer_size / WISPR_SD_CARD_BLOCK_SIZE;
	// number of 512 bytes blocks to write for each psd buffer
	// this is a fixed number now, but could be variable to save card memory
	uint16_t psd_nblocks = PSD_BLOCKS_PER_BUFFER;
		
	// Open the active sd card
	if( sd_card_open(wispr.active_sd_card) == 0 ) {
		printf("SD Card %d failed to open\n\r", wispr.active_sd_card);
	}
	printf("\n\r");

	// setup INA260 power monitor
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
	
	// Initialize spectrum
	if( wispr.mode & WISPR_SPECTRUM ) {
		uint16_t nbins = wispr.fft_size / 2;
		spectrum_init_q31(&nbins, wispr.fft_size, wispr.fft_overlap, wispr.fft_window_type);
	}
	
	printf("\n\rStart continuous data acquisition with %f second buffers (%d samples each)\n\r", 
		adc_buffer_duration, samples_per_adc_buffer);
	
	// initialize the adc with the current config
	ltc2512_init(&wispr, &adc_header);
	
	// start adc - this starts the receiver and conversion clock, but doesn't trigger the adc
	ltc2512_start();

	// Trigger the adc by syncing with the pps timer.
	// This will call ltc2512 trigger function on the next pps rising edge.
	uint32_t start_sec = 0;
	start_sec = pps_timer_sync( ltc2512_trigger );
	
	// enter read loop
	int go = 1;
	uint16_t count = 0;
	uint32_t prev_usec = 0;
		
	while (go) {
	
		// check for a com message, no wait timeout 
		int nrd = com_read_msg (BOARD_COM_PORT, com_buf, 0);
		if( nrd > 0) {
			com_parse_msg(&com_msg, com_buf, nrd);
			printf("com message received: %s\r\n", com_buf);
		}
		
		// read the current a buffer. If a new buffer is not ready read returns 0
		uint16_t nsamps = ltc2512_read_dma(&adc_header, adc_data);
	
		// if a new buffer is available
		if( nsamps == samples_per_adc_buffer ) {
				 
			// reset the wdt every time a buffer is read
			wdt_restart(WDT);
		
			// calculate the spectrum and save it to the active sd card
			if( wispr.mode & WISPR_SPECTRUM ) {
				
				// call spectrum function
				spectrum_q31(&psd_header, psd_data, &adc_header, adc_data, nsamps);
					
				// check if active sd card is full
				if( sd_card_is_full(wispr.active_sd_card, psd_nblocks) ) {
					// toggle between the available sd cards
					swap_sd_cards(&wispr);
				}
				
				// serialize the buffer header - write the latest adc header onto the front of the buffer
				wispr_serialize_data_header(&psd_header, psd_buffer);

				// write the buffer - which includes header and data
				if( sd_card_write(wispr.active_sd_card, psd_buffer, psd_nblocks) == 0 ) {
					printf("sd_card_write: failed\n\r");
				}
				
			}

			//write the waveform to the active sd card 				
			if( wispr.mode & WISPR_WAVEFORM ) {
				
				// check if active sd card is full
				if( sd_card_is_full(wispr.active_sd_card, adc_nblocks) ) {
					// toggle between the available sd cards
					swap_sd_cards(&wispr);
				}
				
				// serialize the buffer header - write the latest adc header onto the front of the buffer
				wispr_serialize_data_header(&adc_header, adc_buffer);

				// write the buffer - which includes header and data
				if( sd_card_write(wispr.active_sd_card, adc_buffer, adc_nblocks) == 0 ) {
					printf("sd_card_write: failed\n\r");
				}

			}
						
			//ina260_read_power(&amps, &volts);
			//printf("ina260: mA = %lu, mV = %lu\r\n", amps, volts);
		
			// sleep between dma buffers, the next dma interrupt will wake from sleep
			pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);

		}

		// handle com actions
		switch (com_msg.type) {
			case COM_SLEEP:
			  go_to_sleep();
			  break;
			case COM_EXIT:
			  go = 0;
			  break;
			case COM_:
			  go = 0;
			  break;
		}

		
	}
	
	ltc2512_stop();
	printf("Finished\n\r");
	
	exit(0);
}


uint8_t swap_sd_cards(wispr_config_t *config)
{	
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
	if( test_sd_card_nblocks > 0 ) {
		
		// grow the card size - used for testing
		uint32_t nblocks = sd_card_get_number_of_blocks(config->active_sd_card);
		sd_card_set_number_of_blocks(config->active_sd_card, nblocks + test_sd_card_nblocks);

		uint32_t epoch = 0;
		rtc_get_epoch(&epoch);
		printf("\r\nSwitch to Card %d at %s\r\n", config->active_sd_card, epoch_time_string(epoch));

		//sd_card_print_info(wispr.active_sd_card);

	}

	// update the config time and save it to preserve the active card number
	rtc_get_epoch(&config->epoch);
	sd_card_write_config(config->active_sd_card, config);

	return(config->active_sd_card);
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
	
	// flush the uarts
	while (!uart_is_tx_empty(UART1)) {}

	/* Switch MCK to slow clock  */
	pmc_switch_mck_to_sclk(PMC_MCKR_PRES_CLK_1);

	/* Disable unused clock to save power */
	pmc_osc_disable_xtal(1);
	pmc_disable_pllack();
	pmc_disable_all_periph_clk();

	// Enter into backup mode
	pmc_enable_backupmode();

}


// 
// Open each sd card and check the free storage.
// Stop and return when a card has space, making that card active.
// Exits with active card mounted and config updated with values from the active card.
//
int initialize_sd_cards(wispr_config_t *config)
{
	//uint8_t active_sd_card = wispr.active_sd_card;
	uint8_t state;
	wispr_config_t config1, config2;
	
	// open card 2, format if needed, and get time
	state = sd_card_open(2);
	if( !(state & SD_CARD_FORMATED) ) {
		printf("SD Card 2 is unformatted.\r\n");
		if( console_prompt_int("Reformat SD Card 2?", 1, 10) ) {
			sd_card_format(2, "Card2");
		}
		wispr_config_set_default(&config2);
		sd_card_write_config(2, &config2);
	}
	if( state == SD_CARD_READY ) {
		sd_card_read_config(2, &config2);
		} else {
		printf("SD Card 2 failed to initialize\r\n");
	}
	sd_card_close(2);
	
	// open card 1, format if needed, and get time
	state = sd_card_open(1);
	if( !(state & SD_CARD_FORMATED) ) {
		printf("SD Card 1 is unformatted.\r\n");
		if( console_prompt_int("Reformat SD Card 1?", 1, 10) ) {
			sd_card_format(1, "Card1");
		}
		wispr_config_set_default(&config1);
		sd_card_write_config(1, &config1);
	}
	if ( state == SD_CARD_READY ) {
		sd_card_read_config(1, &config1);
		} else {
		printf("SD Card 1 failed to initialize\r\n");
	}
	sd_card_close(1);
	
	// select the active card based on the config time

	config->active_sd_card = 1;

	// use card 2 if it's config is newer
	// this assumes that the card 2 was last active
	if(config2.epoch > config1.epoch) config->active_sd_card = 2;

	return((int)config->active_sd_card);
}

void initialize_config(wispr_config_t *config)
{
	uint8_t card_num = config->active_sd_card;
	//card_num = (uint8_t)console_prompt_int("Enter active SD card number", (int)card_num, 5);
	//wispr.active_sd_card = card_num;
	
	// read config from the specified card
	uint8_t state = sd_card_open(card_num);
	if( !(state & SD_CARD_READY) ) {
		printf("\r\nCan't read configuration from SD card %d, using defaults.\r\n", card_num);
		wispr_config_set_default(config);
	} else {
		printf("\r\nRead configuration from SD card %d.\r\n", card_num);
		sd_card_read_config(card_num, config);
	}
	
	// display the config
	wispr_print_config(config);
	
	// Prompt to set new configuration
	while(1) {
		if( console_prompt_int("Change configuration?", 0, 8) ) {
			wispr_config_menu(config, 60);
		} else break;
		wispr_config_print(config);
	}
	
	// save the new config and close the card
	sd_card_write_config(card_num, config);
	sd_card_close(card_num);
	
	// Prompt to reset the cards to start writing and reading from the beginning
	// this will overwrite the existing data on the card
	int reset_sd_card = 0;
	reset_sd_card = console_prompt_int("Reformat and overwrite data on both SD cards", reset_sd_card, 8);

	// this is mainly for testing
	if(reset_sd_card) {

		if( console_prompt_int("Are you sure you want to reformat both SD cards", 0, 8) ) {
			
			sd_card_format(2,"Card2");
			sd_card_open(2);
			sd_card_write_config(2, config);
			if(test_sd_card_nblocks > 0) sd_card_set_number_of_blocks(2, test_sd_card_nblocks);
			sd_card_print_info(2);
			sd_card_close(2);

			sd_card_format(1,"Card1");
			sd_card_open(1);
			sd_card_write_config(1, config);
			if(test_sd_card_nblocks > 0) sd_card_set_number_of_blocks(1, test_sd_card_nblocks);
			sd_card_print_info(1);
			sd_card_close(1);
			
			config->active_sd_card = 1;
			
		}
	}
}


uint32_t initialize_datetime(void)
{
	uint32_t status;
	rtc_time_t dt;

	// read and display the external rtc time
	uint32_t rtc_status = ds3231_get_datetime(&dt);
	if ( rtc_status != RTC_STATUS_OK ) {
		printf("DS3231 RTC failed, status %d\r\n", rtc_status);
		//rtc_status = ds3231_get_datetime(&dt);
	}
	
	// Initialize the internal RTC using the external RTC time
	status = rtc_init(&dt);
	while ( rtc_status != RTC_STATUS_OK ) {
		printf("Waiting for RTC, status %d\r\n", status);
		status = rtc_init(&dt);
	}
	printf("\r\n");
	printf("RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n", 
		dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
	printf("\r\n");

	return(status);
}

uint32_t initialize_datetime_with_gps(void)
{
	uint32_t status;
	uint16_t timeout=10000; //10 sec wait for COM0 input
	rtc_time_t dt;
	
	// request a gps message
	int nrd = com_request_gps(&com_msg, timeout);
	
	// if a gps message is received use it to set the time		
	if(nrd > 0) {
		//Convert epoch time to RTC datetime format
		epoch_to_rtc_time(&dt, com_msg.sec);
	} else { // else NO GPS time available. Sync RTC by DS3231,
		printf("No GPS message received from COM0. Sync int RTC by DS3231\n\r");
		// read ds3231 time
		ds3231_get_datetime(&dt);
	}
	
	// Initialize the internal RTC using ds3231 RTC time
	status = rtc_init(&dt);
	while ( status != RTC_STATUS_OK ) {
		printf("Waiting for RTC, status %d\r\n", status);
		status = rtc_init(&dt);
	}
	printf("RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n",
		dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
	printf("\r\n");
	
	return(status);
}


