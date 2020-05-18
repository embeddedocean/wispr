/*
 * wispr_example1.c
 *
 * - Intermittent data acquisition where adc samples are acquired continuously for a finite time window 
 *   then the system goes to sleep. The system wakes up and reboots after a specified amount of time 
 *   or is waken up by detecting an input on one of the serial ports.
 * - Data is logged to flat binary data files on the 
 * - exFat formatted SD card. 
 * - No data headers are written to the binary files.
 * - Data buffer header info is written to a separate text file (.txt) with the same name as the data files. 
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

// Allocate fixed size buffers
// Using the COMPILER_WORD_ALIGNED macro will avoid any memory alignment problem,
// although the compiler will still give the warning.

COMPILER_WORD_ALIGNED uint8_t adc_buffer[ADC_BUFFER_SIZE+4];
uint8_t *adc_data = adc_buffer; 
wispr_data_header_t adc_header;

COMPILER_WORD_ALIGNED uint8_t psd_buffer[PSD_BUFFER_SIZE+4];
float32_t *psd_data = (float32_t *)&psd_buffer[0];
wispr_data_header_t psd_header;

#define WISPR_I2C_SPEED 100000UL

// set this to a nonzero value only for testing - this will force card swaps 
//uint32_t test_sd_card_nblocks = 300 * ADC_MAX_BLOCKS_PER_BUFFER;
uint32_t test_sd_card_nblocks = 0;

// local function prototypes
void go_to_sleep(void);
uint8_t swap_sd_cards(void);
void format_sd_cards(void);
int initialize_sd_cards(void);
void set_default_config(void);
void initialize_config(void);
void prompt_config_menu(int timeout);
void make_filename(char *name, char *prefix, char *suffix);
void process_spectrum(char *dat_filename, char *psd_filename, uint16_t nbufs);
uint32_t initialize_datetime(void);
uint32_t initialize_datetime_with_gps(void);
void change_gain(void);
FRESULT create_data_header_file(char *filename, wispr_config_t *cfg, wispr_data_header_t *hdr);

// local variables
wispr_config_t wispr; // current configuration
char config_filename[] = "wispr1.txt";
wispr_com_msg_t com_msg;
char com_buf[COM_MAX_MESSAGE_SIZE];
fat_file_t dat_file;
fat_file_t psd_file;

//
// main
//
int main (void)
{
	// initialize global variables
	wispr.active_sd_card = 0; // no active sd card number yet
	dat_file.state = SD_FILE_CLOSED;
	psd_file.state = SD_FILE_CLOSED;
	
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
	// This supplies the 32k clock to the internal rtc and a 1Hz pps
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
	
	//int m = 10;
	//while(m--) {
	//	uint32_t sec, usec;
	//	pps_timer_read(&sec, &usec);
	//	printf("time = %f\r\n", (float)sec + (float)usec * 0.000001 );
	//	delay_ms(100);
	//	wdt_restart(WDT);
	//}
	
	// check all sd cards and configure them if needed
	// this also sets the active sd card number 
	FRESULT res = initialize_sd_cards();
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
		initialize_config();
	}

	// set the fixed configuration values
	wispr.sample_size = ADC_SAMPLE_SIZE;
	wispr.samples_per_buffer = ADC_MAX_SAMPLES_PER_BUFFER;
	wispr.fft_size = PSD_FFT_SIZE;

	// Define the variables that control the window and interval timing.
	uint16_t samples_per_adc_buffer = wispr.samples_per_buffer;
	float adc_buffer_duration =  (float)wispr.samples_per_buffer / (float)wispr.sampling_rate; // seconds
	uint16_t adc_buffers_per_window = (uint16_t)( (float)wispr.acquisition_time / adc_buffer_duration ); 
	// since the adc buffer duration is defined by a fixed number of blocks
	// the actual sampling window may be different than the requested
	float actual_sampling_time = (float)adc_buffers_per_window * adc_buffer_duration;

	wispr.buffers_per_window = adc_buffers_per_window;
	wispr.file_size = adc_buffers_per_window;

	// save the updated config
	sd_card_fwrite_config(config_filename, &wispr);

	// save the config because it could have changed
	wispr_print_config(&wispr);
	
	// setup INA260 power monitor
	uint32_t volts; // mVolts
	int32_t amps;   // mAmps
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
		
	// number of 512 bytes blocks to write for each adc buffer
	uint16_t adc_write_size = wispr.buffer_size / WISPR_SD_CARD_BLOCK_SIZE;

	// check if active sd card is full
	if( sd_card_is_full(wispr.active_sd_card, adc_write_size) ) {
		// close all log files
		sd_card_fclose(&dat_file);
		sd_card_fclose(&psd_file);
		// toggle between the available sd cards
		swap_sd_cards();
	}
	
	// open a new data file
	char dat_filename[32];
	char hdr_filename[32];
	rtc_time_t time;
	ds3231_get_datetime(&time);  // get current time
	sprintf(dat_filename, "%WISPR_%02d%02d%02d_%02d%02d%02d.dat", time.year, time.month,time.day, time.hour, time.minute, time.second);
	sprintf(hdr_filename, "%WISPR_%02d%02d%02d_%02d%02d%02d.txt", time.year, time.month,time.day, time.hour, time.minute, time.second);
	
	if( sd_card_fopen(&dat_file, dat_filename, FA_OPEN_ALWAYS | FA_WRITE, wispr.active_sd_card) == FR_OK ) {
		printf("Creating new data file: %s\r\n", dat_filename);
	}
	
	printf("\n\rStart data acquisition for %.3f seconds (%d buffers)\n\r", actual_sampling_time, wispr.buffers_per_window);
	
	// initialize the adc with the current config
	ltc2512_init(&wispr, &adc_header);
	
	// start adc - this starts the receiver and conversion clock, but doesn't trigger the adc 
	ltc2512_start();

	// Trigger the adc by syncing with the pps timer.
	// This will call ltc2512 trigger function on the next pps rising edge.
	uint32_t start_sec = 0;
	start_sec = pps_timer_sync( ltc2512_trigger );
	
	// loop over adc read buffers in the sampling window
	uint16_t count = 0;
	while ( count < adc_buffers_per_window ) {
		
		// check for a com message, no wait timeout 
		int nrd = com_read_msg (BOARD_COM_PORT, com_buf, 0);
		if( nrd > 0) {
			com_parse_msg(&com_msg, com_buf, nrd);
			printf("com message received: %s\r\n", com_buf);
		}
		
		// read the current buffer. If a new buffer is not ready read returns 0
		uint16_t nsamps = ltc2512_read_dma(&adc_header, adc_data);
		
		// if a new buffer is available
		if( nsamps == samples_per_adc_buffer ) {
			
			// reset the wdt every time a buffer is read
			wdt_restart(WDT);
			
			// serialize the buffer header - write the latest adc header onto the front of the buffer
			//wispr_serialize_data_header(&adc_header, adc_buffer);
			
			// write the adc buffer - both header and data
			if( sd_card_fwrite(&dat_file, adc_buffer, adc_write_size) != FR_OK ) {
				printf("Error writing to file: %s\r\n", dat_file.name);
			}
			
			//printf("usec = %d, delta = %d\r\n", adc_header.usec, adc_header.usec-prev_usec);
			//prev_usec = adc_header.usec;
			
			// increment buffer count
			count++;
			
		}
			
		//ina260_read_power(&amps, &volts);
		//printf("ina260: mA = %lu, mV = %lu\r\n", amps, volts);
		
		// sleep between dma buffers, the next dma interrupt will wake from sleep
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);

	}
	
	sd_card_fclose(&dat_file);
	
	// create a header file with the logging parameters and start time
	adc_header.second = start_sec; // adc trigger start time
	adc_header.usec = 0; // this is zero because the start trigger was synced to the pps edge
	sd_card_fwrite_header(hdr_filename, &wispr, &adc_header);
	
	ltc2512_stop();
	ltc2512_stop_dma();
	
	// run spectrum analysis on the data just collected
	if( wispr.mode & WISPR_SPECTRUM ) {
		char psd_filename[32];
		sprintf(psd_filename, "%WISPR_%02d%02d%02d_%02d%02d%02d.psd", time.year, time.month,time.day, time.hour, time.minute, time.second);
		process_spectrum(dat_filename, psd_filename, adc_buffers_per_window);
	}
	
	// go to deep sleep 
	if( wispr.sleep_time > 0 ) {

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
		sd_card_fwrite_config(config_filename, &wispr);
				
		// close active sd card
		sd_card_unmount(wispr.active_sd_card);
		
		// set the alarm to wakeup for the next window read cycle
		rtc_time_t dt;
		epoch_to_rtc_time(&dt, now + wispr.sleep_time);
		ds3231_set_alarm(&dt);
		printf("\r\nEntering sleep mode for %d seconds, wakeup alarm set for %s\r\n", 
			wispr.sleep_time, epoch_time_string(now + wispr.sleep_time));

		// initialize the wakeup signal on PA02
		board_init_wakeup();

		// enter backup mode sleep
		go_to_sleep();	// The core will reset when exiting from backup mode. 
		
	}
	
	printf("Finished\n\r");
	
	exit(0);
}

void process_spectrum(char *dat_filename, char *psd_filename, uint16_t nbufs)
{	
	printf("\r\nProcessing spectrum and creating file %s\r\n", psd_filename);
	
	// Initialize spectrum
	uint16_t nbins = wispr.fft_size / 2;
	spectrum_init_q31(&nbins, wispr.fft_size, wispr.fft_overlap, wispr.fft_window_type);
	
	//pdc_buffer_locked = 1;
	
	// open the data file
	if( sd_card_fopen(&dat_file, dat_filename, FA_OPEN_ALWAYS | FA_READ, wispr.active_sd_card) != FR_OK ) {
		printf("Error opening data file: %s\r\n", dat_file.name);
	}

	// open a new psd file
	if( sd_card_fopen(&psd_file, psd_filename, FA_OPEN_ALWAYS | FA_WRITE, wispr.active_sd_card) != FR_OK ) {
		printf("Error opening data file: %s\r\n", psd_file.name);
	}
	
	uint16_t adc_nsamps = wispr.samples_per_buffer;
	uint16_t adc_nblocks = ADC_BLOCKS_PER_BUFFER;
	uint16_t psd_nblocks = PSD_BLOCKS_PER_BUFFER;
	
	for(int n = 0; n < nbufs; n++) {
	
		// write the adc buffer - both header and data
		if( sd_card_fread(&dat_file, adc_buffer, adc_nblocks) != FR_OK ) {
			printf("Error writing to file: %s\r\n", dat_file.name);
		}
		 
		// call spectrum function
		spectrum_q31(&psd_header, psd_data, &adc_header, adc_data, adc_nsamps);
					
		// serialize the buffer header - write the latest header onto the front of the buffer
		//wispr_serialize_data_header(&psd_header, psd_buffer);
		
		// write the psd buffer, which contains both header and data
		if( sd_card_fwrite(&psd_file, psd_buffer, psd_nblocks) != FR_OK ) {
			printf("Error writing to file: %s\r\n", psd_file.name);
		}
					
	}
	
	sd_card_fclose(&dat_file);
	sd_card_fclose(&psd_file);

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

uint8_t swap_sd_cards(void)
{	
	if( wispr.active_sd_card == 1 ) {
		sd_card_unmount(1);
		sd_card_mount(2);
		wispr.active_sd_card = 2;
	}
	else if( wispr.active_sd_card == 2 ) {
		sd_card_unmount(2);
		sd_card_mount(1);
		wispr.active_sd_card = 1;
	} else {
		printf("SD Card Swap Failed\n\r");
		return(0);
	}

	// for testing only
	if( test_sd_card_nblocks > 0 ) {
		
		// grow the card size - used for testing
		uint32_t nblocks = sd_card_get_number_of_blocks(wispr.active_sd_card);
		sd_card_set_number_of_blocks(wispr.active_sd_card, nblocks + test_sd_card_nblocks);

		uint32_t epoch = 0;
		rtc_get_epoch(&epoch);
		printf("\r\nSwitch to Card %d at %s\r\n", wispr.active_sd_card, epoch_time_string(epoch));

		//sd_card_print_info(wispr.active_sd_card);

	}

	// update the config time and save it to preserve the active card number
	rtc_get_epoch(&wispr.epoch);
	sd_card_fwrite_config(config_filename, &wispr);

	return(wispr.active_sd_card);
}

void set_default_config(void)
{
	// set config mod time
	rtc_get_epoch(&wispr.epoch);
	
	wispr.version[1] = WISPR_VERSION;
	wispr.version[0] = WISPR_SUBVERSION;
	
	wispr.buffer_size = ADC_BLOCKS_PER_BUFFER * SD_MMC_BLOCK_SIZE;
	wispr.sample_size = ADC_SAMPLE_SIZE;
	wispr.sampling_rate = ADC_DEFAULT_SAMPLING_RATE;

	wispr.acquisition_time = ADC_DEFAULT_AWAKE;
	wispr.sleep_time = ADC_DEFAULT_SLEEP;

	wispr.samples_per_buffer = (uint32_t)(wispr.buffer_size - WISPR_DATA_HEADER_SIZE) / (uint32_t)wispr.sample_size;

	wispr.gain = ADC_DEFAULT_GAIN; // adc gain
	wispr.adc_decimation = LTC2512_DF8; // adc df

	wispr.state = 0; //
	wispr.mode = WISPR_WAVEFORM; //

	wispr.fft_size = 1024;
	wispr.fft_overlap = 0;
	wispr.fft_window_type = RECT_WINDOW; 
	
	wispr.file_size = WISPR_MAX_FILE_SIZE;  // about 50Mb
	
	wispr.active_sd_card = 1;
}

// 
// Open each sd card and check the free storage.
// Stop and return when a card has space, making that card active.
// Exits with active card mounted and config updated with values from the active card.
//
int initialize_sd_cards(void)
{
	FRESULT res;
	
	// loop over all the cards 
	for(int n = 1; n <= NUMBER_SD_CARDS; n++) {
	
		// mount card
		res = sd_card_mount(n);

		// if problem mounting, then continue to next card
		if (res != FR_OK) {
			if ( res == FR_NO_FILESYSTEM ) {		
				printf("initialize_sd_cards: card %d if unformatted\r\n", n);
			} else {
				printf("initialize_sd_cards: card %d error %d\r\n", n, res);
			}
			res = sd_card_unmount(n);
			continue;
		}
		
		// check free space on card
		// continue to next card if there's not enough
		if( sd_card_free_space(n) < 32 ) {
			printf("initialize_sd_cards: card %d is full\r\n", n);
			res = sd_card_unmount(n);
			continue;
		}

		// read the configuration
		res = sd_card_fread_config(config_filename, &wispr);
		if( res != FR_OK) {
			if( res == FR_NO_FILE) {
				// set defaults config and write it 
				set_default_config();
				if( sd_card_fwrite_config(config_filename, &wispr) == FR_OK ) {
					printf("Default configuration set and written to card %d\r\n", n);
				}
			} else {			
				printf("initialize_sd_cards: error %d reading config from card %d\r\n", res, n);
			}
		}
		
		// use this card and break from loop
		wispr.active_sd_card = n;
		
		//printf("initialize_sd_cards: card %d selected\r\n", n);
		
		break;
		
	}
	return(res);
}

void initialize_config(void)
{		
	//uint8_t card_num = wispr.active_sd_card;
	// writes config to whatever card that is currently mounted	

	// display the config
	wispr_print_config(&wispr);
			
	// Prompt to set new configuration
	while(1) {
		if( console_prompt_int("Change configuration?", 0, 8) ) {
			prompt_config_menu(60);
		} else break;	
		wispr_print_config(&wispr);
	}
	
	// save the new config and close the card
	sd_card_fwrite_config(config_filename, &wispr);

}

void prompt_config_menu(int timeout)
{	
	uint32_t u32;
	uint16_t u16;
	uint8_t u8;
	
	wispr.version[1] = WISPR_VERSION;
	wispr.version[0] = WISPR_SUBVERSION;
	
	wispr.buffer_size = ADC_BLOCKS_PER_BUFFER * SD_MMC_BLOCK_SIZE;
	wispr.samples_per_buffer = (wispr.buffer_size - WISPR_DATA_HEADER_SIZE) / 3;
	
	uint16_t blocks_per_buffer = ADC_BLOCKS_PER_BUFFER;
	//blocks_per_buffer = console_prompt_uint32("Enter number of blocks (512 bytes) per buffer", blocks_per_buffer, timeout);
	
	// commented out for fixed sample size 
	//u8 = console_prompt_uint8("Enter sample size in bytes", wispr.sample_size, timeout);
	//if( u8 >= 2 && u8 <= 3 ) wispr.sample_size = u8;
	wispr.sample_size = ADC_SAMPLE_SIZE;
	printf("\r\nFixed sample size: %d bytes\r\n", wispr.sample_size);

	u32 = console_prompt_uint32("Enter sampling rate in Hz", wispr.sampling_rate, timeout);
	if( u32 > 0 && u32 <= 350000 ) wispr.sampling_rate = u32;

	u8 = wispr.gain;
	u8 = console_prompt_uint8("Enter preamp gain setting (0 to 4)", u8, timeout);
	if( u8 >= 0 && u8 <= 4 ) wispr.gain = u8;

	u8 = wispr.adc_decimation;
	u8 = console_prompt_uint8("Enter adc decimation factor (4, 8, 16, or 32)", u8, timeout);
	if( u8 == 4 || u8 == 8 || u8 == 16 || u8 == 32) wispr.adc_decimation = u8;

	// prompt for sampling interval
	u16 = wispr.acquisition_time;
	u16 = console_prompt_uint16("Enter sampling time window in seconds", wispr.acquisition_time, timeout);
	if( u16 >= 1 ) wispr.acquisition_time = u16;
	u16 = console_prompt_uint16("Enter sleep time between sampling windows in seconds", wispr.sleep_time, timeout);
	if( u16 >= 0 ) wispr.sleep_time = u16;
	
	// update variables based on new input
	wispr.buffer_size = (uint16_t)(blocks_per_buffer * SD_MMC_BLOCK_SIZE);
	wispr.samples_per_buffer = (wispr.buffer_size - WISPR_DATA_HEADER_SIZE) / (uint16_t)wispr.sample_size;
	float adc_buffer_duration =  (float)wispr.samples_per_buffer / (float)wispr.sampling_rate; // seconds
	wispr.buffers_per_window = (uint16_t)( (float)wispr.acquisition_time / adc_buffer_duration ); // truncated number of buffers
	
	wispr.state = WISPR_READY;
	uint8_t mode = 0;

	// prompt file file
	//u32 = wispr.file_size;
	//u32 = console_prompt_int("Enter max size of data file in blocks", u32, timeout);
	//wispr.file_size = u32;
	wispr.file_size = wispr.buffers_per_window;
	
	// prompt to record waveform data
	int record_waveform = 0;
	if(wispr.mode & WISPR_WAVEFORM) record_waveform = 1;	
	if( console_prompt_int("Record_waveform?", record_waveform, timeout) ) {
		mode |= WISPR_WAVEFORM;
	}

	// prompt for spectrum parameters
	int record_spectrum = 0;
	if(wispr.mode & WISPR_SPECTRUM) record_spectrum = 1;
	if( console_prompt_int("Record spectrum?", record_spectrum, timeout) ) {
	
		//u16 = wispr.fft_size;
		//u16 = console_prompt_uint16("Enter fft size (32, 64, 126, 512 or 1024)", u16, timeout);
		//wispr.fft_size = u16;
		wispr.fft_size = PSD_FFT_SIZE;
		printf("\r\nFixed fft size: %d\r\n", wispr.fft_size);
	
		u16 = wispr.fft_overlap;
		u16 = console_prompt_uint16("Enter fft overlap size", u16, timeout);	
		wispr.fft_overlap = u16;

		u8 = wispr.fft_window_type;
		u8 = console_prompt_uint8("Enter fft window type (0=Rect, 1=Hamming)", u8, timeout);
		wispr.fft_window_type = u8;
	
		mode |= WISPR_SPECTRUM;
	} 	
	
	//psd_nfft = wispr.fft_size;
	//psd_nbins = wispr.fft_size / 2;
	//psd_overlap = wispr.fft_overlap;

	// set the new mode
	wispr.mode = mode;
	
	// prompt to  reset the time
	if( console_prompt_int("Enter new time?", 0, timeout) ) {
		int go = 1;
		rtc_time_t dt;
		ds3231_get_datetime(&dt);  // get current time
		while(go) {
			dt.year = console_prompt_uint8("Enter year in century (0 to 99)", dt.year, timeout);
			dt.month = console_prompt_uint8("Enter month (1 to 12)", dt.month, timeout);
			dt.day = console_prompt_uint8("Enter day (1 to 31)", dt.day, timeout);
			dt.hour = console_prompt_uint8("Enter hour (0 to 23)", dt.hour, timeout);
			dt.minute = console_prompt_uint8("Enter minute (0 to 59)", dt.minute, timeout);
			dt.second = console_prompt_uint8("Enter second (0 to 59)", dt.second, timeout);
			// set the internal RTC 
			uint32_t status = rtc_init(&dt);
			if( status != RTC_STATUS_OK ) {
				printf("Failed to initialize RTC with new time\r\n");
				rtc_print_error(status);
				continue;
			}
			// set the external RTC
			status = ds3231_set_datetime(&dt);
			if( status != RTC_STATUS_OK) {
				printf("Failed to initialize DS3231 with new time\r\n");
				rtc_print_error(status);
				continue;
			}
			break;
		}
		ds3231_get_datetime(&dt);  // read back time
		printf("\r\nExternal RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n",
			dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
	}
	
	// set config mod time
	rtc_get_epoch(&wispr.epoch);
	
}


uint32_t initialize_datetime(void)
{
	uint32_t status;
	rtc_time_t dt;

	// read and display the external rtc time
	status = ds3231_get_datetime(&dt);
	if ( status != RTC_STATUS_OK ) {
		printf("DS3231 RTC failed, status %d\r\n", status);
		//rtc_status = ds3231_get_datetime(&dt);
	}
	//printf("External RTC time is %02d/%02d/%02d %02d:%02d:%02d\r\n", dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
	
	// Initialize the internal RTC using the external RTC time
	status = rtc_init(&dt);
	while ( status != RTC_STATUS_OK ) {
		printf("Waiting for RTC, status %d\r\n", status);
		status = rtc_init(&dt);
	}
	printf("\r\nRTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n", dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);

	return(status);
}

uint32_t initialize_datetime_with_gps(void)
{
	uint32_t status;
	uint16_t timeout=1000; // wait for COM0 input
	rtc_time_t dt;
	
	//Set RTC time by GPS epoch sec received at COM0. Gain is also changed. HM
	printf("Requesting GPS message to set DS3231 & RTC.\r\n");
	com_msg.lat = 0.0; com_msg.lon = 0.0;
	//Sends $GPS* que to MPC as a request for GPS time and Location
	com_write_msg(BOARD_COM_PORT, "GPS");
	//reply $GPS,1588589815,-35.000000,19.000000*
	if( ds3231_init() != STATUS_OK ) {
		printf("Error initializing DS3231 RTC\r\n");
	}
		
	int nrd = com_read_msg (BOARD_COM_PORT, com_buf, timeout);
		
	if(nrd > 0) {
		
		com_parse_msg(&com_msg, com_buf, nrd);
		epoch_to_rtc_time(&dt, com_msg.sec); //Convert epoch time to RTC datetime format
		//ds3231_epoch_to_datetime(&dt, com_msg.sec); //HM year is wrong. Use epoch_rtc_time()
		// set DS3231 RTC using epoch time received from GPS
		status = ds3231_set_datetime(&dt);
		status = rtc_init(&dt);
		printf("\r\nResetting DS3231 and built-in RTC successful\n\r");
		printf("lat=%f, lon=%f\n\r", com_msg.lat, com_msg.lon); //debug Remove this
		//status = ds3231_get_datetime(&dt);  // read back time
		status = rtc_get_datetime(&dt);
		//printf("DS3231 RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n", dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
		printf("RTC time %02d/%02d/%02d %02d:%02d:%02d\r\n", dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
			
	} else {//NO GPS time available. Sync RTC by DS3231,
	
		printf("No GPS message received from COM0. Sync int RTC by DS3231\n\r");
			
		// read and display the ds3231 time
		ds3231_get_datetime(&dt);
		//printf("\r\n");
		//printf("External RTC time is %02d/%02d/%02d %02d:%02d:%02d\r\n",
		//	dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
			
		// Initialize the internal RTC using ds3231 RTC time
		status = rtc_init(&dt);
		while ( status != RTC_STATUS_OK ) {
			printf("Waiting for RTC, status %d\r\n", status);
			status = rtc_init(&dt);
		}
		printf("\r\nRTC set to  %02d/%02d/%02d %02d:%02d:%02d\r\n",
			dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
		printf("\r\n");
	}
	return(status);
}

void change_gain(void)
{
	uint16_t timeout=10000; //10 sec wait for COM0 input
	uint8_t new_gain;
	
	//HM added to check if a gain change is requested
	com_write_msg(BOARD_COM_PORT, "NGN");
	printf("Type at com0 $NGN,1*\r\n");//HM For debug. Remove this
	int nrd = com_read_msg (BOARD_COM_PORT, com_buf, timeout);

	new_gain = ADC_DEFAULT_GAIN;
			
	if(nrd > 0) {
		com_parse_msg(&com_msg, com_buf, nrd);
		//printf("%d\n\r",com_msg.gain);//  HM debug
		//printf("new gain = %d\n\r", com_msg.gain);//HM debug
		if((com_msg.gain <4) && (com_msg.gain >= 0)) {
			new_gain=com_msg.gain;//HM Gain update is ready. config.settings[0] will be updated in initialize_config()
		}
	}

	// set the global config with the new gain 
	wispr.gain = new_gain;

}


