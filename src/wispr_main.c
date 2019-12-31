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

COMPILER_WORD_ALIGNED uint8_t adc_buffer[ADC_BUFFER_SIZE];
uint8_t *adc_buffer_header = adc_buffer;  // header is at start of buffer
uint8_t *adc_buffer_data = &adc_buffer[ADC_HEADER_SIZE]; // data follows header

uint32_t adc_sampling_rate = 50000;
uint8_t adc_preamp_gain = 0;

uint32_t num_blocks_per_adc_buffer = ADC_BLOCKS_PER_BUFFER;
uint16_t adc_num_samples_per_buffer = ADC_NUM_SAMPLES;

// sd card objects
sd_card_t sd_card1;
sd_card_t sd_card2;

// active sd card pointer
sd_card_t *active_sd_card = NULL;

uint32_t sd_card_nblocks = 1024;

// local function prototypes
int swap_sd_cards(void);

//
// main
//
int main (void)
{
	int result = 0;
	int go = 1;

	sysclk_init();
	board_init();
	i2c_init(TWI0, 100000UL);
	
	// Display configuration prompts
	// make sure the wdt doesn't reset while waiting for the prompt to timeout 
	wdt_restart(WDT);
	int timeout = 5;
	fprintf(stdout, "\r\nEnter configuration or hit Enter to accept default values\r\n" );
	fprintf(stdout, "Prompt will timeout in %d seconds.\r\n", timeout );

	adc_sampling_rate = (uint32_t)console_prompt_int("Enter sampling rate in Hz", adc_sampling_rate, timeout);	
	adc_preamp_gain = (uint8_t)console_prompt_int("Enter preamp gain", adc_preamp_gain, timeout);
	
	//// Setup the DS3231 External RTC
	if( ds3231_init() != STATUS_OK ) {
		printf("Error initializing DS3231 RTC\r\n");
	}

	// read and display the time set in the external rtc
	ds3231_datetime  datetime;
	ds3231_get_datetime(&datetime);
	printf("\r\nExternal RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n",
	  datetime.year, datetime.month, datetime.day, datetime.hour, datetime.minute, datetime.second);

	// prompt to  reset the time 
	wdt_restart(WDT);
	if( console_prompt_int("Enter new time?", 0, timeout) ) {
		go = 1;
		while(go) {
		  timeout = 10;
		  wdt_restart(WDT);
		  datetime.year = (uint8_t)console_prompt_int("Enter year", (int)datetime.year, timeout);
		  wdt_restart(WDT);
		  datetime.month = (uint8_t)console_prompt_int("Enter month", (int)datetime.month, timeout);
		  wdt_restart(WDT);
		  datetime.day = (uint8_t)console_prompt_int("Enter day", (int)datetime.day, timeout);
		  wdt_restart(WDT);
		  datetime.hour = (uint8_t)console_prompt_int("Enter hour", (int)datetime.hour, timeout);
		  wdt_restart(WDT);
		  datetime.minute = (uint8_t)console_prompt_int("Enter minute", (int)datetime.minute, timeout);
		  wdt_restart(WDT);
		  datetime.second = (uint8_t)console_prompt_int("Enter second", (int)datetime.second, timeout);
		  // set the external RTC using
		  if( ds3231_set_datetime(&datetime) == TWI_SUCCESS) break;
		}
		ds3231_get_datetime(&datetime);  // read back time
		printf("\r\nExternal RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n",
			datetime.year, datetime.month, datetime.day, datetime.hour, datetime.minute, datetime.second);
	}
	
	// setup INA260
	uint32_t volts; // mVolts
	int32_t amps; // mAmps
	ina260_init();
	ina260_read_power(&amps, &volts); // first read returns zero 

	// Initialize the internal RTC using the external RTC time
	rtc_init((rtc_time_t *)&datetime);
	printf("\r\nInternal RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n",
		datetime.year, datetime.month, datetime.day, datetime.hour, datetime.minute, datetime.second);

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
	// Select and test SD2
	sd_card_select(2);
	if(sd_card_init(&sd_card2) == SD_MMC_OK) {
		active_sd_card = &sd_card2;
		printf("\n\rSD Card 2\n\r");
		sd_card_print_info(&sd_card2);
	} else {
		printf("SD Card 2 Failed\n\r");
	}

	// Select and test SD1 and leave it active to start writing
	sd_card_select(1);
	if(sd_card_init(&sd_card1) == SD_MMC_OK) {
		active_sd_card = &sd_card1;
		printf("\n\rSD Card 1\n\r");
		sd_card_print_info(&sd_card1);
	} else {
		printf("SD Card 1 Failed\n\r");
	}
	printf("\n\r");

	// Prompt for number of blocks to write. This is useful for testing
	wdt_restart(WDT);
	timeout = 5;
	sd_card_nblocks = sd_card_get_number_of_blocks(active_sd_card);
	sd_card_nblocks = (uint32_t)console_prompt_int("Enter number of block to write on SD cards", (int)sd_card_nblocks, timeout);
	sd_card_set_number_of_blocks(active_sd_card, sd_card_nblocks);

	// initialize ADC
	ltc2512_init(&adc_sampling_rate, LTC2512_DF4, adc_preamp_gain);
	adc_num_samples_per_buffer = ltc2512_init_dma();

	float buffer_duration =  (float)adc_num_samples_per_buffer / (float)adc_sampling_rate;

	// start adc conversions and dma
	ltc2512_start_dma();
	ltc2512_start_conversion();	

	uint32_t cpu_sclk = sysclk_get_cpu_hz();
	printf("\r\nCPU clock: %lu MHz\n\r", cpu_sclk/1000000);
	//printf("peripheral bus clock: %lu MHz\n\r",  sysclk_get_peripheral_bus_hz(TC0)/1000000);

	printf("number samples per buffer: %lu\n\r", adc_num_samples_per_buffer);
	printf("sampling rate: %lu \n\r", adc_sampling_rate);
	printf("buffer duration: %lu msec\n\r", (uint32_t)(1000.0*buffer_duration));

	printf("sd start addr: %lu \n\r", active_sd_card->first_block);
	printf("sd end addr: %lu \n\r", active_sd_card->last_block);
	printf("number blocks per buffer: %lu \n\r", num_blocks_per_adc_buffer);

	ina260_read_power(&amps, &volts);
	printf("\r\nina260: mA = %lu, mV = %lu, mW = %lu\r\n", amps, volts, amps*volts/1000);

	//uint32_t wdt_msec = 3000;
	//wdt_start(wdt_msec);
	
	printf("\n\rStart read loop\n\r");

	// throw away the first adc buffer
	while( ltc2512_read_dma(adc_buffer_header, adc_buffer_data) == 0 ) {}

	go = 1;
	while (go) {
		
		uint16_t nsamps = ltc2512_read_dma(adc_buffer_header, adc_buffer_data);
		
		if( nsamps == adc_num_samples_per_buffer ) {

			// reset the wdt every time a buffer is read
			wdt_restart(WDT);

			result = sd_card_write_raw(active_sd_card, adc_buffer, num_blocks_per_adc_buffer);

			if( result != SD_MMC_OK ) {
				printf("sd_card_write_raw: failed 0x%x.\n\r", result);
			}
			
			// if sd card is full
			if( sd_card_is_full(active_sd_card) ) {
				swap_sd_cards(); // toggle between to available sd cards
			}
			
		}
		
		//ina260_read_power(&amps, &volts);
		//printf("ina260: mA = %lu, mV = %lu\r\n", amps, volts);

		// sleep between dma buffers, the next dma interrupt will wake from sleep
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		
	}

	ltc2512_stop_conversion();
	ltc2512_stop_dma();

	printf("Finished\n\r");

	exit(0);
}

//
// swap active sd card
// but make sure that sd_card_get_info() has been called before using the active card
// 
int swap_sd_cards()
{
	int new_card = 0;
	if(active_sd_card == &sd_card1 ) {
		new_card = sd_card_select(2);
		if(sd_card_init(&sd_card2) == SD_MMC_OK) {
			active_sd_card = &sd_card2;
		} else {
			printf("SD Card2 Failed\n\r");
		}
	} 
	else if(active_sd_card == &sd_card2 ) {
		new_card = sd_card_select(1);
		if( sd_card_init(&sd_card1) == SD_MMC_OK ) {
			active_sd_card = &sd_card1;
		} else {
			printf("SD Card1 Failed\n\r");
		}
	} else {
		printf("SD Card Swap Failed\n\r");
	}
	
	sd_card_set_number_of_blocks(active_sd_card, sd_card_nblocks);

	rtc_time_t time;
	rtc_get_datetime(&time);
	printf("SD Card %d Active at ", new_card);
	printf("%02d/%02d/%02d %02d:%02d:%02d\r\n",
		time.year, time.month, time.day, time.hour, time.minute, time.second);

	return(new_card);		
}

