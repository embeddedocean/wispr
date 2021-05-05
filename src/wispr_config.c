/*
 * wispr_config.c
 *
 * Created: 5/26/2020
 *  Author: Chris
 */ 

#include <string.h>
#include <stdio.h>
#include "wispr.h"
#include "wispr_config.h"
#include "console.h"
#include "rtc_time.h"
#include "ltc2512.h"
#include "spectrum.h"
#include "pcf2129.h"

void wispr_config_set_default(wispr_config_t *config)
{
	// set config mod time
	rtc_get_epoch(&config->epoch);
	
	config->version[1] = WISPR_VERSION;
	config->version[0] = WISPR_SUBVERSION;
	
	config->adc.buffer_size = ADC_BLOCKS_PER_BUFFER * WISPR_SD_CARD_BLOCK_SIZE;
	config->adc.sample_size = ADC_DEFAULT_SAMPLE_SIZE;
	config->adc.sampling_rate = ADC_DEFAULT_SAMPLING_RATE;

	config->acquisition_time = ADC_DEFAULT_AWAKE;
	config->sleep_time = ADC_DEFAULT_SLEEP;

	config->adc.samples_per_buffer = (uint32_t)(config->adc.buffer_size - WISPR_DATA_HEADER_SIZE) / (uint32_t)config->adc.sample_size;

	config->adc.gain = ADC_DEFAULT_GAIN; // adc gain
	config->adc.decimation = LTC2512_DF8; // adc df

	config->state = 0; //
	config->mode = WISPR_DAQ; //

	config->psd.size = PSD_DEFAULT_FFT_SIZE;
	config->psd.overlap = PSD_DEFAULT_OVERLAP;
	config->psd.window_type = RECT_WINDOW;
	
	config->file_size = WISPR_MAX_FILE_SIZE;  
	
	config->active_sd_card = 1;
}


void wispr_config_menu(wispr_config_t *config, int timeout)
{
	uint32_t u32;
	uint16_t u16;
	uint8_t u8;
	
	wispr_adc_t *adc = &config->adc;
	
	config->version[1] = WISPR_VERSION;
	config->version[0] = WISPR_SUBVERSION;
	
	adc->buffer_size = ADC_BLOCKS_PER_BUFFER * WISPR_SD_CARD_BLOCK_SIZE;
	adc->samples_per_buffer = (adc->buffer_size - WISPR_DATA_HEADER_SIZE) / 3;
	
	uint16_t blocks_per_buffer = ADC_BLOCKS_PER_BUFFER;
	//blocks_per_buffer = console_prompt_uint32("Enter number of blocks (512 bytes) per buffer", blocks_per_buffer, timeout);
	
	// commented out for fixed sample size
	//u8 = console_prompt_uint8("Enter sample size in bytes", config->sample_size, timeout);
	//if( u8 >= 2 && u8 <= 3 ) config->sample_size = u8;
	adc->sample_size = ADC_SAMPLE_SIZE;
	printf("\r\nFixed sample size: %d bytes\r\n", adc->sample_size);

	u32 = console_prompt_uint32("Enter sampling rate in Hz", adc->sampling_rate, timeout);
	if( u32 > 0 && u32 <= 350000 ) adc->sampling_rate = u32;

	u8 = adc->gain;
	u8 = console_prompt_uint8("Enter preamp gain setting (0 to 3)", u8, timeout);
	if( u8 >= 0 && u8 <= 3 ) adc->gain = u8;

	u8 = adc->decimation;
	u8 = console_prompt_uint8("Enter adc decimation factor (4, 8, 16, or 32)", u8, timeout);
	if( u8 == 4 || u8 == 8 || u8 == 16 || u8 == 32) adc->decimation = u8;

	// prompt for sampling interval
	//u16 = config->acquisition_time;
	//u16 = console_prompt_uint16("Enter sampling time window in seconds", config->acquisition_time, timeout);
	//if( u16 >= 1 ) config->acquisition_time = u16;
	//u16 = console_prompt_uint16("Enter sleep time between sampling windows in seconds", config->sleep_time, timeout);
	//if( u16 >= 0 ) config->sleep_time = u16;
	
	// update variables based on new input
	adc->buffer_size = (uint16_t)(blocks_per_buffer * WISPR_SD_CARD_BLOCK_SIZE);
	adc->samples_per_buffer = (adc->buffer_size - WISPR_DATA_HEADER_SIZE) / (uint16_t)adc->sample_size;
	float adc_buffer_duration =  (float)adc->samples_per_buffer / (float)adc->sampling_rate; // seconds
	
	//config->buffers_per_window = (uint16_t)( (float)config->acquisition_time / adc_buffer_duration ); // truncated number of buffers
	
	// prompt file file
	u32 = config->file_size;
	u32 = console_prompt_int("Enter size of data file in blocks", u32, timeout);
	config->file_size = u32;

	config->state = WISPR_IDLE;
	uint8_t mode = 0;

/*	
	// prompt to record waveform data
	int record_waveform = 0;
	if(config->mode & WISPR_DAQ) record_waveform = 1;
	if( console_prompt_int("Record_waveform?", record_waveform, timeout) ) {
		mode |= WISPR_DAQ;
	}

	// prompt for spectrum parameters
	int record_spectrum = 0;
	if(config->mode & WISPR_PSD) record_spectrum = 1;
	if( console_prompt_int("Record spectrum?", record_spectrum, timeout) ) {
		
		//u16 = config->psd.size;
		//u16 = console_prompt_uint16("Enter fft size (32, 64, 126, 512 or 1024)", u16, timeout);
		//config->psd.size = u16;
		config->psd.size = PSD_DEFAULT_FFT_SIZE;
		printf("\r\nFixed fft size: %d\r\n", config->psd.size);
		
		u16 = config->psd.overlap;
		u16 = console_prompt_uint16("Enter fft overlap size", u16, timeout);
		config->psd.overlap = u16;

		u8 = config->psd.window_type;
		u8 = console_prompt_uint8("Enter fft window type (0=Rect, 1=Hamming)", u8, timeout);
		config->psd.window_type = u8;
		
		mode |= WISPR_PSD;
	}
	
	//psd_nfft = config->psd.size;
	//psd_nbins = config->psd.size / 2;
	//psd_overlap = config->psd.overlap;

	// set the new mode
	config->mode = mode;
*/
	
	// prompt to  reset the time
	if( console_prompt_int("Enter new time?", 0, timeout) ) {
		int go = 1;
		rtc_time_t dt;
		pcf2129_get_datetime(&dt);  // get current time
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
			status = pcf2129_set_datetime(&dt);
			if( status != RTC_STATUS_OK) {
				printf("Failed to initialize DS3231 with new time\r\n");
				rtc_print_error(status);
				continue;
			}
			break;
		}
		pcf2129_get_datetime(&dt);  // read back time
		printf("\r\nExternal RTC set to %02d/%02d/%02d %02d:%02d:%02d\r\n",
		dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
	}
	
	// set config mod time
	rtc_get_epoch(&config->epoch);
	
}

void wispr_config_print(wispr_config_t *config)
{
	//float buffer_duration =  (float)config->samples_per_buffer / (float)config->sampling_rate;

	fprintf(stdout, "\r\n");
	fprintf(stdout, "WISPR %d.%d configuration\r\n", config->version[1], config->version[0]);
	//fprintf(stdout, "- epoch         %s\r\n", epoch_time_string(config->epoch));
	//fprintf(stdout, "- mode           \r\n");
	fprintf(stdout, "- mode              %d ", config->mode);
	switch(config->mode) {
		case WISPR_DAQ:
		fprintf(stdout, "[DAQ]\r\n");
		break;
		case WISPR_PSD:
		fprintf(stdout, "[PSD]\r\n");
		break;
		case (WISPR_DAQ|WISPR_PSD):
		fprintf(stdout, "[DAQ+PSD]\r\n");
		break;
	}
	fprintf(stdout, "- sample size:      %d bytes\r\n", (int)config->adc.sample_size);
	//fprintf(stdout, "- buffer_size:     %d bytes\r\n", (int)config->adc.buffer_size);
	//fprintf(stdout, "- samples:        %d per buffer\r\n", (int)config->adc.samples_per_buffer);
	fprintf(stdout, "- buffer size:      %d samples (%d bytes)\r\n", (int)config->adc.samples_per_buffer, (int)config->adc.buffer_size);
	fprintf(stdout, "- sampling rate:    %d Hz\r\n", (int)config->adc.sampling_rate);
	//fprintf(stdout, "- duration:       %lu msec\n\r", (uint32_t)(1000.0*buffer_duration));
	fprintf(stdout, "- gain:             %d\r\n", (int)config->adc.gain);
	fprintf(stdout, "- decimation:       %d\r\n", (int)config->adc.decimation);
//	fprintf(stdout, "- acquisition time: %d sec\r\n", (int)config->acquisition_time);
//	fprintf(stdout, "- sleep time:       %d sec\r\n", (int)config->sleep_time);
	fprintf(stdout, "- active card:      %d\r\n", config->active_sd_card);
	fprintf(stdout, "- file size:        %d blocks\r\n", (int)config->file_size);
    if(config->mode & WISPR_PSD) {
		fprintf(stdout, "- fft size:         %d\r\n", (int)config->psd.size);
		fprintf(stdout, "- fft overlap:      %d\r\n", (int)config->psd.overlap);
		fprintf(stdout, "- fft window type:  %d\r\n", (int)config->psd.window_type);
		fprintf(stdout, "- psd nbins:        %d\r\n", (int)config->psd.nbins);
		fprintf(stdout, "- psd navg:         %d\r\n", (int)config->psd.navg);
	}
	fprintf(stdout, "\r\n");
}

