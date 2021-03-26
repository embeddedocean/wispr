/*
 * pmel.c: 
 * Embedded Ocean Systems (EOS), 2019
 *
 *----------------------------------------------------------------
*/

#include <stdio.h>
//#include <stdlib.h>
#include <string.h>

#include "wispr.h"
#include "com.h"
#include "uart_queue.h"
#include "wdt.h"
#include <delay.h>
#include "pmel.h"

static char com_buffer[COM_MAX_MESSAGE_SIZE];
static uint8_t com_cksum = 0;


int pmel_request_gps(wispr_com_msg_t *msg, uint16_t timeout)
{
	char *buf = com_buffer;
	
	//Set RTC time by GPS epoch sec received at COM0. Gain is also changed. HM
	printf("Requesting GPS message to set RTC.\r\n");
	msg->lat = 0.0;
	msg->lon = 0.0;
	
	//Sends $GPS* que to MPC as a request for GPS time and Location
	com_write_msg(BOARD_COM_PORT, "GPS");
	
	int nrd = com_read_msg (BOARD_COM_PORT, buf, timeout);
	
	if(nrd > 0) {
		com_parse_msg(msg, buf, nrd);
		printf("lat=%f, lon=%f\n\r", msg->lat, msg->lon); //debug - Remove this
	}
	
	return(nrd);
}

int pmel_request_gain(wispr_com_msg_t *msg, uint16_t timeout)
{
	uint8_t new_gain;
	char *buf = com_buffer;
	
	//HM added to check if a gain change is requested
	com_write_msg(BOARD_COM_PORT, "NGN");

	printf("Type at com0 $NGN,1*\r\n");//HM For debug. Remove this

	int nrd = com_read_msg (BOARD_COM_PORT, buf, timeout);

	new_gain = ADC_DEFAULT_GAIN;
	
	if(nrd > 0) {
		com_parse_msg(msg, buf, nrd);
		//printf("%d\n\r",com_msg.gain);//  HM debug
		//printf("new gain = %d\n\r", com_msg.gain);//HM debug
		if((msg->gain < 4) && (msg->gain >= 0)) {
			new_gain = msg->gain;//HM Gain update is ready. config.settings[0] will be updated in initialize_config()
		}
	}

	// set the global config with the new gain
	msg->gain = new_gain;
	
	return(nrd);

}


int pmel_file_header(char *buf, wispr_config_t *cfg, wispr_data_header_t *hdr)
{
	int nwrt = 0;
	//float buffer_duration =  (float)hdr->samples_per_buffer / (float)hdr->sampling_rate;
	nwrt += sprintf(&buf[nwrt], "%% WISPR %d.%d\r\n", cfg->version[1], cfg->version[0]);
	nwrt += sprintf(&buf[nwrt], "time = '%s';\r\n", epoch_time_string(hdr->second));
	nwrt += sprintf(&buf[nwrt], "second = %d.%06d;\r\n", hdr->second, hdr->usec); // f_printf doesn't support %f format
	nwrt += sprintf(&buf[nwrt], "mode = %d;\r\n", cfg->mode);
	nwrt += sprintf(&buf[nwrt], "number_buffers = %d;\r\n", cfg->file_size);
	nwrt += sprintf(&buf[nwrt], "samples_per_buffer = %d;\r\n", hdr->samples_per_buffer);
	nwrt += sprintf(&buf[nwrt], "sample_size = %d;\r\n", (int)hdr->sample_size);
	nwrt += sprintf(&buf[nwrt], "buffer_size = %d;\r\n", (int)hdr->buffer_size);
	nwrt += sprintf(&buf[nwrt], "sampling_rate = %d;\r\n", hdr->sampling_rate);
	nwrt += sprintf(&buf[nwrt], "gain = %d;\r\n", cfg->gain);
	nwrt += sprintf(&buf[nwrt], "acquisition_time = %d;\r\n", cfg->acquisition_time);
	nwrt += sprintf(&buf[nwrt], "sleep_time = %d;\r\n", cfg->sleep_time);
	nwrt += sprintf(&buf[nwrt], "fft_size = %d;\r\n", cfg->fft_size);
	nwrt += sprintf(&buf[nwrt], "fft_overlap = %d;\r\n", cfg->fft_overlap);
	nwrt += sprintf(&buf[nwrt], "fft_window_type = %d;\r\n", cfg->fft_window_type);
	nwrt += sprintf(&buf[nwrt], "adc_decimation = %d;\r\n", cfg->adc_decimation);
	nwrt += sprintf(&buf[nwrt], "adc_vref = %d.%02d;\r\n", (int)ADC_VREF, (int)(ADC_VREF*100) - 100*(int)ADC_VREF ); // no %f
	
	//printf(", %d bytes written\r\n", nwrt);
	
	return(nwrt);
}

//---------------------------------------------------------------------
// Add your own message parsing here
//
int pmel_parse_msg (wispr_config_t *cfg, char *buf, int len)
{
	char *args = com_buffer;

	if (verbose_level) {
		printf("com_parse_msg: %s\r\n", buf);
	}

	msg->type = COM_UNKNOWN;
	
	// Add user commands here

	if (strncmp (buf, "EXI", 3) == 0) {  // Exit command
		msg->type = COM_EXIT;
	}
	if (strncmp (buf, "RUN", 3) == 0) {  // Run command
		msg->type = COM_RUN;
	}
	if (strncmp (buf, "PAU", 3) == 0) {  // Pause command
		msg->type = COM_PAUSE;
	}
	if (strncmp (buf, "RST", 3) == 0) {  // Reset command
		msg->type = COM_RESET;
	}
	if (strncmp (buf, "SLP", 3) == 0) {  // Sleep command
		msg->type = COM_SLEEP;
	}
	if (strncmp (buf, "STA", 3) == 0) { // Status command
		msg->type = COM_STATUS;
	}
	if (strncmp (buf, "SET", 3) == 0) { // Set command
		msg->type = COM_SET;
	}

	// GPS message
	if (strncmp (buf, "GPS", 3) == 0) {
		msg->type = COM_GPS;
		strcpy (args, &buf[4]);  // copy args
		sscanf (args, "%lu,%f,%f", &msg->sec, &msg->lat, &msg->lon);
		if(verbose_level) {
			printf("GPS: sec=%lu, lat=%f, lon=%f \r\n", msg->sec, msg->lat, msg->lon);
		}
	}

	// set time
	if (strncmp (buf, "TME", 3) == 0) {
		msg->type = COM_TIME;
		strcpy (args, &buf[4]);  // copy args
		sscanf (args, "%lu", &msg->sec);
	}

	// set gain
	if (strncmp (buf, "NGN", 3) == 0) {
		msg->type = COM_GAIN;
		strcpy (args, &buf[4]);  // copy args
		sscanf (args, "%d", &msg->gain);
	}

	// report SD card memory usage
	if (strncmp (buf, "SDF", 3) == 0) {
		msg->type = COM_SDF;
	}

	if (verbose_level > 2) {
		printf("com_parse_msg: type=%d\r\n", msg->type);
	}

	return (msg->type);
}

