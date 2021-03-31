/*
 * pmel.c: 
 * Embedded Ocean Systems (EOS), 2021
 *
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

int pmel_init(pmel_control_t *pmel)
{
	int status = 0;
	
	// initialize the uart com communications port
	status = com_init(BOARD_COM_PORT, BOARD_COM_BAUDRATE);
	
	// initial control state
	pmel->state = WISPR_IDLE;
	pmel->mode = 0;

	return(status);
}

//
// PMEL message handling and parsing
// Make sure that config does not get corrupted by invalid com parameters
//
int pmel_control (pmel_control_t *pmel, uint16_t timeout)
{
	char *buf = com_buffer;	
	int type = PMEL_NONE;
	
	// check for a com message, no wait timeout
	if( com_read_msg (BOARD_COM_PORT, buf, timeout) == COM_VALID_MSG ) {
		type = pmel_msg_type(buf);
		printf("pmel control message received: %s\r\n", com_buffer);
	}
	
	// Exit command puts system into backup sleep mode
	if ( type == PMEL_EXIT ) {  
		pmel->state = WISPR_SLEEP_BACKUP;
	}

	// Run command
	if ( type == PMEL_RUN ) {  // Run command
		pmel->state = WISPR_ACTIVE;
	}
	
	// Pause command
	if ( type == PMEL_PAUSE ) {  
		pmel->state = WISPR_PAUSED;
	}
	
	// Sleep command
	if ( type == PMEL_SLEEP ) {
		pmel->state = WISPR_SLEEP_WFI;
	}
	
	// Stat command
	if ( type == PMEL_STATUS ) {
		
	}

	// GPS message
	if (type == PMEL_GPS) {	
		sscanf (&buf[4], "%lu,%f,%f", &pmel->gps.second, &pmel->gps.lat, &pmel->gps.lon);
		printf("GPS: sec=%lu, lat=%f, lon=%f \r\n", pmel->gps.second, pmel->gps.lat, pmel->gps.lon);
	}
	
	if ( type == PMEL_TIME ) {	// set time
		sscanf (&buf[4], "%lu", &pmel->second);
		printf("TIME: sec=%lu\r\n", pmel->second);
	}
	
	if ( type == PMEL_GAIN ) { 	// set gain
		uint8_t new_gain = 0;
		sscanf (&buf[4], "%d", &new_gain);
		if( (new_gain < 4 ) && ( new_gain > 0 ) ) {
			pmel->gain = new_gain;
			printf("GAIN: %d\r\n", pmel->gain);
		}
	}

	if ( type == PMEL_SDF ) {	// report SD card memory usage
	}

	return (type);
}

//
// Request and wait for a GPS message
// This will wait until it receives a GPS message, ignoring all other types of messages
// Timeout is in seconds
int pmel_request_gps(pmel_control_t *pmel, uint16_t timeout_sec)
{
	char *buf = com_buffer;
	int type = PMEL_NONE;

	//Set RTC time by GPS epoch sec received at COM0. Gain is also changed. HM
	printf("Requesting GPS message to set RTC.\r\n");
	
	//Sends $GPS* que to MPC as a request for GPS time and Location
	com_write_msg(BOARD_COM_PORT, "GPS");
		
	uint16_t count = 0;
	while( count < timeout_sec ) {
		type = pmel_control(pmel, 1000); // timeout in 1000 msecs
		if( type == PMEL_GPS ) {
			break;
		}
		count++;
	}
	return(type);
}

int pmel_request_gain(pmel_control_t *pmel, uint16_t timeout_sec)
{
	uint8_t new_gain;
	char *buf = com_buffer;
	int type = PMEL_NONE;
	
	// request a gain change 
	com_write_msg(BOARD_COM_PORT, "NGN");

	uint16_t count = 0;
	while( count < timeout_sec ) {
		type = pmel_control(pmel, 1000); // timeout in 1000 msecs
		if( type == PMEL_GAIN ) {
			break;
		}
		count++;
	}
	return(type);
}

//
// Known message types 
//
int pmel_msg_type (char *buf)
{
	int type = PMEL_NONE;

	if( strlen(buf) < 3 ) return(type);	// too short
	
	if (strncmp (buf, "EXI", 3) == 0) {  // Exit command
		type = PMEL_EXIT;
	}
	if (strncmp (buf, "RUN", 3) == 0) {  // Run command
		type = PMEL_RUN;
	}
	if (strncmp (buf, "PAU", 3) == 0) {  // Pause command
		type = PMEL_PAUSE;
	}
	if (strncmp (buf, "RST", 3) == 0) {  // Reset command
		type = PMEL_RESET;
	}
	if (strncmp (buf, "SLP", 3) == 0) {  // Sleep command
		type = PMEL_SLEEP;
	}
	if (strncmp (buf, "STA", 3) == 0) { // Status command
		type = PMEL_STATUS;
	}
	if (strncmp (buf, "SET", 3) == 0) { // Set command
		type = PMEL_SET;
	}
	if (strncmp (buf, "GPS", 3) == 0) {	// GPS message
		type = PMEL_GPS;
	}
	if (strncmp (buf, "TME", 3) == 0) {	// set time
		type = PMEL_TIME;
	}
	if (strncmp (buf, "NGN", 3) == 0) { 	// set gain
		type = PMEL_GAIN;
	}
	if (strncmp (buf, "SDF", 3) == 0) {	// report SD card memory usage
		type = PMEL_SDF;
	}

	return (type);
}


//----------------------------------------------------------------

void pmel_filename(char *name, char *prefix, char *suffix, rtc_time_t *dt)
{
	//rtc_get_datetime(&dt);
	sprintf(name, "%s_%02d%02d%02d_%02d%02d%02d.%s",
		prefix, dt->year, dt->month, dt->day, dt->hour, dt->minute, dt->second, suffix);
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

//
// Update the WISPR config structure with the current PMEL configuration
//
int pmel_update_config (wispr_config_t *pmel, wispr_config_t *wispr)
{
	int status = 0; // 0 = OK, 1 = error
	 
	wispr->state = pmel->state;
	wispr->mode = pmel->mode;
	wispr->epoch = pmel->epoch;
	wispr->sampling_rate = pmel->sampling_rate;
	wispr->adc_decimation = pmel->adc_decimation;
	wispr->fft_size = pmel->fft_size;
	wispr->acquisition_time = pmel->acquisition_time;
	// ...
	return(0);
}


