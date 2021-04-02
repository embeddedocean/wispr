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

int pmel_init(wispr_config_t *config)
{
	int status = 0;
	
	// initialize the uart com communications port
	status = com_init(BOARD_COM_PORT, BOARD_COM_BAUDRATE);
	
	// initial control state
	config->state = WISPR_IDLE;
	config->mode = 0;

	return(status);
}

//
// PMEL message handling and parsing
// Make sure that config does not get corrupted by invalid com parameters
//
int pmel_control (wispr_config_t *config, uint16_t timeout)
{
	char *buf = com_buffer;	
	int type = PMEL_UNKNOWN;
	
	// check for a com message 
	int status = com_read_msg (BOARD_COM_PORT, buf, timeout);
	if( status == COM_VALID_MSG ) {
		type = pmel_msg_type(buf);
		printf("pmel control message received: %s\r\n", com_buffer);
	} else {
		return(type);
	}
	
	// Exit command puts system into backup sleep mode
	if ( type == PMEL_EXIT ) {  
		config->state = WISPR_SLEEP_BACKUP;
	}

	// Run command
	if ( type == PMEL_RUN ) {  // Run command
		config->state = WISPR_ACTIVE;
	}
	
	// Pause command
	if ( type == PMEL_PAUSE ) {  
		config->state = WISPR_PAUSED;
	}
	
	// Sleep command
	if ( type == PMEL_SLEEP ) {
		config->state = WISPR_SLEEP_WFI;
	}
	
	// Stat command
	if ( type == PMEL_STATUS ) {
		
	}

	// GPS message
	if (type == PMEL_GPS) {	
		sscanf (&buf[4], "%lu,%f,%f", &config->gps.second, &config->gps.lat, &config->gps.lon);
		printf("GPS: sec=%lu, lat=%f, lon=%f \r\n", config->gps.second, config->gps.lat, config->gps.lon);
	}
	
	if ( type == PMEL_TIME ) {	// set time
		sscanf (&buf[4], "%lu", &config->gps.second);
		printf("TIME: sec=%lu\r\n", config->gps.second);
	}
	
	if ( type == PMEL_GAIN ) { 	// set gain
		uint8_t new_gain = 0;
		sscanf (&buf[4], "%d", &new_gain);
		if( (new_gain < 4 ) && ( new_gain > 0 ) ) {
			config->gain = new_gain;
			printf("GAIN: %d\r\n", config->gain);
		}
	}

	if ( type == PMEL_SDF ) {	// report SD card memory usage
		//
	}

	if ( type == PMEL_PSD ) {	// report SD card memory usage
		uint16_t fft_size; // fft size used for spectrum
		uint16_t fft_overlap; // fft overlap used for spectrum
		uint16_t duration;  // number of time steps (adc buffer reads) to average
		sscanf (&buf[4], "%u,%u", &fft_size, &duration);
		float adc_buffer_duration = (float)config->samples_per_buffer / (float)config->sampling_rate; // seconds
		config->fft_size = fft_size;
		config->fft_overlap = 0;
		config->mode |= WISPR_SPECTRUM;
		config->psd_nbins = fft_size / 2;
		config->psd_count = 0; // reset the processing counter
		config->psd_navg = (uint16_t)(duration / adc_buffer_duration); // determine number of buffers to average for psd estimate
		printf("PSD: size=%d, navg=%d\r\n", config->fft_size, config->psd_navg);
	}

	// If a valid message was received then send Acknowledge
	if( type != PMEL_UNKNOWN ) {
		status = com_write_msg(BOARD_COM_PORT, "ACK");
	}
	
	return (type);
}

//
// Request and wait for a GPS message
// This will wait until it receives a GPS message, ignoring all other types of messages
// Timeout is in seconds
int pmel_request_gps(wispr_config_t *config, uint16_t timeout_sec)
{
	char *buf = com_buffer;
	int type = PMEL_NONE;

	//Set RTC time by GPS epoch sec received at COM0. Gain is also changed. HM
	printf("Requesting GPS message to set RTC.\r\n");
	
	//Sends $GPS* que to MPC as a request for GPS time and Location
	com_write_msg(BOARD_COM_PORT, "GPS");
		
	uint16_t count = 0;
	while( count < timeout_sec ) {
		type = pmel_control(config, 1000); // timeout in 1000 msecs
		if( type == PMEL_GPS ) {
			break;
		}
		count++;
	}
	return(type);
}

int pmel_request_gain(wispr_config_t *config, uint16_t timeout_sec)
{
	uint8_t new_gain;
	char *buf = com_buffer;
	int type = PMEL_NONE;
	
	// request a gain change 
	com_write_msg(BOARD_COM_PORT, "NGN");

	uint16_t count = 0;
	while( count < timeout_sec ) {
		type = pmel_control(config, 1000); // timeout in 1000 msecs
		if( type == PMEL_GAIN ) {
			break;
		}
		count++;
	}
	return(type);
}

int pmel_transmit_spectrum(wispr_config_t *config, float32_t *psd_average, uint16_t nbins)
{
	int status = 0;
	char buf[8];

	uart_write_queue(BOARD_COM_PORT, "@@@\r\n", 5);

	// scale = 2 * log10( (ADC_VREF / 2147483647.0 / (float32_t)config->fft_size) );
	float32_t scale = 2.0 * (log10f(ADC_VREF) - log10f(2147483647.0) - log10f((float32_t)config->fft_size));

	// overwrite the psd buffer with scaled db values
	for(int n = 0; n < nbins; n++) {
		psd_average[n] = 10.0 * (log10f(psd_average[n]) + scale);
	}

	uint8_t *data = (uint8_t *)psd_average;
	uint8_t crc = com_CRC(data, 4*nbins);
	sprintf(buf, "%02x\r\n", crc);
	status = uart_write_queue(BOARD_COM_PORT, buf, strlen(buf));

	sprintf(buf, "%d\r\n", nbins);
	status = uart_write_queue(BOARD_COM_PORT, buf, strlen(buf));

	// stream binary data out the uart
	for(int n = 0; n < nbins; n++) {
		float32_t db = 10.0 * (log10f(psd_average[n]) + scale);
		while (!uart_is_tx_empty(BOARD_COM_UART)) {}
		uart_write(BOARD_COM_UART, data[n]);
	}
	
	return(status);
}

int pmel_send_sdb(wispr_config_t *config, float32_t *psd_average, uint16_t nbins)
{
	int status = 0;
	char buf[32];

	sprintf(buf, "%d\r\n", nbins);
	status = uart_write_queue(BOARD_COM_PORT, buf, strlen(buf));

	// scaling = ( adc_vref / (2^(4*8-1)-1) / fft_size )^2;  % see spectrum.c
	// scale = 2 * log10( (ADC_VREF / 2147483647.0 / (float32_t)config->fft_size) );
	float32_t scale = 2.0 * (log10f(ADC_VREF) - log10f(2147483647.0) - log10f((float32_t)config->fft_size));
	//printf("psd scale = %f\r\n", scale);

	// bin frequency in kHz
	float32_t freq = 0.0;
	float32_t dfreq = 0.001 * (float32_t)config->sampling_rate / (float32_t)config->fft_size;
	
	for(int n = 0; n < nbins; n++) {
		float32_t db = 10.0 * (log10f(psd_average[n]) + scale);
		sprintf(buf, "SDB,%.3f,%.2f", freq, db);
		//status = uart_write_queue(BOARD_COM_PORT, buf, strlen(buf));
		com_write_msg(BOARD_COM_PORT, buf);
		freq += dfreq;
	}
	
	return(status);
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
	if (strncmp (buf, "PSD", 3) == 0) {	// Spectrum
		type = PMEL_PSD;
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

int pmel_file_header(char *buf, wispr_config_t *config, wispr_data_header_t *hdr)
{
	int nwrt = 0;
	//float buffer_duration =  (float)hdr->samples_per_buffer / (float)hdr->sampling_rate;
	nwrt += sprintf(&buf[nwrt], "%% WISPR %d.%d\r\n", config->version[1], config->version[0]);
	nwrt += sprintf(&buf[nwrt], "time = '%s';\r\n", epoch_time_string(hdr->second));
	nwrt += sprintf(&buf[nwrt], "second = %d;\r\n", hdr->second); // f_printf doesn't support %f format
	nwrt += sprintf(&buf[nwrt], "second = %d;\r\n", hdr->usec); // f_printf doesn't support %f format
	nwrt += sprintf(&buf[nwrt], "mode = %d;\r\n", config->mode);
	nwrt += sprintf(&buf[nwrt], "number_buffers = %d;\r\n", config->file_size);
	nwrt += sprintf(&buf[nwrt], "samples_per_buffer = %d;\r\n", config->samples_per_buffer);
	nwrt += sprintf(&buf[nwrt], "sample_size = %d;\r\n", (int)config->sample_size);
	nwrt += sprintf(&buf[nwrt], "buffer_size = %d;\r\n", (int)config->buffer_size);
	nwrt += sprintf(&buf[nwrt], "sampling_rate = %d;\r\n", config->sampling_rate);
	nwrt += sprintf(&buf[nwrt], "gain = %d;\r\n", config->gain);
	nwrt += sprintf(&buf[nwrt], "acquisition_time = %d;\r\n", config->acquisition_time);
	nwrt += sprintf(&buf[nwrt], "sleep_time = %d;\r\n", config->sleep_time);
	nwrt += sprintf(&buf[nwrt], "fft_size = %d;\r\n", config->fft_size);
	nwrt += sprintf(&buf[nwrt], "fft_overlap = %d;\r\n", config->fft_overlap);
	nwrt += sprintf(&buf[nwrt], "fft_window_type = %d;\r\n", config->fft_window_type);
	nwrt += sprintf(&buf[nwrt], "adc_decimation = %d;\r\n", config->adc_decimation);
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


