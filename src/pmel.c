/*
 * pmel.c: 
 * PMEL specific command interface and 
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
#include "ina260.h"
#include "pmel.h"

static char _buffer[COM_MAX_MESSAGE_SIZE];

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
	char *buf = _buffer;	
	int type = PMEL_UNKNOWN;
	
	// check for a com message 
	int status = com_read_msg (BOARD_COM_PORT, buf, timeout);
	if( status == COM_VALID_MSG ) {
		type = pmel_msg_type(buf);
		printf("pmel control message received: %s\r\n", buf);
	} else {
		status = com_write_msg(BOARD_COM_PORT, "NACK");
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
			config->adc.gain = new_gain;
			printf("GAIN: %d\r\n", config->adc.gain);
		}
	}

	if ( type == PMEL_SDF ) {	// report SD card memory usage
		//
	}

	if ( type == PMEL_PSD ) {	// report SD card memory usage
		uint16_t fft_size; // fft size used for spectrum
		//uint16_t fft_overlap; // fft overlap used for spectrum
		uint16_t duration;  // number of time steps (adc buffer reads) to average
		sscanf (&buf[4], "%u,%u", &fft_size, &duration);
		float adc_buffer_duration = (float)config->adc.samples_per_buffer / (float)config->adc.sampling_rate; // seconds
		config->psd.size = fft_size;
		config->psd.overlap = 0;
		config->mode |= WISPR_SPECTRUM;
		config->psd.nbins = fft_size / 2;
		config->psd.count = 0; // reset the processing counter
		config->psd.navg = (uint16_t)(duration / adc_buffer_duration); // determine number of buffers to average for psd estimate
		printf("PSD: size=%d, navg=%d\r\n", config->psd.size, config->psd.navg);
	}

	// If a valid message was received then send ACK, else send NACK
	if( type != PMEL_UNKNOWN ) {
		status = com_write_msg(BOARD_COM_PORT, "ACK");
	} else {
		status = com_write_msg(BOARD_COM_PORT, "NACK");
	}
	
	return (type);
}

//
// Request and wait for a GPS message
// This will wait until it receives a GPS message, ignoring all other types of messages
// Timeout is in seconds
int pmel_request_gps(wispr_config_t *config, uint16_t timeout_sec)
{
	//char *buf = com_buffer;
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

//
// Wait for ACK
//
int pmel_wait_for_ack (wispr_config_t *config, uint16_t timeout_sec)
{
	int type = PMEL_UNKNOWN;
	
	uint16_t count = 0;
	while( count < timeout_sec ) {
		type = pmel_control(config, 1000); // timeout in 1000 msecs
		if( type == PMEL_ACK ) {
			break;
		}
		count++;
	}

	// check for a com message
	if( type == PMEL_ACK ) {
		return(PMEL_ACK);
	} else {
		return(PMEL_NACK);
	}
}
	
int pmel_transmit_spectrum(wispr_config_t *config, float32_t *psd_average, uint16_t nbins, uint8_t *buffer, pmel_control_t *pmel)
{
	int status = 0;
	char *msg = _buffer;

	uint16_t nwrt = 0;
	nwrt += sprintf(&buffer[nwrt], "%s", pmel_time_string(config->psd.second));
	nwrt += sprintf(&buffer[nwrt], ",%s", pmel->instrument_id);
	nwrt += sprintf(&buffer[nwrt], ",%s", pmel->location_id);
	nwrt += sprintf(&buffer[nwrt], ",%.2f", pmel->volts ); //
	nwrt += sprintf(&buffer[nwrt], ",%.2f", pmel->free);
	nwrt += sprintf(&buffer[nwrt], ",%d.%d", pmel->version[0], pmel->version[1]);
	nwrt += sprintf(&buffer[nwrt], ",%d", config->adc.sampling_rate);
	nwrt += sprintf(&buffer[nwrt], ",%d", config->psd.nbins);

	// scale = 2 * log10( (ADC_VREF / 2147483647.0 / (float32_t)config->psd.size) );
	float32_t scale = 2.0 * (log10f(ADC_VREF) - log10f(2147483647.0) - log10f((float32_t)config->psd.size));

	// overwrite the psd buffer with scaled db values
	for(int n = 0; n < nbins; n++) {
		psd_average[n] = 10.0 * (log10f(psd_average[n]) + scale);
		//nwrt += sprintf(&buffer[nwrt], ",%d.%d", (int)psd_average[n], ((int)(100.0*psd_average[n]) - 100*(int)psd_average[n]) );
		nwrt += sprintf(&buffer[nwrt], ",%.1f", psd_average[n] );
	}

	// send spectrum to controller, repeat until an ACK is received or quit after 10 tries 
	int count = 10;
	while(count--) {

		uart_write_queue(BOARD_COM_PORT, "@@@\r\n", 5);
	
		// write CRC of the message
		uint8_t crc = com_CRC(buffer, nwrt);
		sprintf(msg, "%02x\r\n", crc);
		status = uart_write_queue(BOARD_COM_PORT, msg, strlen(msg));

		// write the number of bytes in the message
		sprintf(msg, "%04x\r\n", nwrt);
		status = uart_write_queue(BOARD_COM_PORT, msg, strlen(msg));	

	 	// stream binary data out the uart
		for(int n = 0; n < nwrt; n++) {
			while (!uart_is_tx_empty(BOARD_COM_UART)) {}
			uart_write(BOARD_COM_UART, buffer[n]);
		}
	
		// wait 10 seconds for ACK
		status = pmel_wait_for_ack(config, 10);
		if( status == PMEL_ACK ) break;
	
	}
	
	return(status);
}

int pmel_send_sdb(wispr_config_t *config, float32_t *psd_average, uint16_t nbins)
{
	int status = 0;
	char *buf = _buffer;

	sprintf(buf, "%d\r\n", nbins);
	status = uart_write_queue(BOARD_COM_PORT, buf, strlen(buf));

	// scaling = ( adc_vref / (2^(4*8-1)-1) / fft_size )^2;  % see spectrum.c
	// scale = 2 * log10( (ADC_VREF / 2147483647.0 / (float32_t)config->fft_size) );
	float32_t scale = 2.0 * (log10f(ADC_VREF) - log10f(2147483647.0) - log10f((float32_t)config->psd.size));
	//printf("psd scale = %f\r\n", scale);

	// bin frequency in kHz
	float32_t freq = 0.0;
	float32_t dfreq = 0.001 * (float32_t)config->adc.sampling_rate / (float32_t)config->psd.size;
	
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
	
	if (strncmp (buf, "ACK", 3) == 0) {  // Exit command
		type = PMEL_ACK;
	}
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

char *pmel_time_string(uint32_t epoch)
{
	rtc_time_t tme;
	epoch_to_rtc_time(&tme, epoch);
	sprintf(_buffer, "%02d:%02d:%02d:%02d:%02d:%02d",
		tme.month,tme.day,tme.year,tme.hour,tme.minute,tme.second);
	return(_buffer);
}

void pmel_filename(char *name, char *prefix, char *suffix, rtc_time_t *dt)
{
	//rtc_get_datetime(&dt);
	sprintf(name, "%s_%02d%02d%02d_%02d%02d%02d.%s",
		prefix, dt->year, dt->month, dt->day, dt->hour, dt->minute, dt->second, suffix);
}

int pmel_file_header(char *buf, wispr_config_t *config, wispr_data_header_t *hdr, pmel_control_t *pmel)
{
	int nwrt = 0;
	//float buffer_duration =  (float)hdr->samples_per_buffer / (float)hdr->sampling_rate;
	nwrt += sprintf(&buf[nwrt], "%% WISPR %d.%d\r\n", config->version[1], config->version[0]);
	nwrt += sprintf(&buf[nwrt], "time = '%s';\r\n", pmel_time_string(hdr->second));
	//nwrt += sprintf(&buf[nwrt], "sec = %d;\r\n", hdr->second); // f_printf doesn't support %f format
	//nwrt += sprintf(&buf[nwrt], "usec = %d;\r\n", hdr->usec); // f_printf doesn't support %f format
	nwrt += sprintf(&buf[nwrt], "intrument_id = '%s'\r\n", pmel->instrument_id);
	nwrt += sprintf(&buf[nwrt], "location_id = '%s'\r\n", pmel->location_id);
	nwrt += sprintf(&buf[nwrt], "volts = %.2f;\r\n", pmel->volts ); //
	nwrt += sprintf(&buf[nwrt], "blocks_free = %d\r\n", pmel->free);
	nwrt += sprintf(&buf[nwrt], "version = %d.%d\r\n", pmel->version[1], pmel->version[0]);
	
	//nwrt += sprintf(&buf[nwrt], "mode = %d;\r\n", config->mode);
	nwrt += sprintf(&buf[nwrt], "number_buffers = %d;\r\n", config->file_size);
	nwrt += sprintf(&buf[nwrt], "buffer_size = %d;\r\n", (int)config->adc.buffer_size);
	nwrt += sprintf(&buf[nwrt], "samples_per_buffer = %d;\r\n", config->adc.samples_per_buffer);
	nwrt += sprintf(&buf[nwrt], "sample_size = %d;\r\n", (int)config->adc.sample_size);
	nwrt += sprintf(&buf[nwrt], "sampling_rate = %d;\r\n", config->adc.sampling_rate);
	nwrt += sprintf(&buf[nwrt], "gain = %d;\r\n", config->adc.gain);
	//nwrt += sprintf(&buf[nwrt], "adc_decimation = %d;\r\n", config->adc.decimation);
	//nwrt += sprintf(&buf[nwrt], "acquisition_time = %d;\r\n", config->acquisition_time);
	//nwrt += sprintf(&buf[nwrt], "sleep_time = %d;\r\n", config->sleep_time);
	//nwrt += sprintf(&buf[nwrt], "fft_size = %d;\r\n", config->psd.size);
	//nwrt += sprintf(&buf[nwrt], "fft_overlap = %d;\r\n", config->psd.overlap);
	//nwrt += sprintf(&buf[nwrt], "fft_window_type = %d;\r\n", config->psd.window_type);
	nwrt += sprintf(&buf[nwrt], "adc_vref = %d.%02d;\r\n", (int)ADC_VREF, (int)(ADC_VREF*100) - 100*(int)ADC_VREF ); // no %f
	
	//printf(", %d bytes written\r\n", nwrt);
	
	return(nwrt);
}

//
// Update the WISPR config structure with the current PMEL configuration
//
int pmel_update_config (wispr_config_t *pmel, wispr_config_t *wispr)
{	 
	wispr->state = pmel->state;
	wispr->mode = pmel->mode;
	wispr->epoch = pmel->epoch;
	wispr->adc.sampling_rate = pmel->adc.sampling_rate;
	wispr->adc.decimation = pmel->adc.decimation;
	wispr->psd.size = pmel->psd.size;
	wispr->acquisition_time = pmel->acquisition_time;
	// ...
	return(0);
}


