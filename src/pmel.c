﻿/*
 * pmel.c: 
 * PMEL specific command interface and 
 * Embedded Ocean Systems (EOS), 2021
 *
*/

#include <stdio.h>
//#include <stdlib.h>
#include <string.h>
#include <ioport.h>

#include "wispr.h"
#include "com.h"
#include "uart_queue.h"
#include "wdt.h"
#include <delay.h>
#include "ina260.h"
#include "sd_card.h"
#include "rtc_time.h"
#include "pcf2129.h"

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
	int status = 0;
	char *buf = _buffer;
	int type = PMEL_UNKNOWN;
	uint32_t sec;
	rtc_time_t dt;
	
	rtc_get_datetime(&dt);  // read time
	epoch_to_rtc_time(&dt, sec);

	// check for a com message 
	status = com_read_msg (BOARD_COM_PORT, buf, timeout);
	if( status == COM_VALID_MSG ) {
		type = pmel_msg_type(buf);
		printf("pmel control message received: %s\r\n", buf);
	} else {
		if(status == COM_INVALID_CRC) {
			status = com_write_msg(BOARD_COM_PORT, "NACK");	
		}
		return(status);
	}

	// Exit command puts system into backup sleep mode
	if ( type == PMEL_EXI ) {  
		config->prev_state = config->state;
		config->state = WISPR_SLEEP_BACKUP;
		// The wdt can't be stopped once it's started so 
		// go into backup sleep mode with no wakeup alarm
		config->sleep_time = 0; 
		printf("PMEL EXIT\r\n");
	}

	// Sleep command
	if ( type == PMEL_SLP ) {
		sscanf (&buf[4], "%lu", &sec);
		config->sleep_time = sec;
		if( (sec > 0) && (sec <= PMEL_MAX_SLEEP) ) {
			config->prev_state = config->state;
			config->state = WISPR_SLEEP_BACKUP;
			printf("PMEL SLEEP for %d seconds\r\n", sec);	
		} else {
			status = COM_INVALID_ARG;
			printf("PMEL SLEEP: invalid time %d\r\n", sec);
		}
	}
	
	// Run command
	if ( type == PMEL_RUN ) { 
		config->prev_state = config->state;
		config->state = WISPR_RUNNING;
		printf("PMEL RUN\r\n");
	}
	
	// Pause command
	if ( type == PMEL_PAU ) {  
		sscanf (&buf[4], "%lu", &sec);
		if( (sec > 0) && (sec <= PMEL_MAX_PAUSE) ) {
			config->pause_time = sec;
			config->prev_state = config->state;
			config->state = WISPR_PAUSED;
			printf("PMEL PAUSE for %d seconds\r\n", sec);
		} else {
			status = COM_INVALID_ARG;
			printf("PMEL PAUSE: invalid time %d\r\n", sec);
		}
	}
	
	// Reset command
	if ( type == PMEL_RST ) {
		// flush the uarts
		while (!uart_is_tx_empty(UART0)) {}
		while (!uart_is_tx_empty(UART1)) {}
		printf("PMEL RESET\r\n");
		// force reset
		rstc_start_software_reset(RSTC);
	}
	
	// Request status command
	if ( type == PMEL_STA ) {
		pmel_send_status(config);
	}

	// Request sd card usage command
	if ( type == PMEL_SDF ) {
		pmel_send_sd_usage(config);
	}

	// GPS message
	if (type == PMEL_GPS) {	
		sscanf (&buf[4], "%lu,%f,%f", &config->gps.second, &config->gps.lat, &config->gps.lon);
		printf("GPS: sec=%lu, lat=%f, lon=%f \r\n", config->gps.second, config->gps.lat, config->gps.lon);
	}
	
	if ( type == PMEL_TME ) {	// set time
		sscanf (&buf[4], "%lu", &sec);
		epoch_to_rtc_time(&dt, sec);	
		// If time is valid initialize internal and external RTC times
		if( rtc_valid_datetime(&dt) == RTC_STATUS_OK ) {
			pcf2129_set_datetime(&dt);  // set external rtc
			status = rtc_init(&dt); // set internal rtc
			while ( status != RTC_STATUS_OK ) {
				printf("Waiting for RTC, status %d\r\n", status);
				status = rtc_init(&dt); 
			}
			printf("SET TIME: %s\r\n", epoch_time_string(sec));
		}
	}
	
	if ( type == PMEL_WTM ) {	// send time
		char msg[32];
		rtc_time_t dt;
		uint32_t sec = 0;
		pcf2129_get_datetime(&dt);  // read time
		sec = rtc_time_to_epoch(&dt);
		sprintf (msg, "WTM,%lu", sec);
		com_write_msg(BOARD_COM_PORT, msg);
	}

	if ( type == PMEL_NGN ) { 	// set gain
		uint8_t new_gain = 0;
		sscanf (&buf[4], "%d", &new_gain);
		if( (new_gain < 4 ) && ( new_gain > 0 ) ) {
			config->adc.gain = new_gain;
			printf("PMEL GAIN: %d\r\n", config->adc.gain);
			// set preamp gain
			ioport_set_pin_level(PIN_PREAMP_G0, (config->adc.gain & 0x01));
			ioport_set_pin_level(PIN_PREAMP_G1, (config->adc.gain & 0x02));
		} else {
			status = COM_INVALID_ARG;
			printf("PMEL NGN: invalid gain %d\r\n", new_gain);			
		}
	}

	if ( type == PMEL_PSD ) {	// report SD card memory usage
		uint16_t fft_size; // fft size used for spectrum
		//uint16_t fft_overlap; // fft overlap used for spectrum
		uint16_t duration;  // number of time steps (adc buffer reads) to average
		sscanf (&buf[4], "%u,%u", &fft_size, &duration);
		// check for valid args
		if( ((fft_size == 128) || (fft_size == 512) || (fft_size == 1024) || (fft_size == 2048)) && ((duration > 1) && (duration < PMEL_MAX_PSD_DARATION)) ) {
			float adc_buffer_duration = (float)config->adc.samples_per_buffer / (float)config->adc.sampling_rate; // seconds
			config->psd.size = fft_size;
			config->psd.overlap = 0;
			config->psd.nbins = fft_size / 2;
			config->psd.count = 0; // reset the processing counter
			config->psd.navg = (uint16_t)(duration / adc_buffer_duration); // determine number of buffers to average for psd estimate
			config->mode |= WISPR_PSD;
			printf("PMEL PSD: size=%d, navg=%d\r\n", config->psd.size, config->psd.navg);
		} else {
			status = COM_INVALID_ARG;
			printf("PMEL PSD: invalid arg\r\n");
		}
	}

	// If a valid message was received then send ACK, else send NACK
	if( (type != PMEL_UNKNOWN) && (status == COM_VALID_MSG) ) {
		status = com_write_msg(BOARD_COM_PORT, "ACK");
	} else {
		status = com_write_msg(BOARD_COM_PORT, "NACK");
	}
	
	return (status);
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
		if( type == PMEL_NGN ) {
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
		return(PMEL_NAK);
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
		nwrt += sprintf( &buffer[nwrt], ",%.1f", psd_average[n] );
	}

	// send spectrum to controller, repeat until an ACK is received or quit after 10 tries 
	int count = 10;
	while(count--) {

		uart_write_queue(BOARD_COM_PORT, "@@@\r\n", 5);
	
		// write CRC of the message
		uint16_t crc = (uint16_t)com_CRC(buffer, nwrt);
		//sprintf(msg, "%02x\r\n", crc);
		//status = uart_write_queue(BOARD_COM_PORT, msg, strlen(msg));
		status = uart_write_queue(BOARD_COM_PORT, (uint8_t *)&crc, 2);

		// write the number of bytes in the message
		//sprintf(msg, "%04x\r\n", nwrt);
		//status = uart_write_queue(BOARD_COM_PORT, msg, strlen(msg));
		status = uart_write_queue(BOARD_COM_PORT, (uint8_t *)&nwrt, 2);

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

int pmel_send_status(wispr_config_t *config)
{
	int status = 0;
	char *msg = _buffer;
	float free, amps, volts;

	// get the number of free blocks on sd card
	sd_card_get_free(config->active_sd_card, &free);

	// read battery voltage
	ina260_read(&amps, &volts, 0);

	uint16_t nwrt = 0;
	nwrt = sprintf(msg, "STA");
	nwrt += sprintf(&msg[nwrt], ",%d", config->adc.gain);
	nwrt += sprintf(&msg[nwrt], ",%d", config->adc.sampling_rate);
	nwrt += sprintf(&msg[nwrt], ",%d", config->file_size);
	nwrt += sprintf(&msg[nwrt], ",%.2f", free);
	nwrt += sprintf(&msg[nwrt], ",%.2f", volts );
	nwrt += sprintf(&msg[nwrt], ",%.2f", ina260_mAh);

	// send and repeat until an ACK is received or quit after 10 tries
	int count = 10;
	while(count--) {
		// send message
		com_write_msg(BOARD_COM_PORT, msg);
		// wait 10 seconds for ACK
		status = pmel_wait_for_ack(config, 10);
		if( status == PMEL_ACK ) break;
	}
	
	return(status);
}

int pmel_send_sd_usage(wispr_config_t *config)
{
	int status = 0;
	char *msg = _buffer;
	float free;

	// get the number of free blocks on sd card
	sd_card_get_free(config->active_sd_card, &free);

	uint16_t nwrt = 0;
	nwrt = sprintf(msg, "SDF,%.2f", free);

	// send and repeat until an ACK is received or quit after 10 tries
	int count = 10;
	while(count--) {
		// send message
		com_write_msg(BOARD_COM_PORT, msg);
		// wait 10 seconds for ACK
		status = pmel_wait_for_ack(config, 10);
		if( status == PMEL_ACK ) break;
	}

	// wait 10 seconds for ACK
	status = pmel_wait_for_ack(config, 10);
	if( status == PMEL_ACK ) {
		
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
	if (strncmp (buf, "NAK", 3) == 0) {  // Exit command
		type = PMEL_NAK;
	}
	if (strncmp (buf, "EXI", 3) == 0) {  // Exit command
		type = PMEL_EXI;
	}
	if (strncmp (buf, "RUN", 3) == 0) {  // Run command
		type = PMEL_RUN;
	}
	if (strncmp (buf, "PAU", 3) == 0) {  // Pause command
		type = PMEL_PAU;
	}
	if (strncmp (buf, "RST", 3) == 0) {  // Reset command
		type = PMEL_RST;
	}
	if (strncmp (buf, "SLP", 3) == 0) {  // Sleep command
		type = PMEL_SLP;
	}
	if (strncmp (buf, "STA", 3) == 0) { // Status command
		type = PMEL_STA;
	}
	if (strncmp (buf, "GPS", 3) == 0) {	// GPS message
		type = PMEL_GPS;
	}
	if (strncmp (buf, "TME", 3) == 0) {	// set time
		type = PMEL_TME;
	}
	if (strncmp (buf, "NGN", 3) == 0) { 	// set gain
		type = PMEL_NGN;
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


