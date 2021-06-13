/*
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

uint16_t pmel_ack_timeout = PMEL_ACK_TIMEOUT_MSEC;

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
// timeout is in msecs
//
int pmel_control (wispr_config_t *config, uint16_t timeout)
{
	int status = COM_VALID_MSG;
	int type = PMEL_UNKNOWN;
	char buf[COM_MAX_MESSAGE_SIZE];

	// check for a com message 
	status = com_read_msg (BOARD_COM_PORT, buf, timeout);
	if(status == COM_NO_MSG) { // probably timed out
		return(status);
	}
	if( status == COM_VALID_MSG ) { // valid msg format
		type = pmel_msg_type(buf);
		if( type == PMEL_UNKNOWN ) { // unknown type
			return(COM_NO_MSG);		
		}
		printf("pmel control message received: %s, %d bytes\r\n", buf, strlen(buf));
	} else {
		com_write_msg(BOARD_COM_PORT, "NAK");
		return(COM_NO_MSG);
	}

	// Exit command puts system into backup sleep mode
	if ( type == PMEL_EXI ) {
		config->prev_state = config->state;
		config->state = WISPR_SLEEP_BACKUP;
		// The wdt can't be stopped once it's started so 
		// go into backup sleep mode with no wakeup alarm
		config->sleep_time = 0; 
		printf("PMEL EXIT\r\n");
		com_write_msg(BOARD_COM_PORT, "ACK");		
	}

	// Sleep command
	if (type == PMEL_SLP) { 
		status = pmel_set_sleep(config, buf);
		if( status == COM_VALID_MSG ) {
			printf("PMEL SLEEP for %d seconds\r\n", config->sleep_time);
			com_write_msg(BOARD_COM_PORT, "ACK");
		} else {
			com_write_msg(BOARD_COM_PORT, "NAK");
		}
	}
	
	// Run command
	if ( type == PMEL_RUN ) {
		config->prev_state = config->state;
		config->state = WISPR_RUNNING;
		printf("PMEL RUN\r\n");
		com_write_msg(BOARD_COM_PORT, "ACK");
	}
	
	// Pause command
	if ( type == PMEL_PAU ) {
		status = pmel_set_pause(config, buf);
		if( status == COM_VALID_MSG ) {
			printf("PMEL PAUSE for %d seconds\r\n", config->pause_time);
			com_write_msg(BOARD_COM_PORT, "ACK");
		} else {
			com_write_msg(BOARD_COM_PORT, "NAK");
		}
	}
	
	// Reset command
	if ( type == PMEL_RST ) {
		config->prev_state = config->state;
		config->state = WISPR_RESET;
		printf("PMEL RESET\r\n");
		com_write_msg(BOARD_COM_PORT, "ACK");
	}
	
	// Request status command
	if (type == PMEL_STA) {
		com_write_msg(BOARD_COM_PORT, "ACK");
		pmel_send_status(config);
	}

	// Request sd card usage command
	if ( type == PMEL_SDF ) {
		com_write_msg(BOARD_COM_PORT, "ACK");
		pmel_send_sd_usage(config);
	}

	// Request power usage command
	if ( type == PMEL_PWR ) {
		com_write_msg(BOARD_COM_PORT, "ACK");
		pmel_send_power_usage(config);
	}

	// GPS message
	//if ( type == PMEL_GPS ) {
	//	sscanf (&buf[4], "%lu,%f,%f", &config->gps.second, &config->gps.lat, &config->gps.lon);
	//	printf("GPS: sec=%lu, lat=%f, lon=%f \r\n", config->gps.second, config->gps.lat, config->gps.lon);
	//	com_write_msg(BOARD_COM_PORT, "ACK");
	//}
	
	// set time
	if ( type == PMEL_TME ) {
		status = pmel_set_time(config, buf);
		if( status == COM_VALID_MSG ) {
			com_write_msg(BOARD_COM_PORT, "ACK");
		} else {
			com_write_msg(BOARD_COM_PORT, "NAK");
		}
	}
	
	// send time
	if ( type == PMEL_WTM ) {
		com_write_msg(BOARD_COM_PORT, "ACK");
		pmel_send_time(config);
	}

	// set gain
	if ( type == PMEL_NGN ) {
		status = pmel_set_gain(config, buf);		
		if( status == COM_VALID_MSG ) {
			printf("PMEL GAIN: %d\r\n", config->adc.gain);
			com_write_msg(BOARD_COM_PORT, "ACK");			
		} else {
			com_write_msg(BOARD_COM_PORT, "NAK");			
		}

	}

	// request spectrum
	if ( type == PMEL_PSD ) {		
		status = pmel_request_spectrum(config, buf);
		if( status == COM_VALID_MSG ) {
			printf("PMEL PSD: size=%d, navg=%d\r\n", config->psd.size, config->psd.navg);
			com_write_msg(BOARD_COM_PORT, "ACK");
		} else {
			com_write_msg(BOARD_COM_PORT, "NAK");			
		}
	}
	
	// set timeout
	if ( type == PMEL_TOU ) {
		status = pmel_set_timeout(config, buf);
		if( status == COM_VALID_MSG ) {
			printf("PMEL ACK TIMEOUT: %d\r\n", pmel_ack_timeout);
			com_write_msg(BOARD_COM_PORT, "ACK");
		} else {
			com_write_msg(BOARD_COM_PORT, "NAK");
		}
	}

	// set ADC sampling frequency and decimation factor
	if ( type == PMEL_ADC ) {
		status = pmel_set_adc(config, buf);
		if( status == COM_VALID_MSG ) {
			printf("PMEL SET ADC: %d, %d\r\n", config->adc.sampling_rate, config->adc.decimation);
			com_write_msg(BOARD_COM_PORT, "ACK");
		} else {
			com_write_msg(BOARD_COM_PORT, "NAK");
		}
	}

	return (status);
}

//
// Wait for ACK
// - allowing other commands to be received while waiting for ack
//
int pmel_wait_for_ack (wispr_config_t *config)
{
	int type = PMEL_UNKNOWN;
	
	uint16_t count = 0;
	while( count < pmel_ack_timeout ) {
		type = pmel_control(config, 1); // timeout in msecs
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

//
// Send message and wait of ack, resend if no ack within timeout
//
int pmel_send_wait_for_ack(wispr_config_t *config, char *msg)
{
	int status = 0;
	// send and repeat until an ACK is received or quit after multiple tries
	int count = PMEL_NUMBER_RETRIES;
	while(count--) {
		// send message
		com_write_msg(BOARD_COM_PORT, msg);
		// wait for ACK
		status = pmel_wait_for_ack(config);
		if( status == PMEL_ACK ) break;
	}
	return(status);
}


int pmel_set_sleep(wispr_config_t *config, char *buf)
{
	int status = COM_VALID_MSG;
	uint32_t sec;
	size_t len = strlen(buf);
	if( len > 3 ) {
		sscanf (&buf[4], "%lu", &sec);
	} else {
		sec = PMEL_MAX_SLEEP;
	}
	if( (sec > 0) && (sec <= PMEL_MAX_SLEEP) ) {
		config->sleep_time = sec;
		config->prev_state = config->state;
		config->state = WISPR_SLEEP_BACKUP;
	} else {
		status = COM_INVALID_ARG;
		printf("PMEL SLEEP: invalid time %lu\r\n", sec);
		com_write_msg(BOARD_COM_PORT, "NAK");
	}
	return(status);
}

int pmel_set_pause(wispr_config_t *config, char *buf)
{
	int status = COM_VALID_MSG;
	uint32_t sec;
	// scan args
	if( strlen(buf) > 3 ) {
		sscanf (&buf[4], "%lu", &sec);
	} else {
		// if no args, default to max
		sec = PMEL_MAX_PAUSE;
	}
	if( (sec > 0) && (sec <= PMEL_MAX_PAUSE) ) {
		config->pause_time = sec;
		config->prev_state = config->state;
		config->state = WISPR_PAUSED;
	} else {
		status = COM_INVALID_ARG;
	}
	return(status);
}

//
// Request Commands
//

//
// Request and wait for a GPS message
// This will wait until it receives a GPS message, ignoring all other types of messages
// Timeout is in seconds
int pmel_request_gps(wispr_config_t *config, uint16_t timeout)
{
	//char *buf = com_buffer;
	int type = PMEL_NONE;

	//Set RTC time by GPS epoch sec received at COM0. Gain is also changed. HM
	printf("Requesting GPS message to set RTC.\r\n");
	
	//Sends $GPS* que to MPC as a request for GPS time and Location
	com_write_msg(BOARD_COM_PORT, "GPS");
		
	uint16_t count = 0;
	while( count < timeout ) {
		type = pmel_control(config, 1); // timeout in 1000 msecs
		if( type == PMEL_GPS ) {
			break;
		}
		count++;
	}
	return(type);
}

int pmel_request_gain(wispr_config_t *config, uint16_t timeout)
{
	int type = PMEL_NONE;
	
	// request a gain change 
	com_write_msg(BOARD_COM_PORT, "NGN");

	uint16_t count = 0;
	while( count < timeout ) {
		type = pmel_control(config, 1); // timeout in msecs
		if( type == PMEL_NGN ) {
			break;
		}
		count++;
	}
	return(type);
}

int pmel_request_spectrum(wispr_config_t *config, char *buf)
{
	int status = COM_VALID_MSG;
	int fft_size; // fft size used for spectrum
	int duration;  // number of time steps (adc buffer reads) to average

	size_t len = strlen(buf);
	
	// scan input arguments
	if( len > 3 ) {
		sscanf (&buf[4], "%d,%d", &fft_size, &duration);
	} else { // no args, use default
		fft_size = PMEL_FFT_SIZE;
		duration = PMEL_PSD_DURATION;
	}

	// check for valid args
	if( (fft_size != 64) && (fft_size != 128) && (fft_size != 512) && (fft_size != 1024) && (fft_size != 2048) ) {
		printf("PMEL PSD: invalid fft size %d\r\n", fft_size);
		return(COM_INVALID_ARG);
	}
	if( (duration < 1) || (duration >= PMEL_MAX_PSD_DARATION) ) {
		printf("PMEL PSD: invalid duration, %d\r\n", duration);
		return(COM_INVALID_ARG);
	}
	
	//if( (freq < 100) || (freq >= PMEL_MAX_SAMPLING_RATE/2) ) {
	//	printf("PMEL PSD: invalid nyquist freq - using default\r\n");
	//}
	float adc_buffer_duration = (float)config->adc.samples_per_buffer / (float)config->adc.sampling_rate; // seconds
	//config->psd.nyquist = freq;
	config->psd.size = (uint16_t)fft_size;
	config->psd.overlap = 0;
	config->psd.nbins = (uint16_t)fft_size / 2 + 1;
	config->psd.count = 0; // reset the processing counter
	config->psd.navg = (uint16_t)( (float)duration / adc_buffer_duration ); // determine number of buffers to average for psd estimate
	// set PSD mode flag
	config->mode |= WISPR_PSD;
	
	return(status);
}

//
// Set Commands
//
int pmel_set_gain(wispr_config_t *config, char *buf)
{
	int status = COM_VALID_MSG;
	int new_gain = 0;
	// scan input arguments
	if( strlen(buf) > 3 ) {
		sscanf (&buf[4], "%d", &new_gain);
	} else { // no args, use default
		new_gain = PMEL_GAIN;
	}
	if( (new_gain < 4 ) && ( new_gain >= 0 ) ) {
		config->adc.gain = (uint8_t)new_gain;
		// set preamp gain
		ioport_set_pin_level(PIN_PREAMP_G0, (config->adc.gain & 0x01));
		ioport_set_pin_level(PIN_PREAMP_G1, (config->adc.gain & 0x02));
	} else {
		status = COM_INVALID_ARG;
		printf("PMEL NGN: invalid gain %d\r\n", new_gain);
	}
	return(status);
}

int pmel_set_time(wispr_config_t *config, char *buf)
{
	int status = COM_VALID_MSG;
	int arg;
	uint32_t sec;
	rtc_time_t dt;
	// scan args
	if( strlen(buf) > 3 ) {
		sscanf (&buf[4], "%d", &arg);
	} else { // no args, invalid input
		return(COM_INVALID_ARG);
	}
	sec = (uint32_t)arg;
	epoch_to_rtc_time(&dt, sec);
	// If time is valid initialize, set internal and external RTC times
	if( rtc_valid_datetime(&dt) == RTC_STATUS_OK ) {
		// set external rtc
		pcf2129_set_datetime(&dt);
		// set internal rtc
		while ( rtc_init(&dt) != RTC_STATUS_OK ) {
			printf("Waiting for RTC, status %d\r\n", status);
		}
		printf("SET TIME: %s\r\n", epoch_time_string(sec));
	} else {
		status = COM_INVALID_ARG;
		printf("SET TIME: Invalid time, %s\r\n", epoch_time_string(sec));
	}
	return(status);
}

int pmel_set_timeout(wispr_config_t *config, char *buf)
{
	int status = COM_VALID_MSG;
	int new_timeout = 0;
	// scan input arguments
	if( strlen(buf) > 3 ) {
		sscanf (&buf[4], "%d", &new_timeout);
	} else { // no args, use default
		new_timeout = PMEL_ACK_TIMEOUT_MSEC;
	}
	if( new_timeout >= 0  ) {
		pmel_ack_timeout = (uint16_t)new_timeout;
	} else {
		status = COM_INVALID_ARG;
	}
	return(status);
}

//
// Set ADC parameters
// NOTE - you need to stop and start adc to enable new settings
//
int pmel_set_adc(wispr_config_t *config, char *buf)
{
	int status = COM_VALID_MSG;

	int new_fs = PMEL_SAMPLING_RATE;
	int new_df = PMEL_ADC_DECIMATION;

	// scan input arguments
	if( strlen(buf) > 3 ) {
		sscanf (&buf[4], "%d,%d", &new_fs, &new_df);
	}
	
	// set new adc sampling rate
	if( (new_fs > 0) && (new_fs <= PMEL_MAX_SAMPLING_RATE) ) {
		config->adc.sampling_rate = (uint32_t)new_fs;
		// redefine file size in blocks with new sampling rate
		float adc_buffer_duration = (float)config->adc.samples_per_buffer / (float)config->adc.sampling_rate; // seconds
		config->file_size = (uint32_t)(config->secs_per_file / adc_buffer_duration) * ADC_BLOCKS_PER_BUFFER;
	} else {
		return(COM_INVALID_ARG);
	}
	
	// set new adc decimation
	if ( (new_df == 4) || (new_df == 8) || (new_df == 16) || (new_df == 32) ) {
		config->adc.decimation = (uint8_t)new_df;
	} else {
		return(COM_INVALID_ARG);
	}
	
	return(status);
}

//
// Send Commands
//

int pmel_send_spectrum(wispr_config_t *config, float32_t *psd_average, uint16_t nbins, uint8_t *buffer, pmel_control_t *pmel)
{
	int status = 0;
	char *buf = (char *)buffer; // just to avoid warnings

	uint16_t nwrt = 0;
	nwrt += sprintf(&buf[nwrt], "%s", pmel_time_string(config->psd.second));
	nwrt += sprintf(&buf[nwrt], ",%s", pmel->instrument_id);
	nwrt += sprintf(&buf[nwrt], ",%s", pmel->location_id);
	nwrt += sprintf(&buf[nwrt], ",%.2f", pmel->volts ); //
	nwrt += sprintf(&buf[nwrt], ",%.2f", pmel->free);
	nwrt += sprintf(&buf[nwrt], ",%u.%u", pmel->version[0], pmel->version[1]);
	nwrt += sprintf(&buf[nwrt], ",%lu", config->adc.sampling_rate);
	nwrt += sprintf(&buf[nwrt], ",%u", config->psd.nbins);
	nwrt += sprintf(&buf[nwrt], ",%u", config->adc.gain);

	// scale = 2 * log10( (ADC_VREF / 2147483647.0 / (float32_t)config->psd.size) );
	float32_t scale = 2.0 * (log10f(ADC_VREF) - log10f(2147483647.0) - log10f((float32_t)config->psd.size));

	// overwrite the psd buffer with scaled db values
	for(int n = 0; n < nbins; n++) {
		psd_average[n] = 10.0 * (log10f(psd_average[n]) + scale);
		//nwrt += sprintf(&buffer[nwrt], ",%d.%d", (int)psd_average[n], ((int)(100.0*psd_average[n]) - 100*(int)psd_average[n]) );
		nwrt += sprintf( &buf[nwrt], ",%.1f", psd_average[n] );
	}

	// send spectrum to controller, repeat until an ACK is received or quit after 10 tries 
	int count = PMEL_NUMBER_RETRIES;
	while(count--) {

		// write pre-message header with @@@ CRC and length of the message that will follow
		char msg[16];
		uint16_t crc = (uint16_t)com_CRC(buffer, nwrt);
		sprintf(msg, "@@@%02x%04x\r\n", crc, nwrt);
		status = uart_write_queue(BOARD_COM_PORT, (uint8_t *)msg, strlen(msg));

	 	// stream binary data out the uart
		for(int n = 0; n < nwrt; n++) {
			while (!uart_is_tx_empty(BOARD_COM_UART)) {}
			uart_write(BOARD_COM_UART, buffer[n]);
		}
	
		// wait 10 seconds for ACK
		status = pmel_wait_for_ack(config);
		if( status == PMEL_ACK ) break;
	
	}
	
	return(status);
}

int pmel_send_status(wispr_config_t *config)
{
	int status = 0;
	char msg[64];
	float free, amps, volts;

	// get the number of free blocks on sd card
	sd_card_get_free(config->active_sd_card, &free);

	// read battery voltage
	ina260_read(&amps, &volts, 0);

	uint16_t nwrt = 0;
	nwrt = sprintf(msg, "STA");
	nwrt += sprintf(&msg[nwrt], ",%d", config->adc.gain);
	nwrt += sprintf(&msg[nwrt], ",%d", config->adc.sampling_rate);
	nwrt += sprintf(&msg[nwrt], ",%d", config->files);
	nwrt += sprintf(&msg[nwrt], ",%.3f", config->secs_per_file);
	nwrt += sprintf(&msg[nwrt], ",%.2f", volts );
	nwrt += sprintf(&msg[nwrt], ",%.2f", ina260_mAh);
	nwrt += sprintf(&msg[nwrt], ",%d", config->resets);

	// send and wait ACK
	status = pmel_send_wait_for_ack(config, msg);
		
	return(status);
}


int pmel_send_sd_usage(wispr_config_t *config)
{
	int status = 0;
	char msg[16];
	float free;

	// get the number of free blocks on sd card
	sd_card_get_free(config->active_sd_card, &free);

	sprintf(msg, "SDF,%.2f", free);

	// send and wait ACK
	status = pmel_send_wait_for_ack(config, msg);

	return(status);
}

int pmel_send_power_usage(wispr_config_t *config)
{
	int status = 0;
	char msg[32];

	sprintf(msg, "PWR,%.2f,%.2f,%.2f", ina260_V, ina260_mA, ina260_mWh);

	// send and wait ACK
	status = pmel_send_wait_for_ack(config, msg);
	
	return(status);
}

int pmel_send_time(wispr_config_t *config)
{
	int status = 0;
	char msg[32];
	rtc_time_t dt;
	uint32_t sec = 0;
	pcf2129_get_datetime(&dt);  // read time
	sec = rtc_time_to_epoch(&dt);
	sprintf (msg, "WTM,%lu", sec);
	// send and wait for ACK
	status = pmel_send_wait_for_ack(config, msg);
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
	if (strncmp (buf, "WTM", 3) == 0) {	// set time
		type = PMEL_WTM;
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
	if (strncmp (buf, "PWR", 3) == 0) {	// Power report
		type = PMEL_PWR;
	}
	if (strncmp (buf, "TOU", 3) == 0) {	// Set timeout
		type = PMEL_TOU;
	}
	if (strncmp (buf, "ADC", 3) == 0) {	// Set timeout
		type = PMEL_ADC;
	}

	return (type);
}


//----------------------------------------------------------------

char pmel_tstr[32];
char *pmel_time_string(uint32_t epoch)
{
	rtc_time_t tme;
	epoch_to_rtc_time(&tme, epoch);
	sprintf(pmel_tstr, "%02d:%02d:%02d:%02d:%02d:%02d",
		tme.month,tme.day,tme.year,tme.hour,tme.minute,tme.second);
	return(pmel_tstr);
}

void pmel_filename(char *name, char *prefix, char *suffix, rtc_time_t *dt)
{
	//rtc_get_datetime(&dt);
	sprintf(name, "%s_%02d%02d%02d_%02d%02d%02d.%s",
		prefix, dt->year, dt->month, dt->day, dt->hour, dt->minute, dt->second, suffix);
}

//
// Build pmel data file header
// Header is a series of matlab formatted lines that can be read with fgets
// Terminated header with a null
//
int pmel_file_header(char *buf, wispr_config_t *config, wispr_data_header_t *hdr, pmel_control_t *pmel)
{
	int nwrt = 0;
	nwrt += sprintf(&buf[nwrt], "%% WISPR %d.%d\r\n", config->version[1], config->version[0]);
	nwrt += sprintf(&buf[nwrt], "time = '%s';\r\n", pmel_time_string(hdr->second));
	nwrt += sprintf(&buf[nwrt], "intrument_id = '%s';\r\n", pmel->instrument_id);
	nwrt += sprintf(&buf[nwrt], "location_id = '%s';\r\n", pmel->location_id);
	nwrt += sprintf(&buf[nwrt], "volts = %.2f;\r\n", pmel->volts );
	nwrt += sprintf(&buf[nwrt], "blocks_free = %.2f;\r\n", pmel->free);
	nwrt += sprintf(&buf[nwrt], "version = %d.%d;\r\n", pmel->version[0], pmel->version[1]);
	nwrt += sprintf(&buf[nwrt], "number_buffers = %d;\r\n", config->file_size);
	nwrt += sprintf(&buf[nwrt], "buffer_size = %d;\r\n", (int)config->adc.buffer_size);
	nwrt += sprintf(&buf[nwrt], "samples_per_buffer = %d;\r\n", config->adc.samples_per_buffer);
	nwrt += sprintf(&buf[nwrt], "sample_size = %d;\r\n", (int)config->adc.sample_size);
	nwrt += sprintf(&buf[nwrt], "sampling_rate = %d;\r\n", config->adc.sampling_rate);
	nwrt += sprintf(&buf[nwrt], "gain = %d;\r\n", config->adc.gain);
	nwrt += sprintf(&buf[nwrt], "adc_vref = %02f;\r\n", ADC_VREF );
	buf[nwrt] = 0; // null terminate the buffer
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


