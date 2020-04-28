/*
 * wispr.c
 *
 * Created: 12/24/2019 5:02:40 AM
 *  Author: Chris
 */ 

#include <stdio.h>
#include <string.h>
#include "wispr.h"

#ifdef __SAM4SD32B__
#include <gpbr.h>
#endif

/*
 * Notes:
 * The SAM4s memory words are little-endian.
 * So everything written to data storage is in little-endian word format.
 * The parse and serialization functions explicitly read/write words 
 * as bytes in l-e order to avoid and cross-platform word alignment issues.
 * Any changes have to be done carefully to preserve the correct buffer indexing.  
 */

int wispr_parse_data_header(uint8_t *buf, wispr_data_header_t *hdr)
{
    if ( strncmp((const char *)buf, "WISPR2", 6) != 0 ) {
      fprintf(stdout, "wispr_parse_data_header: unrecognized header.\n");
      return(0);
	}
	
	hdr->name[0] = buf[0];
	hdr->name[1] = buf[1];
	hdr->name[2] = buf[2];
	hdr->name[3] = buf[3];
	hdr->name[4] = buf[4];
	hdr->name[5] = buf[5];

	hdr->version[0] = buf[6];
	hdr->version[1] = buf[7];

	hdr->type = (uint8_t)buf[8];

	hdr->second  = ((uint32_t)buf[9] << 0);
	hdr->second |= ((uint32_t)buf[10] << 8);
	hdr->second |= ((uint32_t)buf[11] << 16);
	hdr->second |= ((uint32_t)buf[12] << 24);

	hdr->usec  = ((uint32_t)buf[13] << 0);
	hdr->usec |= ((uint32_t)buf[14] << 8);
	hdr->usec |= ((uint32_t)buf[15] << 16);
	hdr->usec |= ((uint32_t)buf[16] << 24);
	
	hdr->settings[0] = buf[17];
	hdr->settings[1] = buf[18];
	hdr->settings[2] = buf[19];
	hdr->settings[3] = buf[20];

	hdr->block_size  = ((uint16_t)buf[21] << 0);
	hdr->block_size |= ((uint16_t)buf[22] << 8);
	
	hdr->sample_size = (uint8_t)buf[23];

	hdr->samples_per_block  = ((uint16_t)buf[24] << 0);
	hdr->samples_per_block |= ((uint16_t)buf[25] << 8);

	hdr->sampling_rate  = ((uint32_t)buf[26] << 0);
	hdr->sampling_rate |= ((uint32_t)buf[27] << 8);
	hdr->sampling_rate |= ((uint32_t)buf[28] << 16);
	hdr->sampling_rate |= ((uint32_t)buf[29] << 24);

	return(WISPR_DATA_HEADER_SIZE);
}


int wispr_serialize_data_header(wispr_data_header_t *hdr, uint8_t *buf)
{
	// update the data header info

	buf[0]  = 'W';
	buf[1]  = 'I';
	buf[2]  = 'S';
	buf[3]  = 'P';
	buf[4]  = 'R';
	buf[5]  = '2';
	
	buf[6]  = (uint8_t)(hdr->version[0]);
	buf[7]  = (uint8_t)(hdr->version[1]);
	
	buf[8]  = (uint8_t)(hdr->type);

	buf[9]  = (uint8_t)(hdr->second >> 0);
	buf[10] = (uint8_t)(hdr->second >> 8);
	buf[11] = (uint8_t)(hdr->second >> 16);
	buf[12] = (uint8_t)(hdr->second >> 24);

	buf[13] = (uint8_t)(hdr->usec >> 0);
	buf[14] = (uint8_t)(hdr->usec >> 8);
	buf[15] = (uint8_t)(hdr->usec >> 16);
	buf[16] = (uint8_t)(hdr->usec >> 24);

	buf[17] = (uint8_t)(hdr->settings[0]);
	buf[18] = (uint8_t)(hdr->settings[1]);
	buf[19] = (uint8_t)(hdr->settings[2]);
	buf[20] = (uint8_t)(hdr->settings[3]);
	
	buf[21] = (uint8_t)(hdr->block_size >> 0);
	buf[22] = (uint8_t)(hdr->block_size >> 8);
	
	buf[23] = (uint8_t)(hdr->sample_size);
	
	buf[24] = (uint8_t)(hdr->samples_per_block >> 0);
	buf[25] = (uint8_t)(hdr->samples_per_block >> 8);
	
	buf[26] = (uint8_t)(hdr->sampling_rate >> 0);
	buf[27] = (uint8_t)(hdr->sampling_rate >> 8);
	buf[28] = (uint8_t)(hdr->sampling_rate >> 16);
	buf[29] = (uint8_t)(hdr->sampling_rate >> 24);

	//buf[30] = (uint8_t)(hdr->channels);

	// header checksum 
	uint8_t chksum2 = 0;
	for(int n=0; n<28; n++) chksum2 += buf[n];
	buf[30]  = chksum2;
	
	// data checksum
	buf[31]  = hdr->data_chksum; 
	
	// update size of record
	return(32);
}

void wispr_print_data_header(wispr_data_header_t *hdr)
{
	fprintf(stdout, "WISPR %d.%d Data Header: ", hdr->version[1], hdr->version[0]);
	fprintf(stdout, "- time: %s\r\n", epoch_time_string(hdr->second));
	fprintf(stdout, "- type: %d\r\n", hdr->type);
	fprintf(stdout, "- settings: %d %d %d %d\r\n",hdr->settings[0],hdr->settings[1],hdr->settings[2],hdr->settings[3]);
	fprintf(stdout, "- bytes per block: %d\r\n", hdr->block_size);
	fprintf(stdout, "- bytes per sample: %d\r\n", hdr->sample_size);
	fprintf(stdout, "- samples per block: %d\r\n", hdr->samples_per_block);
	fprintf(stdout, "- sampling rate: %d\r\n", hdr->sampling_rate);
}


void wispr_print_config(wispr_config_t *hdr)
{
	float buffer_duration =  (float)hdr->samples_per_block / (float)hdr->sampling_rate;

	fprintf(stdout, "\r\n");
	fprintf(stdout, "WISPR %d.%d configuration\r\n", hdr->version[1], hdr->version[0]);
	//fprintf(stdout, "- epoch         %s\r\n", epoch_time_string(hdr->epoch));
	//fprintf(stdout, "- mode           \r\n");
	fprintf(stdout, "- mode           ");
	switch(hdr->mode) {
		case WISPR_WAVEFORM:
		fprintf(stdout, "DAQ");
		break;
		case WISPR_SPECTRUM:
		fprintf(stdout, "PSD");
		break;
		case (WISPR_WAVEFORM|WISPR_SPECTRUM):
		fprintf(stdout, "DAQ/PSD");
		break;
	}
	fprintf(stdout, " (%d)\r\n", hdr->mode);
	fprintf(stdout, "- sample size:   %d bytes\r\n", (int)hdr->sample_size);
	//fprintf(stdout, "- block_size:    %d bytes\r\n", (int)hdr->block_size);
	//fprintf(stdout, "- samples:       %d per buffer\r\n", (int)hdr->samples_per_block);
	fprintf(stdout, "- buffer size:   %d samples (%d bytes)\r\n", (int)hdr->samples_per_block, (int)hdr->block_size);
	fprintf(stdout, "- sampling rate: %d Hz\r\n", (int)hdr->sampling_rate);
	//fprintf(stdout, "- duration:      %lu msec\n\r", (uint32_t)(1000.0*buffer_duration));
	fprintf(stdout, "- gain:          %d\r\n", (int)hdr->gain);
	fprintf(stdout, "- decimation:    %d\r\n", (int)hdr->adc_decimation);
	fprintf(stdout, "- awake time:    %d sec\r\n", (int)hdr->awake_time);
	fprintf(stdout, "- sleep time:    %d sec\r\n", (int)hdr->sleep_time);
	fprintf(stdout, "- fft size:      %d\r\n", (int)hdr->fft_size);
	fprintf(stdout, "- fft overlap:   %d\r\n", (int)hdr->fft_overlap);
	fprintf(stdout, "- active card:   %d\r\n", hdr->active_sd_card);
	fprintf(stdout, "\r\n");
}


int wispr_parse_config(uint8_t *buf, wispr_config_t *hdr)
{
    if ( strncmp((const char *)buf, "WISPR2", 6) != 0 ) {
		fprintf(stdout, "wispr_parse_config: unrecognized config\n");
		return(0);
	}

	hdr->name[0] = buf[0];
	hdr->name[1] = buf[1];
	hdr->name[2] = buf[2];
	hdr->name[3] = buf[3];
	hdr->name[4] = buf[4];
	hdr->name[5] = buf[5];
	
	hdr->version[0] = buf[6];
	hdr->version[1] = buf[7];
	
	hdr->state = buf[8];
	hdr->mode = buf[9];

	hdr->epoch  = ((uint32_t)buf[10]); 
	hdr->epoch |= ((uint32_t)buf[11] << 8); 
	hdr->epoch |= ((uint32_t)buf[12] << 16);
	hdr->epoch |= ((uint32_t)buf[13] << 24);

	hdr->active_sd_card = (uint8_t)buf[14];

	hdr->sample_size = (uint8_t)buf[15];

	hdr->block_size  = ((uint16_t)buf[16]);
	hdr->block_size |= ((uint16_t)buf[17] << 8);
	
	hdr->samples_per_block  = ((uint16_t)buf[18]);
	hdr->samples_per_block |= ((uint16_t)buf[19] << 8);

	hdr->sampling_rate  = ((uint32_t)buf[20]);
	hdr->sampling_rate |= ((uint32_t)buf[21] << 8);
	hdr->sampling_rate |= ((uint32_t)buf[22] << 16);
	hdr->sampling_rate |= ((uint32_t)buf[23] << 24);

	hdr->gain = buf[24];
	hdr->adc_decimation = buf[25];
	// reserved for future use buf[26];
	// reserved for future use buf[27];
	// reserved for future use buf[28];
	// reserved for future use buf[29];
	// reserved for future use buf[30];
	// reserved for future use buf[31];

	hdr->awake_time  = ((uint16_t)buf[32]);
	hdr->awake_time |= ((uint16_t)buf[33] << 8);

	hdr->sleep_time  = ((uint16_t)buf[34]);
	hdr->sleep_time |= ((uint16_t)buf[35] << 8);

	hdr->fft_size  = ((uint16_t)buf[36]);
	hdr->fft_size |= ((uint16_t)buf[37] << 8);

	hdr->fft_overlap  = ((uint16_t)buf[38]);
	hdr->fft_overlap |= ((uint16_t)buf[39] << 8);

	return(40);
}


int wispr_serialize_config(wispr_config_t *hdr, uint8_t *buf)
{
	buf[0]  = 'W';
	buf[1]  = 'I';
	buf[2]  = 'S';
	buf[3]  = 'P';
	buf[4]  = 'R';
	buf[5]  = '2';
		
	buf[6]  = (uint8_t)(hdr->version[0]);
	buf[7]  = (uint8_t)(hdr->version[1]);
	
	buf[8]  = (uint8_t)(hdr->state);
	buf[9]  = (uint8_t)(hdr->mode);

	buf[10] = (uint8_t)(hdr->epoch >> 0);
	buf[11] = (uint8_t)(hdr->epoch >> 8);
	buf[12] = (uint8_t)(hdr->epoch >> 16);
	buf[13] = (uint8_t)(hdr->epoch >> 24);
	
	buf[14] = (uint8_t)(hdr->active_sd_card);
	buf[15] = (uint8_t)(hdr->sample_size);
	
	buf[16] = (uint8_t)(hdr->block_size >> 0);
	buf[17] = (uint8_t)(hdr->block_size >> 8);
	
	buf[18] = (uint8_t)(hdr->samples_per_block >> 0);
	buf[19] = (uint8_t)(hdr->samples_per_block >> 8);
	
	buf[20] = (uint8_t)(hdr->sampling_rate >> 0);
	buf[21] = (uint8_t)(hdr->sampling_rate >> 8);
	buf[22] = (uint8_t)(hdr->sampling_rate >> 16);
	buf[23] = (uint8_t)(hdr->sampling_rate >> 24);

	buf[24] = (uint8_t)(hdr->gain);
	buf[25] = (uint8_t)(hdr->adc_decimation);
	buf[26] = (uint8_t)0;  // reserved
	buf[27] = (uint8_t)0;  // reserved
	buf[28] = (uint8_t)0;  // reserved
	buf[29] = (uint8_t)0;  // reserved
	buf[30] = (uint8_t)0;  // reserved
	buf[31] = (uint8_t)0;  // reserved

	buf[32] = (uint8_t)(hdr->awake_time >> 0);
	buf[33] = (uint8_t)(hdr->awake_time >> 8);
	
	buf[34] = (uint8_t)(hdr->sleep_time >> 0);
	buf[35] = (uint8_t)(hdr->sleep_time >> 8);
	
	buf[36] = (uint8_t)(hdr->fft_size >> 0);
	buf[37] = (uint8_t)(hdr->fft_size >> 8);
	
	buf[38] = (uint8_t)(hdr->fft_overlap >> 0);
	buf[39] = (uint8_t)(hdr->fft_overlap >> 8);

	// update the data record size
	return(40);
}

void wispr_update_data_header(wispr_config_t *wispr, wispr_data_header_t *hdr)
{	
	// Initialize local data header structure with the config
	// this is used to update the current data buffer header
	hdr->version[0] = wispr->version[0];
	hdr->version[1] = wispr->version[1];
	hdr->settings[0] = wispr->gain;
	hdr->settings[1] = wispr->adc_decimation;
	hdr->settings[2] = 0;
	hdr->settings[3] = 0;
	hdr->sample_size = wispr->sample_size; // number of bytes per sample
	hdr->block_size = wispr->block_size; // number of bytes in an adc record block
	hdr->samples_per_block = wispr->samples_per_block;  // number of samples in a block
	hdr->sampling_rate = wispr->sampling_rate; // samples per second
	hdr->second = wispr->epoch; // epoch time stamp
	hdr->usec = 0;
	hdr->header_chksum = 0;
	hdr->data_chksum = 0;
}


#ifdef WINDOWS
void wispr_print_sd_card_header(wispr_sd_card_t *hdr)
{
	fprintf(stdout, "\r\nWISPR %d.%d Card Header\r\n", hdr->version[1], hdr->version[0]);
	fprintf(stdout, "- addr of start block:          %d\r\n", hdr->start);
	fprintf(stdout, "- addr of end block:            %d\r\n", hdr->end);
	fprintf(stdout, "- addr of current write block:  %d\r\n", hdr->write_addr);
	fprintf(stdout, "- addr of current read block:   %d\r\n", hdr->read_addr);
	fprintf(stdout, "- time last header was written: %s\r\n", epoch_time_string(hdr->epoch));
}


int wispr_sd_card_parse_header(uint8_t *buf, wispr_sd_card_t *hdr)
{
    if ( strncmp((const char *)buf, "WISPR", 5) != 0 ) {
		fprintf(stdout, "wispr_sd_card_parse_header: unrecognized header\r\n");
		return(0);
	}

	hdr->version[0] = buf[6];
	hdr->version[1] = buf[7];

	hdr->label[0] = buf[8];
	hdr->label[1] = buf[9];
	hdr->label[2] = buf[10];
	hdr->label[3] = buf[11];
	hdr->label[4] = buf[12];
	hdr->label[5] = buf[13];
	hdr->label[6] = buf[14];
	hdr->label[7] = buf[15];

	hdr->start_block  =  (uint32_t)buf[16];    // addr of first block (uint32_t)
	hdr->start_block |= ((uint32_t)buf[17] << 8);
	hdr->start_block |= ((uint32_t)buf[18] << 16);
	hdr->start_block |= ((uint32_t)buf[19] << 24);    // addr of first block (uint32_t)

	hdr->end_block  =  (uint32_t)buf[20];
	hdr->end_block |= ((uint32_t)buf[21] << 8);
	hdr->end_block |= ((uint32_t)buf[22] << 16);
	hdr->end_block |= ((uint32_t)buf[23] << 24);    // addr of last block (uint32_t)

	hdr->write_addr   = (uint32_t)buf[24];
	hdr->write_addr |= ((uint32_t)buf[25] << 8);
	hdr->write_addr |= ((uint32_t)buf[26] << 16);
	hdr->write_addr |= ((uint32_t)buf[27] << 24);    // addr of current write block (uint32_t)
	
	hdr->read_addr  =  (uint32_t)buf[28];
	hdr->read_addr |= ((uint32_t)buf[29] << 8);
	hdr->read_addr |= ((uint32_t)buf[30] << 16);
	hdr->read_addr |= ((uint32_t)buf[31] << 24);    // addr of current write block (uint32_t)
	
	hdr->epoch  =  (uint32_t)buf[32];
	hdr->epoch |= ((uint32_t)buf[33] << 8);
	hdr->epoch |= ((uint32_t)buf[34] << 16);
	hdr->epoch |= ((uint32_t)buf[35] << 24);    // addr of current write block (uint32_t)
	return(36);
}

int wispr_sd_card_serialize_header(wispr_sd_card_t *hdr, uint8_t *buf)
{
	buf[0]  = 'W';
	buf[1]  = 'I';
	buf[2]  = 'S';
	buf[3]  = 'P';
	buf[4]  = 'R';
	buf[5]  = '2';

	buf[6]  = (uint8_t)(hdr->version[0]);
	buf[7]  = (uint8_t)(hdr->version[1]);

	buf[8]  = hdr->label[0];
	buf[9]  = hdr->label[1];
	buf[10] = hdr->label[2];
	buf[11] = hdr->label[3];
	buf[12] = hdr->label[4];
	buf[13] = hdr->label[5];
	buf[14] = hdr->label[6];
	buf[15] = hdr->label[7];

	buf[16] = (uint8_t)(hdr->start_block >> 0);
	buf[17] = (uint8_t)(hdr->start_block >> 8);
	buf[18] = (uint8_t)(hdr->start_block >> 16);
	buf[19] = (uint8_t)(hdr->start_block >> 24);
	
	buf[20] = (uint8_t)(hdr->end_block >> 0);
	buf[21] = (uint8_t)(hdr->end_block >> 8);
	buf[22] = (uint8_t)(hdr->end_block >> 16);
	buf[23] = (uint8_t)(hdr->end_block >> 24);

	buf[24] = (uint8_t)(hdr->write_addr >> 0);
	buf[25] = (uint8_t)(hdr->write_addr >> 8);
	buf[26] = (uint8_t)(hdr->write_addr >> 16);
	buf[27] = (uint8_t)(hdr->write_addr >> 24);
	
	buf[28] = (uint8_t)(hdr->read_addr >> 0);
	buf[29] = (uint8_t)(hdr->read_addr >> 8);
	buf[30] = (uint8_t)(hdr->read_addr >> 16);
	buf[31] = (uint8_t)(hdr->read_addr >> 24);

	buf[32] = (uint8_t)(hdr->epoch >> 0);
	buf[33] = (uint8_t)(hdr->epoch >> 8);
	buf[34] = (uint8_t)(hdr->epoch >> 16);
	buf[35] = (uint8_t)(hdr->epoch >> 24);

	return(36);
}
#endif

/*
int wispr_gpbr_write_config(wispr_config_t *hdr)
{
	uint32_t reg = 0;

	#ifdef __SAM4SD32B__
	reg  = (WISPR_VERSION << 24); //(hdr->version[1] << 24);
	reg |= (WISPR_SUBVERSION << 16); //(hdr->version[0] << 16);
	reg |= (hdr->state << 8);
	reg |= (hdr->mode << 0);
	gpbr_write(GPBR0, reg);

	reg = (hdr->epoch);
	gpbr_write(GPBR1, reg);

	reg = (hdr->block_size << 16);
	reg |= (hdr->samples_per_block << 0);
	gpbr_write(GPBR2, reg);

	reg = (hdr->sampling_rate);
	gpbr_write(GPBR3, reg);

	reg = (hdr->awake_time << 16);
	reg |= (hdr->sleep_time << 0);
	gpbr_write(GPBR4, reg);

	reg = (hdr->fft_size << 16);
	reg |= (hdr->sample_size << 8);
	reg |= (hdr->active_sd_card << 0);
	gpbr_write(GPBR5, reg);

	//reg  = (hdr->settings[7] << 24);
	//reg |= (hdr->settings[6] << 16);
	//reg |= (hdr->settings[5] << 8);
	//reg |= (hdr->settings[4] << 0);
	reg = 0;
	gpbr_write(GPBR6, reg);

	//reg  = (hdr->settings[3] << 24);
	//reg |= (hdr->settings[2] << 16);
	reg = (hdr->adc_decimation << 8);
	reg |= (hdr->gain << 0);
	gpbr_write(GPBR7, reg);

	#endif

	return(hdr->version[1]);
}

int wispr_gpbr_read_config(wispr_config_t *hdr)
{
	uint32_t reg = 0;

	#ifdef __SAM4SD32B__
	reg = gpbr_read(GPBR0);
	hdr->version[1] = (uint8_t)(reg >> 24);
	hdr->version[0] = (uint8_t)(reg >> 16);
	hdr->state = (uint8_t)(reg >> 8);
	hdr->mode = (uint8_t)(reg >> 0);

	reg = gpbr_read(GPBR1);
	hdr->epoch = (uint32_t)reg;
	
	reg = gpbr_read(GPBR2);
	hdr->block_size   = (uint16_t)(reg >> 16);
	hdr->samples_per_block = (uint16_t)(reg >> 0);
	
	reg = gpbr_read(GPBR3);
	hdr->sampling_rate = reg;
	
	reg = gpbr_read(GPBR4);
	hdr->awake_time   = (uint16_t)(reg >> 16);
	hdr->sleep_time = (uint16_t)(reg >> 0);
	
	reg = gpbr_read(GPBR5);
	hdr->fft_size  = (uint16_t)(reg >> 16);
	hdr->sample_size = (uint8_t)(reg >> 8);
	hdr->active_sd_card = (uint8_t)(reg >> 0);

	reg = gpbr_read(GPBR6);
	//hdr->settings[7] = (uint8_t)(reg >> 24);
	//hdr->settings[6] = (uint8_t)(reg >> 16);
	//hdr->settings[5] = (uint8_t)(reg >> 8);
	//hdr->settings[4] = (uint8_t)(reg >> 0);

	reg = gpbr_read(GPBR7);
	//hdr->settings[3] = (uint8_t)(reg >> 24);
	//hdr->settings[2] = (uint8_t)(reg >> 16);
	hdr->adc_decimation = (uint8_t)(reg >> 8);
	hdr->gain = (uint8_t)(reg >> 0);
	
	// if the version is different, then config has not been set yet
	if( (hdr->version[1] != WISPR_VERSION) || (hdr->version[0] != WISPR_SUBVERSION) ) {
		fprintf(stdout, "\r\nERROR: incorrect configuration version: %d.%d \r\n", hdr->version[1], hdr->version[0]);
		return(0);
	}

	// set specific settings
	#endif
	
	// else return a valid flag
	return(hdr->version[1]);
}
*/

