/*
 * wispr.c
 *
 * Created: 12/24/2019 5:02:40 AM
 *  Author: Chris
 */ 

#include <stdio.h>
#include "wispr.h"
#include "rtc_time.h"

#include <gpbr.h>


int wispr_parse_data_header(uint8_t *buf, wispr_data_header_t *hdr)
{
    if ((buf[0] != 'W') && (buf[1] != 'I') && (buf[2] != 'S') && (buf[3] != 'P') && (buf[4] != 'R')) {
      fprintf(stdout, "wispr_parse_header: unknown header %c%c%c%c%c\n", buf[0],buf[1],buf[2],buf[3],buf[4]);
      return(0);
	}
	
	hdr->name[0] = buf[0];
	hdr->name[1] = buf[1];
	hdr->name[2] = buf[2];
	hdr->name[3] = buf[3];
	hdr->name[4] = buf[4];

	hdr->version[1] = buf[5];
	hdr->version[0] = buf[6];

	hdr->second = (buf[7] << 24) | (buf[8] << 16) | (buf[9] << 8) | (buf[10] << 0);
	hdr->usec = (buf[11] << 24) | (buf[12] << 16) | (buf[13] << 8) | (buf[14] << 0);
	
	hdr->settings[3] = buf[15];
	hdr->settings[2] = buf[16];
	hdr->settings[1] = buf[17];
	hdr->settings[0] = buf[18];

	hdr->blocks_size = (buf[19] << 8) | (buf[20] << 0);
	hdr->sample_size = buf[21];
	hdr->samples_per_block = (buf[22] << 8) | (buf[23] << 0);
	hdr->sampling_rate = ((uint32_t)buf[24] << 24) | ((uint32_t)buf[25] << 16) | ((uint32_t)buf[26] << 8) | ((uint32_t)buf[27] << 0);

	return(ADC_HEADER_SIZE);
}


int wispr_serialize_data_header(wispr_data_header_t *hdr, uint8_t *buf)
{
	// update the data header info
	uint8_t chksum2 = 0;

	buf[0]   = 'W';
	buf[1]   = 'I';
	buf[2]   = 'S';
	buf[3]   = 'P';
	buf[4]   = 'R';
	
	buf[5]   = (uint8_t)(hdr->version[1]);
	buf[6]   = (uint8_t)(hdr->version[0]);
	
	buf[7]  = (uint8_t)(hdr->second >> 24);
	buf[8]  = (uint8_t)(hdr->second >> 16);
	buf[9]  = (uint8_t)(hdr->second >> 8);
	buf[10]  = (uint8_t)(hdr->second >> 0);

	buf[11]  = (uint8_t)(hdr->usec >> 24);
	buf[12]  = (uint8_t)(hdr->usec >> 16);
	buf[13]  = (uint8_t)(hdr->usec >> 8);
	buf[14]  = (uint8_t)(hdr->usec >> 0);

	buf[15]   = (uint8_t)(hdr->settings[3]);
	buf[16]   = (uint8_t)(hdr->settings[2]);
	buf[17]   = (uint8_t)(hdr->settings[1]);
	buf[18]   = (uint8_t)(hdr->settings[0]);
	
	buf[19]   = (uint8_t)(hdr->blocks_size >> 8);
	buf[20]   = (uint8_t)(hdr->blocks_size >> 0);
	
	buf[21]  = (uint8_t)(hdr->sample_size);
	
	buf[22]  = (uint8_t)(hdr->samples_per_block >> 8);
	buf[23]  = (uint8_t)(hdr->samples_per_block >> 0);
	
	buf[24]  = (uint8_t)(hdr->sampling_rate >> 24);
	buf[25]  = (uint8_t)(hdr->sampling_rate >> 16);
	buf[26]  = (uint8_t)(hdr->sampling_rate >> 8);
	buf[27]  = (uint8_t)(hdr->sampling_rate >> 0);

	// header checksum 
	for(int n=0; n<28; n++) chksum2 += buf[n];
	buf[28]  = chksum2;
	// data checksum
	buf[29]  = hdr->data_chksum; 
	
	// update size of record
	return(30);
}

void wispr_print_data_header(wispr_data_header_t *hdr)
{
	fprintf(stdout, "WISPR %d.%d: ", hdr->version[1], hdr->version[0]);
	fprintf(stdout, "- time: %s\r\n", epoch_time_string(hdr->second));
	fprintf(stdout, "- settings: %02x %02x \r\n", hdr->settings[0], hdr->settings[1]);
	fprintf(stdout, "- bytes per block: %d\r\n", hdr->blocks_size);
	fprintf(stdout, "- bytes per sample: %d\r\n", hdr->sample_size);
	fprintf(stdout, "- samples per block: %d\r\n", hdr->samples_per_block);
	fprintf(stdout, "- sampling rate: %d\r\n", hdr->sampling_rate);
}

//
// 
//
int wispr_write_config(wispr_config_t *hdr)
{
	uint32_t reg = 0;

	reg  = (hdr->version[1] << 24);
	reg |= (hdr->version[0] << 16);
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

	reg  = (hdr->settings[7] << 24);
	reg |= (hdr->settings[6] << 16);
	reg |= (hdr->settings[5] << 8);
	reg |= (hdr->settings[4] << 0);
	gpbr_write(GPBR4, reg);

	reg  = (hdr->settings[3] << 24);
	reg |= (hdr->settings[2] << 16);
	reg |= (hdr->settings[1] << 8);
	reg |= (hdr->settings[0] << 0);
	gpbr_write(GPBR5, reg);

	reg = (hdr->window << 16);
	reg |= (hdr->interval << 0);
	gpbr_write(GPBR6, reg);

	return(hdr->version[1]);
}

int wispr_read_config(wispr_config_t *hdr)
{
	uint32_t reg = 0;
	
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
	hdr->settings[7] = (uint8_t)(reg >> 24);
	hdr->settings[6] = (uint8_t)(reg >> 16);
	hdr->settings[5] = (uint8_t)(reg >> 8);
	hdr->settings[4] = (uint8_t)(reg >> 0);

	reg = gpbr_read(GPBR5);
	hdr->settings[3] = (uint8_t)(reg >> 24);
	hdr->settings[2] = (uint8_t)(reg >> 16);
	hdr->settings[1] = (uint8_t)(reg >> 8);
	hdr->settings[0] = (uint8_t)(reg >> 0);
	
	reg = gpbr_read(GPBR6);
	hdr->window   = (uint16_t)(reg >> 16);
	hdr->interval = (uint16_t)(reg >> 0);
	
	// if the version is different, then config has not been set yet
	if( (hdr->version[1] != WISPR_VERSION) || (hdr->version[0] != WISPR_SUBVERSION) ) {
		return(0);
	}

	// set specific settings
	hdr->sample_size = hdr->settings[1];
	hdr->active_sd_card = hdr->settings[0];
	
	// else return a valid flag 
	return(hdr->version[1]);
}


void wispr_print_config(wispr_config_t *hdr)
{
	float buffer_duration =  (float)hdr->samples_per_block / (float)hdr->sampling_rate;

	fprintf(stdout, "\r\nWISPR %d.%d configuration\r\n", hdr->version[1], hdr->version[0]);
	fprintf(stdout, "- time %s\r\n", epoch_time_string(hdr->epoch));
	fprintf(stdout, "- epoch: %lu\r\n", hdr->epoch);
	fprintf(stdout, "- bytes per sample: %d\r\n", hdr->sample_size);
	fprintf(stdout, "- block size: %d\r\n", hdr->block_size);
	fprintf(stdout, "- samples per block: %d\r\n", hdr->samples_per_block);
	fprintf(stdout, "- sampling rate: %d\r\n", hdr->sampling_rate);
	fprintf(stdout, "- buffer duration: %lu msec\n\r", (uint32_t)(1000.0*buffer_duration));
	fprintf(stdout, "- settings: %02x %02x %02x %02x %02x %02x %02x %02x\r\n", 
		hdr->settings[7], hdr->settings[6],hdr->settings[5], hdr->settings[4],
		hdr->settings[3], hdr->settings[2],hdr->settings[1], hdr->settings[0]);
	fprintf(stdout, "- active sd card: %d\r\n", hdr->active_sd_card);
}


int wispr_parse_config(uint8_t *buf, wispr_config_t *hdr)
{
	if ((buf[0] != 'W') && (buf[1] != 'I') && (buf[2] != 'S') && (buf[3] != 'P') && (buf[4] != 'R')) {
		fprintf(stdout, "wispr_parse_config: unrecognized name\n");
		return(0);
	}

	hdr->name[0] = buf[0];
	hdr->name[1] = buf[1];
	hdr->name[2] = buf[2];
	hdr->name[3] = buf[3];
	hdr->name[4] = buf[4];
	hdr->version[0] = buf[5];
	hdr->version[1] = buf[6];
	//hdr->mode[0] = buf[7];
	//hdr->mode[1] = buf[7];

	hdr->epoch = (uint32_t)buf[7]; 
	hdr->epoch |= ((uint32_t)buf[8] << 8); 
	hdr->epoch |= ((uint32_t)buf[9] << 16);
	hdr->epoch |= ((uint32_t)buf[10] << 24);

	hdr->sample_size = buf[11];
	hdr->block_size = buf[12] | (buf[13] << 8);
	hdr->samples_per_block = buf[14] | (buf[15] << 8);
	hdr->sampling_rate = (uint32_t)buf[16] | ((uint32_t)buf[17] << 8) | ((uint32_t)buf[18] << 16) | ((uint32_t)buf[19] << 24);

	hdr->settings[0] = buf[20];
	hdr->settings[1] = buf[21];
	hdr->settings[2] = buf[22];
	hdr->settings[3] = buf[23];
	hdr->settings[4] = buf[24];
	hdr->settings[5] = buf[25];
	hdr->settings[6] = buf[26];
	hdr->settings[7] = buf[27];

	hdr->window = buf[28] | (buf[29] << 8);
	hdr->interval = buf[30] | (buf[31] << 8);

	return(32);
}


int wispr_serialize_config(wispr_config_t *hdr, uint8_t *buf)
{
	buf[0]  = 'W';
	buf[1]  = 'I';
	buf[2]  = 'S';
	buf[3]  = 'P';
	buf[4]  = 'R';
	buf[5]  = (uint8_t)(hdr->version[0]);
	buf[6]  = (uint8_t)(hdr->version[1]);

	buf[7]  = (uint8_t)(hdr->epoch >> 0);
	buf[8]  = (uint8_t)(hdr->epoch >> 8);
	buf[9]  = (uint8_t)(hdr->epoch >> 16);
	buf[10] = (uint8_t)(hdr->epoch >> 24);

	buf[11] = (uint8_t)(hdr->sample_size);
	buf[12] = (uint8_t)(hdr->block_size >> 0);
	buf[13] = (uint8_t)(hdr->block_size >> 8);
	buf[14] = (uint8_t)(hdr->samples_per_block >> 0);
	buf[15] = (uint8_t)(hdr->samples_per_block >> 8);
	buf[16] = (uint8_t)(hdr->sampling_rate >> 0);
	buf[17] = (uint8_t)(hdr->sampling_rate >> 8);
	buf[18] = (uint8_t)(hdr->sampling_rate >> 16);
	buf[19] = (uint8_t)(hdr->sampling_rate >> 24);

	buf[20] = (uint8_t)(hdr->settings[0]);
	buf[21] = (uint8_t)(hdr->settings[1]);
	buf[22] = (uint8_t)(hdr->settings[2]);
	buf[23] = (uint8_t)(hdr->settings[3]);
	buf[24] = (uint8_t)(hdr->settings[4]);
	buf[25] = (uint8_t)(hdr->settings[5]);
	buf[26] = (uint8_t)(hdr->settings[6]);
	buf[27] = (uint8_t)(hdr->settings[7]);

	buf[28] = (uint8_t)(hdr->window >> 0);
	buf[29] = (uint8_t)(hdr->window >> 8);
	buf[30] = (uint8_t)(hdr->interval >> 0);
	buf[31] = (uint8_t)(hdr->interval >> 8);

	// update the data record size
	return(32);
}



