/*
 * wispr.c
 *
 * Created: 12/24/2019 5:02:40 AM
 *  Author: Chris
 */ 

#include <stdio.h>
#include "wispr.h"

#define SAM4S

#ifdef SAM4S
#include <gpbr.h>
#endif

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

	hdr->block_size = (buf[19] << 8) | (buf[20] << 0);
	hdr->sample_size = buf[21];
	hdr->samples_per_block = (buf[22] << 8) | (buf[23] << 0);
	hdr->sampling_rate = ((uint32_t)buf[24] << 24) | ((uint32_t)buf[25] << 16) | ((uint32_t)buf[26] << 8) | ((uint32_t)buf[27] << 0);

	return(WISPR_DATA_HEADER_SIZE);
}


int wispr_serialize_data_header(wispr_data_header_t *hdr, uint8_t *buf)
{
	// update the data header info

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
	
	buf[19]   = (uint8_t)(hdr->block_size >> 8);
	buf[20]   = (uint8_t)(hdr->block_size >> 0);
	
	buf[21]  = (uint8_t)(hdr->sample_size);
	
	buf[22]  = (uint8_t)(hdr->samples_per_block >> 8);
	buf[23]  = (uint8_t)(hdr->samples_per_block >> 0);
	
	buf[24]  = (uint8_t)(hdr->sampling_rate >> 24);
	buf[25]  = (uint8_t)(hdr->sampling_rate >> 16);
	buf[26]  = (uint8_t)(hdr->sampling_rate >> 8);
	buf[27]  = (uint8_t)(hdr->sampling_rate >> 0);

	// header checksum 
	uint8_t chksum2 = 0;
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
	fprintf(stdout, "- bytes per block: %d\r\n", hdr->block_size);
	fprintf(stdout, "- bytes per sample: %d\r\n", hdr->sample_size);
	fprintf(stdout, "- samples per block: %d\r\n", hdr->samples_per_block);
	fprintf(stdout, "- sampling rate: %d\r\n", hdr->sampling_rate);
}

//
// 
//
int wispr_gpbr_write_config(wispr_config_t *hdr)
{
	uint32_t reg = 0;

#ifdef SAM4S
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

	reg = (hdr->window << 16);
	reg |= (hdr->interval << 0);
	gpbr_write(GPBR4, reg);

	reg = (hdr->fft_size << 16);
	reg |= (hdr->sample_size << 8);
	reg |= (hdr->active_sd_card << 0);
	gpbr_write(GPBR5, reg);

	reg  = (hdr->settings[7] << 24);
	reg |= (hdr->settings[6] << 16);
	reg |= (hdr->settings[5] << 8);
	reg |= (hdr->settings[4] << 0);
	gpbr_write(GPBR6, reg);

	reg  = (hdr->settings[3] << 24);
	reg |= (hdr->settings[2] << 16);
	reg |= (hdr->settings[1] << 8);
	reg |= (hdr->settings[0] << 0);
	gpbr_write(GPBR7, reg);

#endif

	return(hdr->version[1]);
}

int wispr_gpbr_read_config(wispr_config_t *hdr)
{
	uint32_t reg = 0;

#ifdef SAM4S
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
	hdr->window   = (uint16_t)(reg >> 16);
	hdr->interval = (uint16_t)(reg >> 0);
	
	reg = gpbr_read(GPBR5);
	hdr->fft_size  = (uint16_t)(reg >> 16);
	hdr->sample_size = (uint8_t)(reg >> 8);
	hdr->active_sd_card = (uint8_t)(reg >> 0);

	reg = gpbr_read(GPBR6);
	hdr->settings[7] = (uint8_t)(reg >> 24);
	hdr->settings[6] = (uint8_t)(reg >> 16);
	hdr->settings[5] = (uint8_t)(reg >> 8);
	hdr->settings[4] = (uint8_t)(reg >> 0);

	reg = gpbr_read(GPBR7);
	hdr->settings[3] = (uint8_t)(reg >> 24);
	hdr->settings[2] = (uint8_t)(reg >> 16);
	hdr->settings[1] = (uint8_t)(reg >> 8);
	hdr->settings[0] = (uint8_t)(reg >> 0);
	
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


void wispr_print_config(wispr_config_t *hdr)
{
	float buffer_duration =  (float)hdr->samples_per_block / (float)hdr->sampling_rate;

	fprintf(stdout, "\r\nWISPR %d.%d configuration\r\n", hdr->version[1], hdr->version[0]);
	fprintf(stdout, "- time last changed %s\r\n", epoch_time_string(hdr->epoch));
	fprintf(stdout, "- state %02x, mode %02x\r\n", hdr->state, hdr->mode);
	//fprintf(stdout, "- epoch: %lu\r\n", hdr->epoch);
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
	hdr->state = buf[7];
	hdr->mode = buf[8];

	hdr->epoch = (uint32_t)buf[9]; 
	hdr->epoch |= ((uint32_t)buf[10] << 8); 
	hdr->epoch |= ((uint32_t)buf[11] << 16);
	hdr->epoch |= ((uint32_t)buf[12] << 24);

	hdr->sample_size = buf[13];
	hdr->block_size = buf[14] | (buf[15] << 8);
	hdr->samples_per_block = buf[16] | (buf[17] << 8);
	hdr->sampling_rate = (uint32_t)buf[18] | ((uint32_t)buf[19] << 8) | ((uint32_t)buf[20] << 16) | ((uint32_t)buf[21] << 24);

	hdr->settings[0] = buf[22];
	hdr->settings[1] = buf[23];
	hdr->settings[2] = buf[24];
	hdr->settings[3] = buf[25];
	hdr->settings[4] = buf[26];
	hdr->settings[5] = buf[27];
	hdr->settings[6] = buf[28];
	hdr->settings[7] = buf[29];

	// pull out specific settings
	//hdr->sample_size = hdr->settings[1];
	//hdr->active_sd_card = hdr->settings[0];

	hdr->window = buf[30] | (buf[31] << 8);
	hdr->interval = buf[32] | (buf[33] << 8);
	hdr->fft_size = buf[34] | (buf[35] << 8);

	return(36);
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
	
	buf[7]  = (uint8_t)(hdr->state);
	buf[8]  = (uint8_t)(hdr->mode);

	buf[9]  = (uint8_t)(hdr->epoch >> 0);
	buf[10]  = (uint8_t)(hdr->epoch >> 8);
	buf[11]  = (uint8_t)(hdr->epoch >> 16);
	buf[12] = (uint8_t)(hdr->epoch >> 24);

	buf[13] = (uint8_t)(hdr->sample_size);
	buf[14] = (uint8_t)(hdr->block_size >> 0);
	buf[15] = (uint8_t)(hdr->block_size >> 8);
	buf[16] = (uint8_t)(hdr->samples_per_block >> 0);
	buf[17] = (uint8_t)(hdr->samples_per_block >> 8);
	buf[18] = (uint8_t)(hdr->sampling_rate >> 0);
	buf[19] = (uint8_t)(hdr->sampling_rate >> 8);
	buf[20] = (uint8_t)(hdr->sampling_rate >> 16);
	buf[21] = (uint8_t)(hdr->sampling_rate >> 24);

	// update specific settings
	//hdr->settings[1] = hdr->sample_size;
	//hdr->settings[0] = hdr->active_sd_card;

	buf[22] = (uint8_t)(hdr->settings[0]);
	buf[23] = (uint8_t)(hdr->settings[1]);
	buf[24] = (uint8_t)(hdr->settings[2]);
	buf[25] = (uint8_t)(hdr->settings[3]);
	buf[26] = (uint8_t)(hdr->settings[4]);
	buf[27] = (uint8_t)(hdr->settings[5]);
	buf[28] = (uint8_t)(hdr->settings[6]);
	buf[29] = (uint8_t)(hdr->settings[7]);

	buf[30] = (uint8_t)(hdr->window >> 0);
	buf[31] = (uint8_t)(hdr->window >> 8);
	buf[32] = (uint8_t)(hdr->interval >> 0);
	buf[33] = (uint8_t)(hdr->interval >> 8);
	buf[34] = (uint8_t)(hdr->fft_size >> 0);
	buf[35] = (uint8_t)(hdr->fft_size >> 8);

	// update the data record size
	return(36);
}

void wispr_update_data_header(wispr_config_t *wispr, wispr_data_header_t *hdr)
{	
	// Initialize local data header structure with the config
	// this is used to update the current data buffer header
	hdr->version[0] = wispr->version[0];
	hdr->version[1] = wispr->version[1];
	hdr->settings[0] = wispr->mode;
	hdr->settings[1] = wispr->settings[5];
	hdr->settings[2] = wispr->settings[6];
	hdr->settings[3] = wispr->settings[7];
	hdr->sample_size = wispr->sample_size; // number of bytes per sample
	hdr->block_size = wispr->block_size; // number of bytes in an adc record block
	hdr->samples_per_block = wispr->samples_per_block;  // number of samples in a block
	hdr->sampling_rate = wispr->sampling_rate; // samples per second
	hdr->second = wispr->epoch; // epoch time stamp
	hdr->usec = 0;
	hdr->header_chksum = 0;
	hdr->data_chksum = 0;

}

//
//
//
int wispr_sd_card_parse_header(uint8_t *buf, wispr_sd_card_t *hdr)
{
	if ((buf[0] != 'W') && (buf[1] != 'I') && (buf[2] != 'S') && (buf[3] != 'P') && (buf[4] != 'R')) {
		fprintf(stdout, "wispr_parse_header: unrecognized\r\n");
		return(0);
	}

	uint8_t size = buf[5];
	hdr->version = buf[6];
	hdr->name[0] = buf[7];
	hdr->name[1] = buf[8];
	hdr->name[2] = buf[9];
	hdr->name[3] = buf[10];
	hdr->name[4] = buf[11];
	hdr->name[5] = buf[12];
	hdr->name[6] = buf[13];
	hdr->name[7] = buf[14];

	hdr->start_block  =  (uint32_t)buf[15];    // addr of first block (uint32_t)
	hdr->start_block |= ((uint32_t)buf[16] << 8);
	hdr->start_block |= ((uint32_t)buf[17] << 16);
	hdr->start_block |= ((uint32_t)buf[18] << 24);    // addr of first block (uint32_t)

	hdr->end_block  =  (uint32_t)buf[19];
	hdr->end_block |= ((uint32_t)buf[20] << 8);
	hdr->end_block |= ((uint32_t)buf[21] << 16);
	hdr->end_block |= ((uint32_t)buf[22] << 24);    // addr of last block (uint32_t)

	hdr->write_addr   = (uint32_t)buf[23];
	hdr->write_addr |= ((uint32_t)buf[24] << 8);
	hdr->write_addr |= ((uint32_t)buf[25] << 16);
	hdr->write_addr |= ((uint32_t)buf[26] << 24);    // addr of current write block (uint32_t)
	
	hdr->read_addr  = (uint32_t)buf[27];
	hdr->read_addr |= ((uint32_t)buf[28] << 8);
	hdr->read_addr |= ((uint32_t)buf[29] << 16);
	hdr->read_addr |= ((uint32_t)buf[30] << 24);    // addr of current write block (uint32_t)
	
	hdr->epoch  =  (uint32_t)buf[31];
	hdr->epoch |= ((uint32_t)buf[32] << 8);
	hdr->epoch |= ((uint32_t)buf[33] << 16);
	hdr->epoch |= ((uint32_t)buf[34] << 24);    // addr of current write block (uint32_t)
	return(35);
}

int wispr_sd_card_serialize_header(wispr_sd_card_t *hdr, uint8_t *buf)
{
	buf[0]   = 'W';
	buf[1]   = 'I';
	buf[2]   = 'S';
	buf[3]   = 'P';
	buf[4]   = 'R';

	buf[5]   = 35; // header size

	buf[6]   = (uint8_t)(hdr->version);

	buf[7]   = hdr->name[0];
	buf[8]   = hdr->name[1];
	buf[9]   = hdr->name[2];
	buf[10]  = hdr->name[3];
	buf[11]  = hdr->name[4];
	buf[12]  = hdr->name[5];
	buf[13]  = hdr->name[6];
	buf[14]  = hdr->name[7];

	buf[15]  = (uint8_t)(hdr->start_block >> 0);
	buf[16]  = (uint8_t)(hdr->start_block >> 8);
	buf[17]  = (uint8_t)(hdr->start_block >> 16);
	buf[18]  = (uint8_t)(hdr->start_block >> 24);
	buf[19]  = (uint8_t)(hdr->end_block >> 0);
	buf[20]  = (uint8_t)(hdr->end_block >> 8);
	buf[21]  = (uint8_t)(hdr->end_block >> 16);
	buf[22]  = (uint8_t)(hdr->end_block >> 24);

	buf[23]  = (uint8_t)(hdr->write_addr >> 0);
	buf[24]  = (uint8_t)(hdr->write_addr >> 8);
	buf[25]  = (uint8_t)(hdr->write_addr >> 16);
	buf[26]  = (uint8_t)(hdr->write_addr >> 24);
	buf[27]  = (uint8_t)(hdr->read_addr >> 0);
	buf[28]  = (uint8_t)(hdr->read_addr >> 8);
	buf[29]  = (uint8_t)(hdr->read_addr >> 16);
	buf[30]  = (uint8_t)(hdr->read_addr >> 24);

	buf[31]  = (uint8_t)(hdr->epoch >> 0);
	buf[32]  = (uint8_t)(hdr->epoch >> 8);
	buf[33]  = (uint8_t)(hdr->epoch >> 16);
	buf[34]  = (uint8_t)(hdr->epoch >> 24);

	return(35);
}
