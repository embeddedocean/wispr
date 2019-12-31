/*
 * wispr.c
 *
 * Created: 12/24/2019 5:02:40 AM
 *  Author: Chris
 */ 

#include <stdio.h>
#include "wispr.h"

int wispr_parse_header(uint8_t *buf, wispr_header_t *hdr)
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
	hdr->version[0] = buf[5];
	hdr->version[1] = buf[6];
	hdr->size = buf[7];
	hdr->settings[0] = buf[8];
	hdr->settings[1] = buf[9];
	hdr->bytes_per_sample = buf[10];
	hdr->num_samples = buf[11] | (buf[12] << 8);
	hdr->sampling_rate = (uint32_t)buf[13] | ((uint32_t)buf[14] << 8) | ((uint32_t)buf[15] << 16) | ((uint32_t)buf[16] << 24);
	hdr->year = buf[17];
	hdr->month = buf[18];
	hdr->day = buf[19];
	hdr->hour = buf[20];
	hdr->minute = buf[21];
	hdr->second = buf[22];
	hdr->usec = buf[23] | (buf[24] << 8) | (buf[25] << 16) | (buf[26] << 24);

	return(ADC_HEADER_SIZE);
}


void wispr_update_header(uint8_t *buf, wispr_header_t *hdr)
{
	// update the data header info
	uint8_t chksum2 = 0;

	buf[0]   = 'W';
	buf[1]   = 'I';
	buf[2]   = 'S';
	buf[3]   = 'P';
	buf[4]   = 'R';
	buf[5]   = (uint8_t)(hdr->version[0]);
	buf[6]   = (uint8_t)(hdr->version[1]);
	buf[7]   = (uint8_t)(hdr->size);
	buf[8]   = (uint8_t)(hdr->settings[0]);
	buf[9]   = (uint8_t)(hdr->settings[1]);
	buf[10]  = (uint8_t)(hdr->bytes_per_sample);
	buf[11]  = (uint8_t)(hdr->num_samples >> 0);
	buf[12]  = (uint8_t)(hdr->num_samples >> 8);
	buf[13]  = (uint8_t)(hdr->sampling_rate >> 0);
	buf[14]  = (uint8_t)(hdr->sampling_rate >> 8);
	buf[15]  = (uint8_t)(hdr->sampling_rate >> 16);
	buf[16]  = (uint8_t)(hdr->sampling_rate >> 24);
	buf[17]  = (uint8_t)(hdr->year);
	buf[18]  = (uint8_t)(hdr->month);
	buf[19]  = (uint8_t)(hdr->day);
	buf[20]  = (uint8_t)(hdr->hour);
	buf[21]  = (uint8_t)(hdr->minute);
	buf[22]  = (uint8_t)(hdr->second);
	buf[23]  = (uint8_t)(hdr->usec >> 0);
	buf[24]  = (uint8_t)(hdr->usec >> 8);
	buf[25]  = (uint8_t)(hdr->usec >> 16);
	buf[26]  = (uint8_t)(hdr->usec >> 24);
	// header checksum 
	for(int n=0; n<27; n++) chksum2 += buf[n];
	buf[27]  = chksum2;
	// data checksum
	buf[28]  = hdr->data_chksum; 
	buf[29]  = 0xaa;  // padding
}

void wispr_print_header(wispr_header_t *header)
{
	fprintf(stdout, "-- WISPR %d.%d: ", header->version[0], header->version[1]);
	fprintf(stdout, "%02d/%02d/%02d %02d-%02d-%02d\r\n",
		header->year,header->month,header->day,header->hour,header->minute,header->second);
	fprintf(stdout, " header size: %d\r\n", header->size);
	fprintf(stdout, " settings: %02x %02x \r\n", header->settings[0], header->settings[1]);
	fprintf(stdout, " bytes per sample: %d\r\n", header->bytes_per_sample);
	fprintf(stdout, " samples per buffer: %d\r\n", header->num_samples);
	fprintf(stdout, " sampling rate: %d\r\n", header->sampling_rate);
}
